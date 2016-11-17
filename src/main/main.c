/*
 * This file is part of Ninjaflight.
 *
 * Ninjaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Ninjaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Ninjaflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <platform.h>

#include "build_config.h"
#include "debug.h"

#include "common/axis.h"
#include "common/color.h"
//#include "common/atomic.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/streambuf.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "config/feature.h"

#include "drivers/nvic.h"

#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/dma.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "drivers/serial_uart.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/pwm_mapping.h"
#include "drivers/pwm_rx.h"
#include "drivers/adc.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/inverter.h"
#include "drivers/pwm_output.h"
#include "drivers/flash_m25p16.h"
#include "drivers/sonar_hcsr04.h"
#include "drivers/sdcard.h"
#include "drivers/usb_io.h"
#include "drivers/transponder_ir.h"
#include "drivers/gyro_sync.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "io/serial.h"
#include "io/flashfs.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/display.h"
#include "io/asyncfatfs/asyncfatfs.h"
#include "io/transponder_ir.h"
#include "io/msp.h"
#include "io/serial_msp.h"
#include "io/serial_cli.h"

#include "sensors/sonar.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/initialisation.h"
#include "sensors/instruments.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "flight/anglerate.h"
#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_system.h"
#include "config/feature.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#include "ninjaflight.h"
#include "scheduler.h"

static struct ninja ninja;

// TODO: refactor this to use proper timeouts
extern uint32_t currentTime;
static filterStatePt1_t filteredCycleTimeState;
uint16_t filteredCycleTime;
uint16_t cycleTime;

// TODO: remove when we are done refactoring
struct instruments default_ins;
struct battery default_battery;
struct mixer default_mixer;
struct anglerate default_controller;

extern uint8_t motorControlEnable;

#ifdef SOFTSERIAL_LOOPBACK
serialPort_t *loopbackPort;
#endif

void mixerUsePWMIOConfiguration(struct mixer *self, pwmIOConfiguration_t *pwmIOConfiguration);
void rxInit(modeActivationCondition_t *modeActivationConditions);

void navigationInit(struct pid_config *pidProfile);
const struct sonar_hardware *sonarGetHardwareConfiguration(current_sensor_type_t  currentMeterType);

#ifdef STM32F303xC
// from system_stm32f30x.c
void SetSysClock(void);
#endif
#ifdef STM32F10X
// from system_stm32f10x.c
void SetSysClock(bool overclock);
#endif

typedef enum {
    SYSTEM_STATE_INITIALISING        = 0,
    SYSTEM_STATE_CONFIG_LOADED       = (1 << 0),
    SYSTEM_STATE_SENSORS_READY       = (1 << 1),
    SYSTEM_STATE_MOTORS_READY        = (1 << 2),
    SYSTEM_STATE_TRANSPONDER_ENABLED = (1 << 3),
    SYSTEM_STATE_READY               = (1 << 7)
} systemState_e;

static uint8_t systemState = SYSTEM_STATE_INITIALISING;


static void taskTransponder(void){
	#ifdef TRANSPONDER
	ninja_update_transponder(&ninja);
	#endif
}

static void taskPidLoop(void){
	cycleTime = getTaskDeltaTime(TASK_SELF);

	// Calculate average cycle time and average jitter
    filteredCycleTime = filterApplyPt1(cycleTime, &filteredCycleTimeState, 1, (cycleTime * 0.0000001f));

	// TODO: should we use filtered cycle time for dt or the raw one? Filtered may be better.
	float dT = (float)cycleTime * 0.000001f;
	// prevent zero dt
	if(dT < 0.000001f) dT = 0.0000001f;

	static const int GYRO_WATCHDOG_DELAY=100; // Watchdog for boards without interrupt for gyro
	// getTaskDeltaTime() returns delta time freezed at the moment of entering the scheduler. currentTime is freezed at the very same point.
    // To make busy-waiting timeout work we need to account for time spent within busy-waiting loop
    uint32_t currentDeltaTime = getTaskDeltaTime(TASK_SELF);

    if (imuConfig()->gyroSync) {
		if(gyroSyncCheckUpdate() || ((currentDeltaTime + (micros() - currentTime)) >= (gyro_sync_get_looptime() + GYRO_WATCHDOG_DELAY))) {
			ninja_run_pid_loop(&ninja, dT);
		}
    } else {
		ninja_run_pid_loop(&ninja, dT);
	}
}

cfTask_t cfTasks[TASK_COUNT] = {
    [TASK_SYSTEM] = {
        .taskName = "SYSTEM",
        .taskFunc = taskSystem,
        .desiredPeriod = 1000000 / 10,          // 10 Hz, every 100 ms
        .staticPriority = TASK_PRIORITY_HIGH,
    },

    [TASK_GYROPID] = {
        .taskName = "GYRO/PID",
        .taskFunc = taskPidLoop,
        .desiredPeriod = 1000,                  // every 1 ms
        .staticPriority = TASK_PRIORITY_REALTIME,
    },

    [TASK_ACCEL] = {
        .taskName = "ACCEL",
        .taskFunc = taskUpdateAccelerometer,
        .desiredPeriod = 1000,                  // every 1 ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },

    [TASK_SERIAL] = {
        .taskName = "SERIAL",
        .taskFunc = taskHandleSerial,
        .desiredPeriod = 1000000 / 100,         // 100 Hz should be enough to flush up to 115 bytes @ 115200 baud
        .staticPriority = TASK_PRIORITY_LOW,
    },

#ifdef BEEPER
    [TASK_BEEPER] = {
        .taskName = "BEEPER",
        .taskFunc = taskUpdateBeeper,
        .desiredPeriod = 1000000 / 100,         // 100 Hz, every 10 ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

    [TASK_BATTERY] = {
        .taskName = "BATTERY",
        .taskFunc = taskUpdateBattery,
        .desiredPeriod = 1000000 / 50,          // 50 Hz, every 20 ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },

    [TASK_RX] = {
        .taskName = "RX",
        .checkFunc = taskUpdateRxCheck,
        .taskFunc = taskUpdateRxMain,
        .desiredPeriod = 1000000 / 50,          // If event-based scheduling doesn't work, fallback to periodic scheduling
        .staticPriority = TASK_PRIORITY_HIGH,
    },

#ifdef GPS
    [TASK_GPS] = {
        .taskName = "GPS",
        .taskFunc = taskProcessGPS,
        .desiredPeriod = 1000000 / 10,          // GPS usually don't go faster than 10Hz
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#if USE_MAG == 1
    [TASK_COMPASS] = {
        .taskName = "COMPASS",
        .taskFunc = taskUpdateCompass,
        .desiredPeriod = 1000000 / 10,          // 10 Hz, every 100 ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef BARO
    [TASK_BARO] = {
        .taskName = "BARO",
        .taskFunc = taskUpdateBaro,
        .desiredPeriod = 1000000 / 20,          // 20 Hz, every 50 ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef SONAR
    [TASK_SONAR] = {
        .taskName = "SONAR",
        .taskFunc = taskUpdateSonar,
        .desiredPeriod = 70000,                 // every 70 ms, approximately 14 Hz
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#if defined(BARO) || defined(SONAR)
    [TASK_ALTITUDE] = {
        .taskName = "ALTITUDE",
        .taskFunc = taskCalculateAltitude,
        .desiredPeriod = 1000000 / 40,          // 40 Hz, every 25 ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

    [TASK_TRANSPONDER] = {
        .taskName = "TRANSPONDER",
        .taskFunc = taskTransponder,
        .desiredPeriod = 1000000 / 250,         // 250 Hz, every 4 ms
        .staticPriority = TASK_PRIORITY_LOW,
    },

#ifdef DISPLAY
    [TASK_DISPLAY] = {
        .taskName = "DISPLAY",
        .taskFunc = taskUpdateDisplay,
        .desiredPeriod = 1000000 / 10,          // 10 Hz, every 100 ms
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef TELEMETRY
    [TASK_TELEMETRY] = {
        .taskName = "TELEMETRY",
        .taskFunc = taskTelemetry,
        .desiredPeriod = 1000000 / 250,         // 250 Hz, every 4 ms
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif

#ifdef LED_STRIP
    [TASK_LEDSTRIP] = {
        .taskName = "LEDSTRIP",
        .taskFunc = taskLedStrip,
        .desiredPeriod = 1000000 / 100,         // 100 Hz, every 10 ms
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif
};
static void flashLedsAndBeep(void)
{
    led_on(1);
    led_off(0);
    for (uint8_t i = 0; i < 10; i++) {
        led_toggle(1);
        led_toggle(0);
        usleep(25000);
        BEEP_ON;
        usleep(25000);
        BEEP_OFF;
    }
    led_off(0);
    led_off(1);
}

#ifdef BUTTONS
static void buttonsInit(void)
{

    gpio_config_t buttonAGpioConfig = {
        BUTTON_A_PIN,
        Mode_IPU,
        Speed_2MHz
    };
    gpioInit(BUTTON_A_PORT, &buttonAGpioConfig);

    gpio_config_t buttonBGpioConfig = {
        BUTTON_B_PIN,
        Mode_IPU,
        Speed_2MHz
    };
    gpioInit(BUTTON_B_PORT, &buttonBGpioConfig);

    usleep(10);  // allow GPIO configuration to settle
}

static void buttonsHandleColdBootButtonPresses(void)
{
    uint8_t secondsRemaining = 10;
    bool bothButtonsHeld;
    do {
        bothButtonsHeld = !digitalIn(BUTTON_A_PORT, BUTTON_A_PIN) && !digitalIn(BUTTON_B_PORT, BUTTON_B_PIN);
        if (bothButtonsHeld) {
            if (--secondsRemaining == 0) {
                resetEEPROM();
                systemReset();
            }

            if (secondsRemaining > 5) {
                usleep(1000000UL);
            } else {
                // flash quicker after a few seconds
                usleep(500000);
                led_toggle(0);
                usleep(500000);
            }
            led_toggle(0);
        }
    } while (bothButtonsHeld);

    // buttons released between 5 and 10 seconds
    if (secondsRemaining < 5) {

        usbGenerateDisconnectPulse();

        flashLedsAndBeep();

        systemResetToBootloader();
    }
}

#endif

static void init(void)
{
    drv_pwm_config_t pwm_params;

    printfSupportInit();

    initEEPROM();

    ensureEEPROMContainsValidData();
    readEEPROM();

    systemState |= SYSTEM_STATE_CONFIG_LOADED;

#ifdef STM32F303
    // start fpu
    SCB->CPACR = (0x3 << (10*2)) | (0x3 << (11*2));
#endif

#ifdef STM32F303xC
    SetSysClock();
#endif
#ifdef STM32F10X
    // Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers
    // Configure the Flash Latency cycles and enable prefetch buffer
    SetSysClock(systemConfig()->emf_avoidance);
#endif
    i2cSetOverclock(systemConfig()->i2c_highspeed);

    systemInit();

#ifdef USE_HARDWARE_REVISION_DETECTION
    detectHardwareRevision();
#endif

    // Latch active features to be used for feature() in the remainder of init().
    latchActiveFeatures();
#ifdef ALIENFLIGHTF3
    if (hardwareRevision == AFF3_REV_1) {
        led_init(false);
    } else {
        led_init(true);
    }
#else
    led_init(false);
#endif

#ifdef BEEPER
    beeperConfig_t beeperConfig = {
        .gpioPeripheral = BEEP_PERIPHERAL,
        .gpioPin = BEEP_PIN,
        .gpioPort = BEEP_GPIO,
#ifdef BEEPER_INVERTED
        .gpioMode = Mode_Out_PP,
        .isInverted = true
#else
        .gpioMode = Mode_Out_OD,
        .isInverted = false
#endif
    };
#ifdef NAZE
    if (hardwareRevision >= NAZE32_REV5) {
        // naze rev4 and below used opendrain to PNP for buzzer. Rev5 and above use PP to NPN.
        beeperConfig.gpioMode = Mode_Out_PP;
        beeperConfig.isInverted = true;
    }
#endif

    beeperInit(&beeperConfig);
#endif

#ifdef BUTTONS
    buttonsInit();

    if (!isMPUSoftReset()) {
        buttonsHandleColdBootButtonPresses();
    }
#endif

#ifdef SPEKTRUM_BIND
    if (feature(FEATURE_RX_SERIAL)) {
        switch (rxConfig()->serialrx_provider) {
            case SERIALRX_SPEKTRUM1024:
            case SERIALRX_SPEKTRUM2048:
                // Spektrum satellite binding if enabled on startup.
                // Must be called before that 100ms sleep so that we don't lose satellite's binding window after startup.
                // The rest of Spektrum initialization will happen later - via spektrumInit()
                spektrumBind(rxConfig());
                break;
			default:
				break;
        }
    }
#endif

    usleep(100000);

    timerInit();  // timer must be initialized before any channel is allocated

    dmaInit();


    serialInit(feature(FEATURE_SOFTSERIAL));

    mixer_init(&default_mixer,
		mixerConfig(),
		motor3DConfig(),
		motorAndServoConfig(),
		rxConfig(),
		rcControlsConfig(),
		servoProfile()->servoConf,
		customMotorMixer(0), MAX_SUPPORTED_MOTORS);

    memset(&pwm_params, 0, sizeof(pwm_params));

#ifdef SONAR

    if (feature(FEATURE_SONAR)) {
		sonar_init(&default_sonar);
		// TODO: fix this
		/*
        sonarHardware = sonarGetHardwareConfiguration(batteryConfig()->currentMeterType);
        sonarGPIOConfig_t sonarGPIOConfig = {
            .gpio = SONAR_GPIO,
            .triggerPin = sonarHardware->echo_pin,
            .echoPin = sonarHardware->trigger_pin,
        };
        pwm_params.sonarGPIOConfig = &sonarGPIOConfig;
		*/
    }
#endif

    // when using airplane/wing mixer, servo/motor outputs are remapped
    if (mixerConfig()->mixerMode == MIXER_AIRPLANE || mixerConfig()->mixerMode == MIXER_FLYING_WING || mixerConfig()->mixerMode == MIXER_CUSTOM_AIRPLANE)
        pwm_params.airplane = true;
    else
        pwm_params.airplane = false;
#if defined(USE_UART2) && defined(STM32F10X)
    pwm_params.useUART2 = doesConfigurationUsePort(SERIAL_PORT_UART2);
#endif
#if defined(USE_UART3)
    pwm_params.useUART3 = doesConfigurationUsePort(SERIAL_PORT_UART3);
#endif
#if defined(USE_UART4)
    pwm_params.useUART4 = doesConfigurationUsePort(SERIAL_PORT_UART4);
#endif
#if defined(USE_UART5)
    pwm_params.useUART5 = doesConfigurationUsePort(SERIAL_PORT_UART5);
#endif
    pwm_params.useVbat = feature(FEATURE_VBAT);
    pwm_params.useSoftSerial = feature(FEATURE_SOFTSERIAL);
    pwm_params.useParallelPWM = feature(FEATURE_RX_PARALLEL_PWM);
    pwm_params.useRSSIADC = feature(FEATURE_RSSI_ADC);
    pwm_params.useCurrentMeterADC = (
        feature(FEATURE_CURRENT_METER)
        && batteryConfig()->currentMeterType == CURRENT_SENSOR_ADC
    );
    pwm_params.useLEDStrip = feature(FEATURE_LED_STRIP);
    pwm_params.usePPM = feature(FEATURE_RX_PPM);
    pwm_params.useSerialRx = feature(FEATURE_RX_SERIAL);
#ifdef SONAR
    pwm_params.useSonar = feature(FEATURE_SONAR);
#endif

#ifdef USE_SERVOS
    pwm_params.useServos = mixer_get_servo_count(&default_mixer) || feature(FEATURE_SERVO_TILT);
    pwm_params.useChannelForwarding = feature(FEATURE_CHANNEL_FORWARDING);
    pwm_params.servoCenterPulse = motorAndServoConfig()->servoCenterPulse;
    pwm_params.servoPwmRate = motorAndServoConfig()->servo_pwm_rate;
#endif

    pwm_params.useOneshot = feature(FEATURE_ONESHOT125);
    pwm_params.motorPwmRate = motorAndServoConfig()->motor_pwm_rate;
    pwm_params.idlePulse = motorAndServoConfig()->mincommand;
    if (feature(FEATURE_3D))
        pwm_params.idlePulse = motor3DConfig()->neutral3d;
    if (pwm_params.motorPwmRate > 500)
        pwm_params.idlePulse = 0; // brushed motors

    pwmRxInit();

    // pwmInit() needs to be called as soon as possible for ESC compatibility reasons
	pwmInit(&pwm_params);
    //pwmIOConfiguration_t *pwmIOConfiguration = pwmInit(&pwm_params);
	//mixer_use_pwmio_config(&default_mixer, pwmIOConfiguration);

#ifdef DEBUG_PWM_CONFIGURATION
    debug[2] = pwmIOConfiguration->pwmInputCount;
    debug[3] = pwmIOConfiguration->ppmInputCount;
#endif

    if (!feature(FEATURE_ONESHOT125))
        motorControlEnable = true;

    systemState |= SYSTEM_STATE_MOTORS_READY;

#ifdef INVERTER
    initInverter();
#endif


#ifdef USE_SPI
    spiInit(SPI1);
    spiInit(SPI2);
#ifdef STM32F303xC
#ifdef ALIENFLIGHTF3
    if (hardwareRevision == AFF3_REV_2) {
        spiInit(SPI3);
    }
#else
    spiInit(SPI3);
#endif
#endif
#endif

#ifdef USE_HARDWARE_REVISION_DETECTION
    updateHardwareRevision();
#endif

#if defined(NAZE)
    if (hardwareRevision == NAZE32_SP) {
        serialRemovePort(SERIAL_PORT_SOFTSERIAL2);
    } else  {
        serialRemovePort(SERIAL_PORT_UART3);
    }
#endif

#if defined(SPRACINGF3) && defined(SONAR) && defined(USE_SOFTSERIAL2)
    if (feature(FEATURE_SONAR) && feature(FEATURE_SOFTSERIAL)) {
        serialRemovePort(SERIAL_PORT_SOFTSERIAL2);
    }
#endif

#if defined(SPRACINGF3MINI) && defined(SONAR) && defined(USE_SOFTSERIAL1)
    if (feature(FEATURE_SONAR) && feature(FEATURE_SOFTSERIAL)) {
        serialRemovePort(SERIAL_PORT_SOFTSERIAL1);
    }
#endif


#ifdef USE_I2C
#if defined(NAZE)
    if (hardwareRevision != NAZE32_SP) {
        i2cInit(I2C_DEVICE);
    } else {
        if (!doesConfigurationUsePort(SERIAL_PORT_UART3)) {
            i2cInit(I2C_DEVICE);
        }
    }
#elif defined(CC3D)
    if (!doesConfigurationUsePort(SERIAL_PORT_UART3)) {
        i2cInit(I2C_DEVICE);
    }
#else
    i2cInit(I2C_DEVICE);
#endif
#endif

#ifdef USE_ADC
    drv_adc_config_t adc_params;

    adc_params.enableVBat = feature(FEATURE_VBAT);
    adc_params.enableRSSI = feature(FEATURE_RSSI_ADC);
    adc_params.enableCurrentMeter = feature(FEATURE_CURRENT_METER);
    adc_params.enableExternal1 = false;
#ifdef OLIMEXINO
    adc_params.enableExternal1 = true;
#endif
#ifdef NAZE
    // optional ADC5 input on rev.5 hardware
    adc_params.enableExternal1 = (hardwareRevision >= NAZE32_REV5);
#endif

    adcInit(&adc_params);
#endif

#ifdef DISPLAY
    if (feature(FEATURE_DISPLAY)) {
        displayInit();
    }
#endif

    gyroSetSampleRate(imuConfig()->looptime, gyroConfig()->gyro_lpf, imuConfig()->gyroSync, imuConfig()->gyroSyncDenominator);   // Set gyro sampling rate divider before initialization

    if (!sensorsAutodetect()) {
        // if gyro was not detected due to whatever reason, we give up now.
        failureMode(FAILURE_MISSING_ACC);
    }
	
    if (USE_MAG && sensors(SENSOR_MAG))
    	mag.init();

	ins_init(&default_ins, 
		boardAlignment(),
		imuConfig(),
		throttleCorrectionConfig(),
		gyroConfig(),
		compassConfig(),
		sensorTrims(),
		accelerometerConfig(),
		gyro.scale,
		acc.acc_1G
	);

	ins_set_gyro_alignment(&default_ins, gyroAlign);
	ins_set_acc_alignment(&default_ins, accAlign);
	ins_set_mag_alignment(&default_ins, magAlign);

#ifdef GPS
    if (feature(FEATURE_GPS)) {
        gpsInit();
        navigationInit(pidProfile());
    }
#endif

    systemState |= SYSTEM_STATE_SENSORS_READY;

    flashLedsAndBeep();

    mspInit();
    mspSerialInit();

#ifdef USE_CLI
    cliInit();
#endif

	// is this ok here?
	anglerate_init(&default_controller,
		&default_ins,
		pidProfile(),
		currentControlRateProfile,
		imuConfig()->max_angle_inclination,
		&accelerometerConfig()->trims,
		rxConfig()
	);

    failsafeInit();

    rxInit(modeActivationProfile()->modeActivationConditions);

#ifdef SONAR
    if (feature(FEATURE_SONAR)) {
        sonar_init(&default_sonar);
    }
#endif

#ifdef LED_STRIP
    ledStripInit();

    if (feature(FEATURE_LED_STRIP)) {
        ledStripEnable();
    }
#endif

#ifdef TELEMETRY
    if (feature(FEATURE_TELEMETRY)) {
        telemetryInit();
    }
#endif

#ifdef USB_CABLE_DETECTION
    usbCableDetectInit();
#endif

#ifdef TRANSPONDER
    if (feature(FEATURE_TRANSPONDER)) {
        transponderInit(transponderConfig()->data);
        transponderEnable();
        transponderStartRepeating();
        systemState |= SYSTEM_STATE_TRANSPONDER_ENABLED;
    }
#endif

#ifdef USE_FLASHFS
#ifdef NAZE
    if (hardwareRevision == NAZE32_REV5) {
        m25p16_init();
    }
#elif defined(USE_FLASH_M25P16)
    m25p16_init();
#endif

    flashfsInit();
#endif

#ifdef USE_SDCARD
    bool sdcardUseDMA = false;

    sdcardInsertionDetectInit();

#ifdef SDCARD_DMA_CHANNEL_TX

#if defined(LED_STRIP) && defined(WS2811_DMA_CHANNEL)
    // Ensure the SPI Tx DMA doesn't overlap with the led strip
    sdcardUseDMA = !feature(FEATURE_LED_STRIP) || SDCARD_DMA_CHANNEL_TX != WS2811_DMA_CHANNEL;
#else
    sdcardUseDMA = true;
#endif

#endif

    sdcard_init(sdcardUseDMA);

    afatfs_init();
#endif

#ifdef BLACKBOX
    initBlackbox();
#endif
/*
    if (mixerConfig()->mixerMode == MIXER_GIMBAL) {
        accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
    }
    gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
#ifdef BARO
    baroSetCalibrationCycles(CALIBRATING_BARO_CYCLES);
#endif
*/
    // start all timers
    // TODO - not implemented yet
    timerStart();

    ENABLE_STATE(SMALL_ANGLE);
    DISABLE_ARMING_FLAG(PREVENT_ARMING);

#ifdef SOFTSERIAL_LOOPBACK
    // FIXME this is a hack, perhaps add a FUNCTION_LOOPBACK to support it properly
    loopbackPort = (serialPort_t*)&(softSerialPorts[0]);
    if (!loopbackPort->vTable) {
        loopbackPort = openSoftSerial(0, NULL, 19200, SERIAL_NOT_INVERTED);
    }
    serialPrint(loopbackPort, "LOOPBACK\r\n");
#endif

    // Now that everything has powered up the voltage and cell count be determined.

    if (feature(FEATURE_VBAT | FEATURE_CURRENT_METER))
        battery_init(&default_battery, batteryConfig());

#ifdef DISPLAY
    if (feature(FEATURE_DISPLAY)) {
#ifdef USE_OLED_GPS_DEBUG_PAGE_ONLY
        displayShowFixedPage(PAGE_GPS);
#else
        displayResetPageCycling();
        displayEnablePageCycling();
#endif
    }
#endif

#ifdef CJMCU
    led_on(2);
#endif

	ninja_init(&ninja);

    // Latch active features AGAIN since some may be modified by init().
    latchActiveFeatures();
    motorControlEnable = true;

    systemState |= SYSTEM_STATE_READY;
}

#ifdef SOFTSERIAL_LOOPBACK
void processLoopback(void) {
    if (loopbackPort) {
        uint8_t bytesWaiting;
        while ((bytesWaiting = serialRxBytesWaiting(loopbackPort))) {
            uint8_t b = serialRead(loopbackPort);
            serialWrite(loopbackPort, b);
        };
    }
}
#else
#define processLoopback()
#endif

int main(void) {
    init();

    // Setup scheduler
    schedulerInit();
    setTaskEnabled(TASK_GYROPID, true);
    rescheduleTask(TASK_GYROPID, imuConfig()->gyroSync ? gyro_sync_get_looptime() - INTERRUPT_WAIT_TIME : gyro_sync_get_looptime());
    setTaskEnabled(TASK_ACCEL, sensors(SENSOR_ACC));
    setTaskEnabled(TASK_SERIAL, true);
#ifdef BEEPER
    setTaskEnabled(TASK_BEEPER, true);
#endif
    setTaskEnabled(TASK_BATTERY, feature(FEATURE_VBAT) || feature(FEATURE_CURRENT_METER));
    setTaskEnabled(TASK_RX, true);
#ifdef GPS
    setTaskEnabled(TASK_GPS, feature(FEATURE_GPS));
#endif
#if USE_MAG == 1
	setTaskEnabled(TASK_COMPASS, sensors(SENSOR_MAG));
	#if defined(MPU6500_SPI_INSTANCE) && defined(USE_MAG_AK8963)
		// fixme temporary solution for AK6983 via slave I2C on MPU9250
		rescheduleTask(TASK_COMPASS, 1000000 / 40);
	#endif
#endif
#ifdef BARO
    setTaskEnabled(TASK_BARO, sensors(SENSOR_BARO));
#endif
#ifdef SONAR
    setTaskEnabled(TASK_SONAR, sensors(SENSOR_SONAR));
#endif
#if defined(BARO) || defined(SONAR)
    setTaskEnabled(TASK_ALTITUDE, sensors(SENSOR_BARO) || sensors(SENSOR_SONAR));
#endif
#ifdef DISPLAY
    setTaskEnabled(TASK_DISPLAY, feature(FEATURE_DISPLAY));
#endif
#ifdef TELEMETRY
    setTaskEnabled(TASK_TELEMETRY, feature(FEATURE_TELEMETRY));
#endif
#ifdef LED_STRIP
    setTaskEnabled(TASK_LEDSTRIP, feature(FEATURE_LED_STRIP));
#endif
#ifdef TRANSPONDER
    setTaskEnabled(TASK_TRANSPONDER, feature(FEATURE_TRANSPONDER));
#endif

    while (true) {
        scheduler();
        processLoopback();
    }
}

void HardFault_Handler(void);
void HardFault_Handler(void)
{
    // fall out of the sky
    uint8_t requiredStateForMotors = SYSTEM_STATE_CONFIG_LOADED | SYSTEM_STATE_MOTORS_READY;
    if ((systemState & requiredStateForMotors) == requiredStateForMotors) {
		// TODO: there was a wierd dependency on motorCount value that was set for the mixer
		// debug the motorCount so that pwm always knows it implicitly
		for(int c = 0; c < MAX_SUPPORTED_MOTORS; c++){
			// we may write below mincommand but in this case it should be ok
			// also we write this to all motors
			pwmWriteMotor(c, 1000);
		}
    }
#ifdef TRANSPONDER
    // prevent IR LEDs from burning out.
    uint8_t requiredStateForTransponder = SYSTEM_STATE_CONFIG_LOADED | SYSTEM_STATE_TRANSPONDER_ENABLED;
    if ((systemState & requiredStateForTransponder) == requiredStateForTransponder) {
        transponderIrDisable();
    }
#endif

    while (1);
}
