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

#include "system_calls.h"
#include "build_config.h"
#include "debug.h"

#include "common/axis.h"
#include "common/color.h"
//#include "common/atomic.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/streambuf.h"

#include "config/feature.h"

#include "drivers/nvic.h"

#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/dma.h"
#include "drivers/gpio.h"
#include "drivers/config_flash.h"
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
#include "drivers/flashfs.h"
#include "drivers/asyncfatfs/asyncfatfs.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "io/serial.h"
#include "io/ledstrip.h"
#include "io/display.h"
#include "io/transponder_ir.h"
#include "io/serial_msp.h"

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

#include "flight/anglerate.h"
#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"

#include "config/config.h"
#include "config/feature.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include "ninjaflight.h"

// TODO: refactor this to use proper timeouts
extern uint32_t currentTime;
/*
static filterStatePt1_t filteredCycleTimeState;
uint16_t filteredCycleTime;
uint16_t cycleTime;
*/
// TODO: remove when we are done refactoring
//struct instruments default_ins;
//struct battery default_battery;
//struct mixer default_mixer;
//struct anglerate default_controller;

#ifdef SOFTSERIAL_LOOPBACK
serialPort_t *loopbackPort;
#endif

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

//static const struct system_calls system_calls = {0};

static void pre_init(const struct config *config){
	printfSupportInit();

    //initEEPROM();

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
    SetSysClock(config->system.emf_avoidance);
#endif

    // Latch active features to be used for feature() in the remainder of init().
    //latchActiveFeatures();
    i2cSetOverclock(config->system.i2c_highspeed);

    systemInit();

#ifdef USE_HARDWARE_REVISION_DETECTION
    detectHardwareRevision();
#endif

#ifdef ALIENFLIGHTF3
    if (hardwareRevision == AFF3_REV_1) {
        led_init(false);
    } else {
        led_init(true);
    }
#else
    led_init(false);
#endif
}

static void init(const struct config *config)
{
    drv_pwm_config_t pwm_params;

    if(USE_BEEPER){
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
	}

#ifdef BUTTONS
    buttonsInit();

    if (!isMPUSoftReset()) {
        buttonsHandleColdBootButtonPresses();
    }
#endif

    usleep(100000);

    timerInit();  // timer must be initialized before any channel is allocated

    dmaInit();

    serialInit(feature(config, FEATURE_SOFTSERIAL));

    memset(&pwm_params, 0, sizeof(pwm_params));

#ifdef SONAR
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
#endif

    // when using airplane/wing mixer, servo/motor outputs are remapped
    if (config->mixer.mixerMode == MIXER_AIRPLANE || config->mixer.mixerMode == MIXER_FLYING_WING || config->mixer.mixerMode == MIXER_CUSTOM_AIRPLANE)
        pwm_params.airplane = true;
    else
        pwm_params.airplane = false;
#if defined(USE_UART2) && defined(STM32F10X)
    pwm_params.useUART2 = doesConfigurationUsePort(&config->serial, SERIAL_PORT_UART2);
#endif
#if defined(USE_UART3)
    pwm_params.useUART3 = doesConfigurationUsePort(&config->serial, SERIAL_PORT_UART3);
#endif
#if defined(USE_UART4)
    pwm_params.useUART4 = doesConfigurationUsePort(&config->serial, SERIAL_PORT_UART4);
#endif
#if defined(USE_UART5)
    pwm_params.useUART5 = doesConfigurationUsePort(&config->serial, SERIAL_PORT_UART5);
#endif
    pwm_params.useVbat = feature(config, FEATURE_VBAT);
    pwm_params.useSoftSerial = feature(config, FEATURE_SOFTSERIAL);
    pwm_params.useParallelPWM = feature(config, FEATURE_RX_PARALLEL_PWM);
    pwm_params.useRSSIADC = feature(config, FEATURE_RSSI_ADC);
    pwm_params.useCurrentMeterADC = (
        feature(config, FEATURE_CURRENT_METER)
        && config->bat.currentMeterType == CURRENT_SENSOR_ADC
    );
    pwm_params.useLEDStrip = feature(config, FEATURE_LED_STRIP);
    pwm_params.usePPM = feature(config, FEATURE_RX_PPM);
    pwm_params.useSerialRx = feature(config, FEATURE_RX_SERIAL);
#ifdef SONAR
    pwm_params.useSonar = feature(config, FEATURE_SONAR);
#endif

#ifdef USE_SERVOS
    pwm_params.useServos = feature(config, FEATURE_SERVO_TILT);
    pwm_params.useChannelForwarding = feature(config, FEATURE_CHANNEL_FORWARDING);
    pwm_params.servoCenterPulse = config->pwm_out.servoCenterPulse;
    pwm_params.servoPwmRate = config->pwm_out.servo_pwm_rate;
#endif

    pwm_params.useOneshot = feature(config, FEATURE_ONESHOT125);
    pwm_params.motorPwmRate = config->pwm_out.motor_pwm_rate;
    pwm_params.idlePulse = config->pwm_out.mincommand;
    if (feature(config, FEATURE_3D))
        pwm_params.idlePulse = config->motor_3d.neutral3d;
    if (pwm_params.motorPwmRate > 500)
        pwm_params.idlePulse = 0; // brushed motors

    pwmRxInit();

    // pwmInit() needs to be called as soon as possible for ESC compatibility reasons
	pwmInit(&pwm_params, &config->pwm_in);
    //pwmIOConfiguration_t *pwmIOConfiguration = pwmInit(&pwm_params);
	//mixer_use_pwmio_config(&ninja.mixer, pwmIOConfiguration);

#ifdef DEBUG_PWM_CONFIGURATION
    debug[2] = pwmIOConfiguration->pwmInputCount;
    debug[3] = pwmIOConfiguration->ppmInputCount;
#endif

	// TODO: why is it important to consider oneshot here? We now handle motor control differently. 
    //if (!feature(FEATURE_ONESHOT125))
    //    ninja.motorControlEnable = true;

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
    if (feature(config, FEATURE_SONAR) && feature(config, FEATURE_SOFTSERIAL)) {
        serialRemovePort(SERIAL_PORT_SOFTSERIAL2);
    }
#endif

#if defined(SPRACINGF3MINI) && defined(SONAR) && defined(USE_SOFTSERIAL1)
    if (feature(config, FEATURE_SONAR) && feature(config, FEATURE_SOFTSERIAL)) {
        serialRemovePort(SERIAL_PORT_SOFTSERIAL1);
    }
#endif


#ifdef USE_I2C
	// TODO: wtf is all this shit?
#if defined(NAZE)
    if (hardwareRevision != NAZE32_SP) {
        i2c_init();
    } else {
        if (!doesConfigurationUsePort(&config->serial, SERIAL_PORT_UART3)) {
        	i2c_init();
        }
    }
#elif defined(CC3D)
    if (!doesConfigurationUsePort(&config->serial, SERIAL_PORT_UART3)) {
		i2c_init();
    }
#else
	i2c_init();
#endif
#endif

#ifdef USE_ADC
    drv_adc_config_t adc_params;

    adc_params.enableVBat = feature(config, FEATURE_VBAT);
    adc_params.enableRSSI = feature(config, FEATURE_RSSI_ADC);
    adc_params.enableCurrentMeter = feature(config, FEATURE_CURRENT_METER);
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
    if (feature(config, FEATURE_DISPLAY)) {
        displayInit();
    }
#endif

    if (!sensorsAutodetect(config)) {
        // if gyro was not detected due to whatever reason, we give up now.
        failureMode(FAILURE_MISSING_ACC);
    }
	
    if (USE_MAG && mag.init)
    	mag.init();

	#ifdef USB_CABLE_DETECTION
    usbCableDetectInit();
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
    sdcardUseDMA = !feature(config, FEATURE_LED_STRIP) || SDCARD_DMA_CHANNEL_TX != WS2811_DMA_CHANNEL;
#else
    sdcardUseDMA = true;
#endif

#endif

    sdcard_init(sdcardUseDMA);

    afatfs_init();
#endif

    systemState |= SYSTEM_STATE_SENSORS_READY;
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

#ifdef SOFTSERIAL_LOOPBACK
    // FIXME this is a hack, perhaps add a FUNCTION_LOOPBACK to support it properly
    loopbackPort = (serialPort_t*)&(softSerialPorts[0]);
    if (!loopbackPort->vTable) {
        loopbackPort = openSoftSerial(0, NULL, 19200, SERIAL_NOT_INVERTED);
    }
    serialPrint(loopbackPort, "LOOPBACK\r\n");
#endif


#ifdef DISPLAY
    if (feature(config, FEATURE_DISPLAY)) {
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

    // Latch active features AGAIN since some may be modified by init().
    //latchActiveFeatures();

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

static sys_micros_t _micros(const struct system_calls_time *time){
	(void)time;
	// micros is pretty much thread safe.
	return micros();
}

static void _write_motor(const struct system_calls_pwm *pwm, uint8_t id, uint16_t value){
	(void)pwm;
	pwmWriteMotor(id, value);
}

static void _write_servo(const struct system_calls_pwm *pwm, uint8_t id, uint16_t value){
	(void)pwm;
	pwmWriteServo(id, value);
}

static uint16_t _read_pwm(const struct system_calls_pwm *pwm, uint8_t id){
	(void)pwm;
	return pwmRead(id);
}

static uint16_t _read_ppm(const struct system_calls_pwm *pwm, uint8_t id){
	(void)pwm;
	return ppmRead(id);
}

static int _gyro_sync(const struct system_calls_imu *imu){
	(void)imu;
	if(gyro.sync) return gyro.sync();
	return -1;
}

static int _read_gyro(const struct system_calls_imu *imu, int16_t output[3]){
	(void)imu;
	if(gyro.read) {
		gyro.read(output);
	}
	return 0;
}

static int _read_acc(const struct system_calls_imu *imu, int16_t output[3]){
	(void)imu;
	if(acc.read){
		acc.read(output);
	}
	return 0;
}

static int _read_pressure(const struct system_calls_imu *sys, uint32_t *pressure){
	(void)sys;
	*pressure = 111000;
	return 0;
}

static int _read_temperature(const struct system_calls_imu *sys, int16_t *temp){
	(void)sys;
	*temp = 2500;
	return 0;
}

static void _led_on(const struct system_calls_leds *leds, uint8_t id, bool on){
	(void)leds;
	if(on) led_on(id);
	else led_off(id);
}

static void _led_toggle(const struct system_calls_leds *leds, uint8_t id){
	(void)leds;
	led_toggle(id);
}

static void _beeper_on(const struct system_calls_beeper *calls, bool on){
	(void)calls;
	if(USE_BEEPER){
		if(on) BEEP_ON;
		else BEEP_OFF;
	}
}

static int _eeprom_read(const struct system_calls_bdev *self, void *dst, uint16_t addr, size_t size){
	(void)self;
	return flash_read(addr, dst, size);
}

static int _eeprom_write(const struct system_calls_bdev *self, uint16_t addr, const void *data, size_t size){
	(void)self;
	return flash_write(addr, data, size);
}

static int _eeprom_erase_page(const struct system_calls_bdev *self, uint16_t addr){
	(void)self;
	return flash_erase_page(addr);
}

static void _eeprom_get_info(const struct system_calls_bdev *self, struct system_bdev_info *info){
	(void)self;
	info->page_size = flash_get_page_size();
	info->num_pages = flash_get_num_pages();
}

static int16_t _logger_write(const struct system_calls_logger *self, const void *data, size_t size){
	(void)self;
	(void)data;
	return size;
}

static int _read_range(const struct system_calls_range *sys, uint16_t deg, uint16_t *range){
	(void)sys;
	(void)deg;
	*range = 200;
	return 0;
}

static struct system_calls syscalls = {
	.pwm = {
		.write_motor = _write_motor,
		.write_servo = _write_servo,
		.read_ppm = _read_ppm,
		.read_pwm = _read_pwm
	},
	.imu = {
		.gyro_sync = _gyro_sync,
		.read_gyro = _read_gyro,
		.read_acc = _read_acc,
		.read_pressure = _read_pressure,
		.read_temperature = _read_temperature
	},
	.leds = {
		.on = _led_on,
		.toggle = _led_toggle
	},
	.beeper = {
		.on = _beeper_on
	},
	.time = {
		.micros = _micros
	},
	.eeprom = {
		.read = _eeprom_read,
		.write = _eeprom_write,
		.erase_page = _eeprom_erase_page,
		.get_info = _eeprom_get_info
	},
	.logger = {
		.write = _logger_write
	},
	.range = {
		.read_range = _read_range
	}
};

struct config_store config;
static struct ninja ninja;
static struct fastloop fastloop;

void HardFault_Handler(void);
void _main_task(void *param){
	(void)param;
	static volatile uint8_t done = 0;

    init(&config.data);

	fastloop_init(&fastloop, &syscalls, &config.data);
	ninja_init(&ninja, &fastloop, &syscalls, &config);

	fastloop_start(&fastloop);

	while (!done) {
        ninja_heartbeat(&ninja);
        processLoopback();
		// TODO: make this task sync to gyro instead of using timeout
		vTaskDelay(1);
    }
}

void _led_blink(void *ptr){
	(void)ptr;
	led_on(0);
	led_on(1);

    while (1){
		led_toggle(1);
		led_toggle(0);
		vTaskDelay(500);
	}
}

int main(void) {
	memset(&gyro, 0, sizeof(gyro));
	// config is loaded first so it should only access flash functions
	config_load(&config, &syscalls);

	pre_init(&config.data);

	xTaskCreate(_led_blink, "blnk", 420 / sizeof(StackType_t), NULL, 1, NULL);
	xTaskCreate(_main_task, "main", 2296 / sizeof(StackType_t), NULL, 2, NULL);
	vTaskStartScheduler();
}

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
	while(1){
		led_on(0);
		led_on(1);
		usleep(100000);
		led_off(0);
		led_off(1);
		usleep(50000);
	}
}

void vApplicationMallocFailedHook(void){
	while(1){
		led_on(0);
		led_on(1);
		usleep(500000);
		led_off(0);
		led_off(1);
		usleep(50000);
	}
}

void vApplicationStackOverflowHook(void){
	while(1){
		led_on(0);
		led_on(1);
		usleep(50000);
		led_off(0);
		led_off(1);
		usleep(500000);
	}
}

