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
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include <platform.h>

#include "scheduler.h"
#include "debug.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"
#include "common/filter.h"

#include "config/parameter_group.h"
#include "config/tilt.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/light_led.h"

//#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/gyro_sync.h"
#include "drivers/pwm_output.h"
#include "io/rc_adjustments.h"

#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "sensors/initialisation.h"

#include "io/beeper.h"
#include "io/display.h"
#include "io/rc_curves.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_cli.h"
#include "io/serial_msp.h"
#include "io/statusindicator.h"
#include "io/asyncfatfs/asyncfatfs.h"
#include "io/transponder_ir.h"


#include "rx/rx.h"
#include "rx/msp.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "flight/rate_profile.h"
#include "flight/mixer.h"
#include "flight/anglerate.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/gtune.h"
#include "flight/navigation.h"
#include "flight/tilt.h"
#include "sensors/instruments.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/feature.h"

#include "ninjaflight.h"

// June 2013     V2.2-dev

enum {
    ALIGN_GYRO = 0,
    ALIGN_ACCEL = 1,
    ALIGN_MAG = 2
};

/* VBAT monitoring interval (in microseconds) - 1s*/
#define VBATINTERVAL (6 * 3500)
/* IBat monitoring interval (in microseconds) - 6 default looptimes */
#define IBATINTERVAL (6 * 3500)

// TODO: remove this once we have refactored enough to pass self to this module
//static struct ninja _default_fc;
//static struct ninja *self = &_default_fc;

float dT;

extern uint32_t currentTime;
extern uint8_t PIDweight[3];

#ifdef GTUNE

static void updateGtuneState(void)
{
    static bool GTuneWasUsed = false;

    if (rcModeIsActive(BOXGTUNE)) {
        if (!FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
            ENABLE_FLIGHT_MODE(GTUNE_MODE);
            init_Gtune();
            GTuneWasUsed = true;
        }
        if (!FLIGHT_MODE(GTUNE_MODE) && !ARMING_FLAG(ARMED) && GTuneWasUsed) {
            saveConfigAndNotify();
            GTuneWasUsed = false;
        }
    } else {
        if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
            DISABLE_FLIGHT_MODE(GTUNE_MODE);
        }
    }
}
#endif

bool isCalibrating(void)
{
	return ins_is_calibrated(&ninja.ins);
}

static uint8_t calibrating = 0;
void ninja_calibrate_acc(void){
	ins_start_acc_calibration(&ninja.ins);
	calibrating = 1;
}

void ninja_calibrate_mag(void){
	ins_start_mag_calibration(&ninja.ins);
	calibrating = 1;
}

static void updateLEDs(void)
{
    if (ARMING_FLAG(ARMED)) {
        led_on(0);
    } else {
        if (rcModeIsActive(BOXARM) == 0) {
            ENABLE_ARMING_FLAG(OK_TO_ARM);
        }

/*
		// TODO: for now allow arming when not leveled but rethink this logic entirely
        if (!imu_is_leveled(&default_imu, armingConfig()->max_arm_angle)) {
            DISABLE_ARMING_FLAG(OK_TO_ARM);
        }
*/
		// TODO: this should probably be checked elsewhere
		if(calibrating && !isCalibrating()){
			// save config here since calibration may have changed it
			saveConfigAndNotify();
			calibrating = 0;
		}

        if (isCalibrating() || isSystemOverloaded()) {
            warningLedFlash();
            DISABLE_ARMING_FLAG(OK_TO_ARM);
        } else {
            if (ARMING_FLAG(OK_TO_ARM)) {
                warningLedDisable();
            } else {
                warningLedFlash();
            }
        }

        warningLedUpdate();
    }
}
#define TELEMETRY_FUNCTION_MASK (FUNCTION_TELEMETRY_FRSKY | FUNCTION_TELEMETRY_HOTT | FUNCTION_TELEMETRY_SMARTPORT | FUNCTION_TELEMETRY_LTM | FUNCTION_TELEMETRY_MAVLINK)

#ifdef TELEMETRY
static void releaseSharedTelemetryPorts(void) {
    serialPort_t *sharedPort = findSharedSerialPort(TELEMETRY_FUNCTION_MASK, FUNCTION_MSP);
     while (sharedPort) {
         mspSerialReleasePortIfAllocated(sharedPort);
         sharedPort = findNextSharedSerialPort(TELEMETRY_FUNCTION_MASK, FUNCTION_MSP);
     }
}
#endif
static void updateMagHold(void)
{
	// TODO: really refactor this kind of crap. This belongs in flight control code
    if (ABS(rcCommand[YAW]) < 15 && FLIGHT_MODE(MAG_MODE)) {
        int16_t dif = DECIDEGREES_TO_DEGREES(ins_get_yaw_dd(&ninja.ins)) - ninja.magHold;
        if (dif <= -180)
            dif += 360;
        if (dif >= +180)
            dif -= 360;
        dif *= -rcControlsConfig()->yaw_control_direction;
		// TODO: small angle was set in imu before. Generally this kind of logic is horrible. This needs to be scrapped.
        if (STATE(SMALL_ANGLE))
            rcCommand[YAW] -= dif * pidProfile()->P8[PIDMAG] / 30;    // 18 deg
    } else
        ninja.magHold = DECIDEGREES_TO_DEGREES(ins_get_yaw_dd(&ninja.ins));
}

#if defined(BARO) || defined(SONAR)
static bool haveUpdatedRcCommandsOnce = false;
#endif


// TODO: untangle all the crap with dt
void ninja_run_pid_loop(struct ninja *self, float dt_){
	UNUSED(self);
	UNUSED(dt_);

    currentTime = micros();
	static uint32_t previousIMUUpdateTime = 0;
    float dt = (currentTime - previousIMUUpdateTime) * 1e-6f;
	dT = dt;
    previousIMUUpdateTime = currentTime;

	int16_t rawgyro[3];
	if (gyro.read(rawgyro)) {
		ins_process_gyro(&ninja.ins, rawgyro[0], rawgyro[1], rawgyro[2]);
	}

#if defined(GPS)
/*
	// TODO: rethink how to enter gps yaw angle into ins (probably we enter all gps data and then ins decides how to use the angle!)
    if (STATE(FIXED_WING) && sensors(SENSOR_GPS) && STATE(GPS_FIX) && GPS_numSat >= 5 && GPS_speed >= 300) {
        // In case of a fixed-wing aircraft we can use GPS course over ground to correct heading
        ins_input_yaw_dd(&default_imu, GPS_ground_course);
    }
	*/
#endif

#if defined(BARO) || defined(SONAR)
    haveUpdatedRcCommandsOnce = true;
#endif

    // Read out gyro temperature. can use it for something somewhere. maybe get MCU temperature instead? lots of fun possibilities.
    if (gyro.temperature) {
        gyro.temperature(&self->telemTemperature1);
    }

	if (USE_MAG && sensors(SENSOR_MAG)) {
		updateMagHold();
	}

#ifdef GTUNE
        updateGtuneState();
#endif

#if defined(BARO) || defined(SONAR)
        if (sensors(SENSOR_BARO) || sensors(SENSOR_SONAR)) {
            if (FLIGHT_MODE(BARO_MODE) || FLIGHT_MODE(SONAR_MODE)) {
                applyAltHold();
            }
        }
#endif

    // If we're armed, at minimum throttle, and we do arming via the
    // sticks, do not process yaw input from the rx.  We do this so the
    // motors do not spin up while we are trying to arm or disarm.
    // Allow yaw control for tricopters if the user wants the servo to move even when unarmed.
    if (isUsingSticksForArming() && rc_get_channel_value(THROTTLE) <= rxConfig()->mincheck
#ifdef USE_SERVOS
                && !((mixerConfig()->mixerMode == MIXER_TRI || mixerConfig()->mixerMode == MIXER_CUSTOM_TRI) && mixerConfig()->tri_unarmed_servo)
#endif
                && mixerConfig()->mixerMode != MIXER_AIRPLANE
                && mixerConfig()->mixerMode != MIXER_FLYING_WING
    ) {
        rcCommand[YAW] = 0;
    }

	// TODO: make throttle correction work after refactoring
	/*
    if (throttleCorrectionConfig()->throttle_correction_value && (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE))) {
        rcCommand[THROTTLE] += imu_calc_throttle_angle_correction(&default_imu, throttleCorrectionConfig()->throttle_correction_value);
    }*/

#ifdef GPS
    if (sensors(SENSOR_GPS)) {
        if ((FLIGHT_MODE(GPS_HOME_MODE) || FLIGHT_MODE(GPS_HOLD_MODE)) && STATE(GPS_FIX_HOME)) {
            updateGpsStateForHomeAndHoldMode();
        }
    }
#endif
	// TODO: move this once we have tested current refactored code
	ninja_update_controller(&ninja, dt);

    	// TODO: refactor this
#ifdef GTUNE
	// TODO: unit test this gtune stuff. This may not be the right place to put it.
	if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
		 for(int c = 0; c < 3; c++) calculate_Gtune(c);
	}
#endif

		// TODO: gimbal should be driven directly through the aux channels when enabled.
	//input[INPUT_GIMBAL_PITCH] = scaleRange(self->gimbal_angles[PITCH], -1800, 1800, -500, +500);
	//input[INPUT_GIMBAL_ROLL] = scaleRange(self->gimbal_angles[ROLL], -1800, 1800, -500, +500);

	//input[INPUT_STABILIZED_THROTTLE] = mixer_get_motor_value(&default_mixer, 0) - 1500;  // Since it derives from rcCommand or mincommand and must be [-500:+500]

	// write out all motors and servos
	for (int i = 0; i < MIXER_MAX_MOTORS; i++){
		pwmWriteMotor(i, mixer_get_motor_value(&ninja.mixer, i));
	}
#ifdef USE_SERVOS
	for (int i = 0; i < MIXER_MAX_SERVOS; i++){
		pwmWriteServo(i, mixer_get_servo_value(&ninja.mixer, i));
	}
#endif
	if (feature(FEATURE_ONESHOT125)) {
		pwmCompleteOneshotMotorUpdate(MIXER_MAX_MOTORS);
	}

#ifdef USE_SDCARD
        afatfs_poll();
#endif

#ifdef BLACKBOX
    if (!cliMode && feature(FEATURE_BLACKBOX)) {
        handleBlackbox();
    }
#endif
}

void taskUpdateAccelerometer(void){
	int16_t accADCRaw[3];
	if (sensors(SENSOR_ACC) && acc.read(accADCRaw)) {
		ins_process_acc(&ninja.ins, accADCRaw[0], accADCRaw[1], accADCRaw[2]);
        //updateAccelerationReadings();
	}
}

void taskHandleSerial(void)
{
    handleSerial();
}

#ifdef BEEPER
void taskUpdateBeeper(void)
{
    beeperUpdate();          //call periodic beeper handler
}
#endif

void taskUpdateBattery(void)
{
    static uint32_t vbatLastServiced = 0;
    static uint32_t ibatLastServiced = 0;

    if (feature(FEATURE_VBAT)) {
        if (cmp32(currentTime, vbatLastServiced) >= VBATINTERVAL) {
            vbatLastServiced = currentTime;
            battery_update(&default_battery);
        }
    }

    if (feature(FEATURE_CURRENT_METER)) {
        int32_t ibatTimeSinceLastServiced = cmp32(currentTime, ibatLastServiced);

        if (ibatTimeSinceLastServiced >= IBATINTERVAL) {
            ibatLastServiced = currentTime;

            throttleStatus_e throttleStatus = calculateThrottleStatus(rxConfig(), rcControlsConfig()->deadband3d_throttle);

            battery_update_current_meter(&default_battery, ibatTimeSinceLastServiced, throttleStatus);
        }
    }
}

bool taskUpdateRxCheck(uint32_t currentDeltaTime)
{
    UNUSED(currentDeltaTime);
    updateRx(currentTime);
    return shouldProcessRx(currentTime);
}

void taskUpdateRxMain(void)
{
    updateLEDs();

	ninja_process_rx(&ninja);

#ifdef BARO
    // updateRcCommands() sets rcCommand[], updateAltHoldState depends on valid rcCommand[] data.
    if (haveUpdatedRcCommandsOnce) {
        if (sensors(SENSOR_BARO)) {
            updateAltHoldState();
        }
    }
#endif

#ifdef SONAR
    // updateRcCommands() sets rcCommand[], updateAltHoldState depends on valid rcCommand[] data.
    if (haveUpdatedRcCommandsOnce) {
        if (sensors(SENSOR_SONAR)) {
            updateSonarAltHoldState();
        }
    }
#endif
}

#ifdef GPS
void taskProcessGPS(void)
{
    // if GPS feature is enabled, gpsThread() will be called at some intervals to check for stuck
    // hardware, wrong baud rates, init GPS if needed, etc. Don't use SENSOR_GPS here as gpsThread() can and will
    // change this based on available hardware
    if (feature(FEATURE_GPS)) {
        gpsThread();
    }

    if (sensors(SENSOR_GPS)) {
        updateGpsIndicator(currentTime);
    }
}
#endif

void taskUpdateCompass(void){
	if(USE_MAG){
		int16_t raw[3];
		if (sensors(SENSOR_MAG) && mag.read(raw)){
			ins_process_mag(&ninja.ins, raw[0], raw[1], raw[2]);
			//updateCompass(&sensorTrims()->magZero);
		}
	}
}

#ifdef BARO
void taskUpdateBaro(void)
{
    if (sensors(SENSOR_BARO)) {
        uint32_t newDeadline = baroUpdate();
        rescheduleTask(TASK_SELF, newDeadline);
    }
}
#endif

#ifdef SONAR
void taskUpdateSonar(void)
{
    if (sensors(SENSOR_SONAR)) {
        sonar_update(&default_sonar);
    }
}
#endif

#if defined(BARO) || defined(SONAR)
void taskCalculateAltitude(void)
{
    if (false
#if defined(BARO)
        || (sensors(SENSOR_BARO) && isBaroReady())
#endif
#if defined(SONAR)
        || sensors(SENSOR_SONAR)
#endif
        ) {
        calculateEstimatedAltitude(currentTime);
    }}
#endif

#ifdef DISPLAY
void taskUpdateDisplay(void)
{
    if (feature(FEATURE_DISPLAY)) {
        updateDisplay();
    }
}
#endif

#ifdef TELEMETRY
void taskTelemetry(void)
{
    telemetryCheckState();

    if (!cliMode && feature(FEATURE_TELEMETRY)) {
        telemetryProcess(rcControlsConfig()->deadband3d_throttle);
    }
}
#endif

#ifdef LED_STRIP
void taskLedStrip(void)
{
    if (feature(FEATURE_LED_STRIP)) {
        updateLedStrip();
    }
}
#endif

#ifdef TRANSPONDER
void ninja_update_transponder(struct ninja *self){
	UNUSED(self);
    if (feature(FEATURE_TRANSPONDER)) {
        updateTransponder();
    }
}
#endif
