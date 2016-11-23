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

// TODO: remove this once we have refactored enough to pass self to this module
//static struct ninja _default_fc;
//static struct ninja *self = &_default_fc;

extern uint32_t currentTime;

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

#if 0
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
#endif
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
#if 0
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
#endif
