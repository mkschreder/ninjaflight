/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <platform.h>

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/gyro_sync.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"

#include "rx/rx.h"

#include "io/rc_controls.h"

#include "flight/rate_profile.h"
#include "flight/pid.h"

int16_t axisPID[3];

#ifdef BLACKBOX
int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3];
#endif

// PIDweight is a scale factor for PIDs which is derived from the throttle and TPA setting, and 100 = 100% scale means no PID reduction
uint8_t PIDweight[3];

int32_t lastITerm[3], ITermLimit[3];
float lastITermf[3], ITermLimitf[3];

biquad_t deltaFilterState[3];
pidControllerFuncPtr pid_controller = pidMultiWiiRewrite;

void pidResetITerm(void)
{
    for (int axis = 0; axis < 3; axis++) {
        lastITerm[axis] = 0;
        lastITermf[axis] = 0.0f;
    }
}


void pidFilterIsSetCheck(const struct pid_config *pidProfile)
{
    static bool deltaStateIsSet = false;
    if (!deltaStateIsSet && pidProfile->dterm_cut_hz) {
        for (int axis = 0; axis < 3; axis++) {
            BiQuadNewLpf(pidProfile->dterm_cut_hz, &deltaFilterState[axis], gyro_sync_get_looptime());
        }
        deltaStateIsSet = true;
    }
}

void pidSetController(pid_controller_type_t type)
{
    switch (type) {
        default:
        case PID_CONTROLLER_MWREWRITE:
            pid_controller = pidMultiWiiRewrite;
            break;
#ifndef SKIP_PID_LUXFLOAT
        case PID_CONTROLLER_LUX_FLOAT:
            pid_controller = pidLuxFloat;
            break;
#endif
#ifndef SKIP_PID_MW23
        case PID_CONTROLLER_MW23:
            pid_controller = pidMultiWii23;
            break;
#endif
    }
}
