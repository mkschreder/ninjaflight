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
#include <string.h>
#include <math.h>

#include <platform.h>

#include "build_config.h"

#include "config/runtime_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/gyro_sync.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#include "rx/rx.h"

#include "io/rc_controls.h"

#include "anglerate.h"
#include "rate_profile.h"
#include "navigation.h"
#include "mixer.h"
#include "gtune.h"

#define ANGLERATE_FLAG_ANTIWINDUP (1 << 0)
#define ANGLERATE_FLAG_PLIMIT (1 << 0)

static void _anglerate_delta_state_update(struct anglerate *self) {
    if (!self->_delta_state_set && self->config->dterm_cut_hz) {
        for (int axis = 0; axis < 3; axis++) {
            BiQuadNewLpf(self->config->dterm_cut_hz, &self->deltaFilterState[axis], gyro_sync_get_looptime());
        }
        self->_delta_state_set = true;
    }
}

static int16_t _multiwii_rewrite_calc_axis(struct anglerate *self, int axis, int32_t gyroRate, int32_t angleRate)
{
    const int32_t rateError = angleRate - gyroRate;

    // -----calculate P component
    int32_t PTerm = (rateError * self->config->P8[axis] * self->PIDweight[axis] / 100) >> 7;
    // Constrain YAW by yaw_p_limit value if not servo driven, in that case servolimits apply
    if (axis == YAW && self->config->yaw_p_limit && (self->flags & ANGLERATE_FLAG_PLIMIT)) {
        PTerm = constrain(PTerm, -self->config->yaw_p_limit, self->config->yaw_p_limit);
    }

    // -----calculate I component
    // There should be no division before accumulating the error to integrator, because the precision would be reduced.
    // Precision is critical, as I prevents from long-time drift. Thus, 32 bits integrator (Q19.13 format) is used.
    // Time correction (to avoid different I scaling for different builds based on average cycle time)
    // is normalized to cycle time = 2048 (2^11).
	// TODO: why is loop time cast from 32 bit to 16 bit??
    int32_t ITerm = self->lastITerm[axis] + ((rateError * (uint16_t)gyro_sync_get_looptime()) >> 11) * self->config->I8[axis];
    // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
    // I coefficient (I8) moved before integration to make limiting independent from PID settings
    ITerm = constrain(ITerm, (int32_t)(-PID_MAX_I << 13), (int32_t)(PID_MAX_I << 13));
    // Anti windup protection
    if (rcModeIsActive(BOXAIRMODE)) {
		// TODO: state here is defined in runtime_config.h, which is nonsense. Move it into this module when done refactoring.
        if (self->flags & ANGLERATE_FLAG_ANTIWINDUP) {
            ITerm = constrain(ITerm, -self->ITermLimit[axis], self->ITermLimit[axis]);
        } else {
            self->ITermLimit[axis] = ABS(ITerm);
        }
    }
    self->lastITerm[axis] = ITerm;
    ITerm = ITerm >> 13; // take integer part of Q19.13 value

    // -----calculate D component
    int32_t DTerm;
    if (self->config->D8[axis] == 0) {
        // optimisation for when D8 is zero, often used by YAW axis
        DTerm = 0;
    } else {
        // delta calculated from measurement
        int32_t delta = -(gyroRate - self->lastRateForDelta[axis]);
        self->lastRateForDelta[axis] = gyroRate;
        // Divide delta by targetLooptime to get differential (ie dr/dt)
        delta = (delta * ((uint16_t)0xFFFF / ((uint16_t)gyro_sync_get_looptime() >> 4))) >> 5;
        if (self->config->dterm_cut_hz) {
            // DTerm delta low pass filter
            delta = lrintf(applyBiQuadFilter((float)delta, &self->deltaFilterState[axis]));
        } else {
            // When DTerm low pass filter disabled apply moving average to reduce noise
            delta = filterApplyAverage(delta, DTERM_AVERAGE_COUNT, self->deltaStatei[axis]);
        }
        DTerm = (delta * self->config->D8[axis] * self->PIDweight[axis] / 100) >> 8;
        DTerm = constrain(DTerm, -PID_MAX_D, PID_MAX_D);
    }

    self->output.axis_P[axis] = PTerm;
    self->output.axis_I[axis] = ITerm;
    self->output.axis_D[axis] = DTerm;
    // -----calculate total PID output
    return PTerm + ITerm + DTerm;
}

static void _multiwii_rewrite_update(struct anglerate *self, const gyro_rates_t gyro, const euler_angles_t att, float dt) {
	UNUSED(dt); // TODO: maybe we should use dt?
    int8_t horizonLevelStrength = 0;

    _anglerate_delta_state_update(self);

    if (FLIGHT_MODE(HORIZON_MODE)) {
        // Figure out the most deflected stick position
        const int32_t stickPosAil = ABS(getRcStickDeflection(ROLL, self->rx_config->midrc));
        const int32_t stickPosEle = ABS(getRcStickDeflection(PITCH, self->rx_config->midrc));
        const int32_t mostDeflectedPos =  MAX(stickPosAil, stickPosEle);

        // Progressively turn off the horizon self level strength as the stick is banged over
        horizonLevelStrength = (500 - mostDeflectedPos) / 5;  // 100 at centre stick, 0 = max stick deflection

        // Using D8[PIDLEVEL] as a Sensitivity for Horizon.
        // 0 more level to 255 more rate. Default value of 100 seems to work fine.
        // For more rate mode increase D and slower flips and rolls will be possible
        horizonLevelStrength = constrain((10 * (horizonLevelStrength - 100) * (10 * self->config->D8[PIDLEVEL] / 80) / 100) + 100, 0, 100);
    }

	
    // ----------PID controller----------
    for (int axis = 0; axis < 3; axis++) {
        const uint8_t rate = self->rate_config->rates[axis];

        // -----Get the desired angle rate depending on flight mode
        int32_t angleRate;
        if (axis == FD_YAW) {
            // YAW is always gyro-controlled (MAG correction is applied to rcCommand)
            angleRate = (((int32_t)(rate + 27) * rcCommand[YAW]) >> 5);
        } else {
            // control is GYRO based for ACRO and HORIZON - direct sticks control is applied to rate PID
            angleRate = ((int32_t)(rate + 27) * rcCommand[axis]) >> 4;
            if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
                // calculate error angle and limit the angle to the max inclination
                // multiplication of rcCommand corresponds to changing the sticks scaling here
#ifdef GPS
                const int32_t errorAngle = constrain(2 * rcCommand[axis] + GPS_angle[axis], -((int)self->max_angle_inclination), self->max_angle_inclination)
                        - att.raw[axis] + self->angle_trim->raw[axis];
#else
                const int32_t errorAngle = constrain(2 * rcCommand[axis], -((int)self->max_angle_inclination), self->max_angle_inclination)
                        - att.raw[axis] + self->angle_trim->raw[axis];
#endif
                if (FLIGHT_MODE(ANGLE_MODE)) {
                    // ANGLE mode
                    angleRate = (errorAngle * self->config->P8[PIDLEVEL]) >> 4;
                } else {
                    // HORIZON mode
                    // mix in errorAngle to desired angleRate to add a little auto-level feel.
                    // horizonLevelStrength has been scaled to the stick input
                    angleRate += (errorAngle * self->config->I8[PIDLEVEL] * horizonLevelStrength / 100) >> 4;
                }
            }
        }

        // --------low-level gyro-based PID. ----------
        const int32_t gyroRate = gyro[axis] / 4;
        self->output.axis[axis] = _multiwii_rewrite_calc_axis(self, axis, gyroRate, angleRate);

#ifdef GTUNE
        if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
             calculate_Gtune(axis);
        }
#endif
    }
}

// constants to scale pidLuxFloat so output is same as pidMultiWiiRewrite
static const float luxPTermScale = 1.0f / 128;
static const float luxITermScale = 1000000.0f / 0x1000000;
static const float luxDTermScale = (0.000001f * (float)0xFFFF) / 512;
static const float luxGyroScale = 16.4f / 4; // the 16.4 is needed because mwrewrite does not scale according to the gyro model gyro.scale

static int16_t _luxfloat_calc_axis(struct anglerate *self, int axis, float gyroRate, float angleRate, float dT){
    const float rateError = angleRate - gyroRate;

    // -----calculate P component
    float PTerm = luxPTermScale * rateError * self->config->P8[axis] * self->PIDweight[axis] / 100;

    // Constrain YAW by yaw_p_limit value if not servo driven, in that case servolimits apply
    if (axis == YAW && self->config->yaw_p_limit && (self->flags & ANGLERATE_FLAG_PLIMIT)) {
        PTerm = constrainf(PTerm, -self->config->yaw_p_limit, self->config->yaw_p_limit);
    }

    // -----calculate I component
    float ITerm = self->lastITermf[axis] + luxITermScale * rateError * dT * self->config->I8[axis];
    // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
    // I coefficient (I8) moved before integration to make limiting independent from PID settings
    ITerm = constrainf(ITerm, -PID_MAX_I, PID_MAX_I);
    // Anti windup protection
    if (rcModeIsActive(BOXAIRMODE)) {
        if (self->flags & ANGLERATE_FLAG_ANTIWINDUP) {
            ITerm = constrainf(ITerm, -self->ITermLimitf[axis], self->ITermLimitf[axis]);
        } else {
            self->ITermLimitf[axis] = ABS(ITerm);
        }
    }
    self->lastITermf[axis] = ITerm;

    // -----calculate D component
    float DTerm;
    if (self->config->D8[axis] == 0) {
        // optimisation for when D8 is zero, often used by YAW axis
        DTerm = 0;
    } else {
        // delta calculated from measurement
        float delta = -(gyroRate - self->lastRateForDelta[axis]);
        self->lastRateForDelta[axis] = gyroRate;
        // Divide delta by dT to get differential (ie dr/dt)
        delta *= (1.0f / dT);
        if (self->config->dterm_cut_hz) {
            // DTerm delta low pass filter
            delta = applyBiQuadFilter(delta, &self->deltaFilterState[axis]);
        } else {
            // When DTerm low pass filter disabled apply moving average to reduce noise
            delta = filterApplyAveragef(delta, DTERM_AVERAGE_COUNT, self->deltaStatef[axis]);
        }
        DTerm = luxDTermScale * delta * self->config->D8[axis] * self->PIDweight[axis] / 100;
        DTerm = constrainf(DTerm, -PID_MAX_D, PID_MAX_D);
    }

	self->output.axis_P[axis] = PTerm;
    self->output.axis_I[axis] = ITerm;
    self->output.axis_D[axis] = DTerm;
    // -----calculate total PID output
    return lrintf(PTerm + ITerm + DTerm);
}

static void _luxfloat_update(struct anglerate *self, const gyro_rates_t gyro, const euler_angles_t att, float dT){
    _anglerate_delta_state_update(self);

    float horizonLevelStrength = 0;
    if (FLIGHT_MODE(HORIZON_MODE)) {
        // Figure out the most deflected stick position
        const int32_t stickPosAil = ABS(getRcStickDeflection(ROLL, self->rx_config->midrc));
        const int32_t stickPosEle = ABS(getRcStickDeflection(PITCH, self->rx_config->midrc));
        const int32_t mostDeflectedPos =  MAX(stickPosAil, stickPosEle);

        // Progressively turn off the horizon self level strength as the stick is banged over
        horizonLevelStrength = (float)(500 - mostDeflectedPos) / 500;  // 1 at centre stick, 0 = max stick deflection
        if(self->config->D8[PIDLEVEL] == 0){
            horizonLevelStrength = 0;
        } else {
            horizonLevelStrength = constrainf(((horizonLevelStrength - 1) * (100 / self->config->D8[PIDLEVEL])) + 1, 0, 1);
        }
    }

    // ----------PID controller----------
    for (int axis = 0; axis < 3; axis++) {
        const uint8_t rate = self->rate_config->rates[axis];

        // -----Get the desired angle rate depending on flight mode
        float angleRate;
        if (axis == FD_YAW) {
            // YAW is always gyro-controlled (MAG correction is applied to rcCommand) 100dps to 1100dps max yaw rate
            angleRate = (float)((rate + 27) * rcCommand[YAW]) / 32.0f;
        } else {
            // control is GYRO based for ACRO and HORIZON - direct sticks control is applied to rate PID
            angleRate = (float)((rate + 27) * rcCommand[axis]) / 16.0f; // 200dps to 1200dps max roll/pitch rate
            if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
                // calculate error angle and limit the angle to the max inclination
                // multiplication of rcCommand corresponds to changing the sticks scaling here
#ifdef GPS
                const float errorAngle = constrain(2 * rcCommand[axis] + GPS_angle[axis], -((int)self->max_angle_inclination), self->max_angle_inclination)
                        - att.raw[axis] + self->angle_trim->raw[axis];
#else
                const float errorAngle = constrain(2 * rcCommand[axis], -((int)self->max_angle_inclination), self->max_angle_inclination)
                        - att.raw[axis] + self->angle_trim->raw[axis];
#endif
                if (FLIGHT_MODE(ANGLE_MODE)) {
                    // ANGLE mode
                    angleRate = errorAngle * self->config->P8[PIDLEVEL] / 16.0f;
                } else {
                    // HORIZON mode
                    // mix in errorAngle to desired angleRate to add a little auto-level feel.
                    // horizonLevelStrength has been scaled to the stick input
                    angleRate += errorAngle * self->config->I8[PIDLEVEL] * horizonLevelStrength / 16.0f;
                }
            }
        }

        // --------low-level gyro-based PID. ----------
		// TODO: refactor this so that we always have gyro scaled to rad/s
        const float gyroRate = luxGyroScale * gyro[axis] * (1.0f / 16.4f); // * (imu_get_gyro_scale(self->imu) / RAD); TODO: make sure that we are not dependent on gyro scale
        self->output.axis[axis] = _luxfloat_calc_axis(self, axis, gyroRate, angleRate, dT);
        //output->axis[axis] = constrain(output->axis[axis], -PID_LUX_FLOAT_MAX_PID, PID_LUX_FLOAT_MAX_PID);
#ifdef GTUNE
        if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
            calculate_Gtune(axis);
        }
#endif
    }
}

static void _multiwii23_update(struct anglerate *self, const gyro_rates_t gyro, const euler_angles_t att, float dt){
	UNUSED(dt);
    int axis, prop = 0;
    int32_t rc, error, errorAngle, delta, gyroError;
    int32_t PTerm, ITerm, PTermACC, ITermACC, DTerm;
    static int16_t lastErrorForDelta[2];
    static int32_t delta1[2], delta2[2];

    _anglerate_delta_state_update(self);

    if (FLIGHT_MODE(HORIZON_MODE)) {
        prop = MIN(MAX(ABS(rcCommand[PITCH]), ABS(rcCommand[ROLL])), 512);
    }

    // PITCH & ROLL
    for (axis = 0; axis < 2; axis++) {

        rc = rcCommand[axis] << 1;

        gyroError = gyro[axis] / 4;

        error = rc - gyroError;
        self->lastITerm[axis]  = constrain(self->lastITerm[axis] + error, -16000, +16000);   // WindUp   16 bits is ok here

        if (ABS(gyro[axis]) > (640 * 4)) {
            self->lastITerm[axis] = 0;
        }

        // Anti windup protection
        if (rcModeIsActive(BOXAIRMODE)) {
            if (self->flags & ANGLERATE_FLAG_ANTIWINDUP) {
                self->lastITerm[axis] = constrain(self->lastITerm[axis], -self->ITermLimit[axis], self->ITermLimit[axis]);
            } else {
                self->ITermLimit[axis] = ABS(self->lastITerm[axis]);
            }
        }

        ITerm = (self->lastITerm[axis] >> 7) * self->config->I8[axis] >> 6;   // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000

        PTerm = (int32_t)rc * self->config->P8[axis] >> 6;

        if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {   // axis relying on ACC
            // 50 degrees max inclination
#ifdef GPS
            errorAngle = constrain(2 * rcCommand[axis] + GPS_angle[axis], -((int) self->max_angle_inclination),
                +self->max_angle_inclination) - att.raw[axis] + self->angle_trim->raw[axis];
#else
            errorAngle = constrain(2 * rcCommand[axis], -((int) self->max_angle_inclination),
                +self->max_angle_inclination) - att.raw[axis] + self->angle_trim->raw[axis];
#endif

            self->ITermAngle[axis]  = constrain(self->ITermAngle[axis] + errorAngle, -10000, +10000);                                                // WindUp     //16 bits is ok here

            PTermACC = ((int32_t)errorAngle * self->config->P8[PIDLEVEL]) >> 7;   // 32 bits is needed for calculation: errorAngle*P8 could exceed 32768   16 bits is ok for result

            int16_t limit = self->config->D8[PIDLEVEL] * 5;
            PTermACC = constrain(PTermACC, -limit, +limit);

            ITermACC = ((int32_t)self->ITermAngle[axis] * self->config->I8[PIDLEVEL]) >> 12;  // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result

            ITerm = ITermACC + ((ITerm - ITermACC) * prop >> 9);
            PTerm = PTermACC + ((PTerm - PTermACC) * prop >> 9);
        }

		uint16_t P = (uint16_t)self->config->P8[axis] * self->pidScale[axis] / 100;

        PTerm -= ((int32_t)gyroError * P) >> 6;   // 32 bits is needed for calculation

        //-----calculate D-term based on the configured approach (delta from measurement or deltafromError)
        // Delta from measurement
        delta = -(gyroError - lastErrorForDelta[axis]);
        lastErrorForDelta[axis] = gyroError;
        if (self->config->dterm_cut_hz) {
            // Dterm delta low pass
            DTerm = delta;
            DTerm = lrintf(applyBiQuadFilter((float) DTerm, &self->deltaFilterState[axis])) * 3;  // Keep same scaling as unfiltered DTerm
        } else {
            // When dterm filter disabled apply moving average to reduce noise
            DTerm  = delta1[axis] + delta2[axis] + delta;
            delta2[axis] = delta1[axis];
            delta1[axis] = delta;
        }

		uint16_t D = (uint16_t)self->config->D8[axis] * self->pidScale[axis] / 100;
        DTerm = ((int32_t)DTerm * D) >> 5;   // 32 bits is needed for calculation

        self->output.axis[axis] = PTerm + ITerm + DTerm;

#ifdef GTUNE
        if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
            calculate_Gtune(axis);
        }
#endif

        self->output.axis_P[axis] = PTerm;
        self->output.axis_I[axis] = ITerm;
        self->output.axis_D[axis] = DTerm;
    }

    //YAW
    rc = (int32_t)rcCommand[YAW] * (2 * self->rate_config->rates[YAW] + 30)  >> 5;
#ifdef ALIENWFLIGHT
    error = rc - gyro[FD_YAW];
#else
    error = rc - (gyro[FD_YAW] / 4);
#endif
    self->lastITerm[FD_YAW]  += (int32_t)error * self->config->I8[FD_YAW];
    self->lastITerm[FD_YAW]  = constrain(self->lastITerm[FD_YAW], 2 - ((int32_t)1 << 28), -2 + ((int32_t)1 << 28));
    if (ABS(rc) > 50) self->lastITerm[FD_YAW] = 0;

    PTerm = (int32_t)error * self->config->P8[FD_YAW] >> 6; // TODO: Bitwise shift on a signed integer is not recommended

    // Constrain YAW by D value if not servo driven in that case servolimits apply
    if((self->flags & ANGLERATE_FLAG_PLIMIT) && self->config->yaw_p_limit < YAW_P_LIMIT_MAX) {
        PTerm = constrain(PTerm, -self->config->yaw_p_limit, self->config->yaw_p_limit);
    }

    ITerm = constrain((int16_t)(self->lastITerm[FD_YAW] >> 13), -GYRO_I_MAX, +GYRO_I_MAX);

    self->output.axis[FD_YAW] =  PTerm + ITerm;

#ifdef GTUNE
    if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
        calculate_Gtune(FD_YAW);
    }
#endif

    self->output.axis_P[FD_YAW] = PTerm;
    self->output.axis_I[FD_YAW] = ITerm;
    self->output.axis_D[FD_YAW] = 0;
}


void anglerate_init(struct anglerate *self,
	struct instruments *ins,
	const struct pid_config *config,
	const struct rate_config *rate_config,
	uint16_t max_angle_inclination,
	const rollAndPitchTrims_t *angle_trim,
	const rxConfig_t *rx_config){
	memset(self, 0, sizeof(struct anglerate));
	self->ins = ins;
	self->update = _multiwii_rewrite_update;
	for(int c = 0; c < 3; c++) {
		self->pidScale[c] = 100;
		self->PIDweight[c] = 100;
	}
	self->config = config;
	self->rate_config = rate_config;
	self->max_angle_inclination = max_angle_inclination;
	self->angle_trim = angle_trim;
	self->rx_config = rx_config;
}

void anglerate_set_algo(struct anglerate *self, pid_controller_type_t type){
	switch (type) {
        default:
        case PID_CONTROLLER_MWREWRITE:
            self->update = _multiwii_rewrite_update;
            break;
#ifndef SKIP_PID_LUXFLOAT
        case PID_CONTROLLER_LUX_FLOAT:
            self->update = _luxfloat_update;
            break;
#endif
#ifndef SKIP_PID_MW23
        case PID_CONTROLLER_MW23:
            self->update = _multiwii23_update;
            break;
#endif
    }

}

void anglerate_reset_angle_i(struct anglerate *self){
    self->ITermAngle[AI_ROLL] = 0;
    self->ITermAngle[AI_PITCH] = 0;
}

void anglerate_reset_rate_i(struct anglerate *self) {
	memset(self->lastITerm, 0, sizeof(self->lastITerm));
	memset(self->lastITermf, 0, sizeof(self->lastITermf));
}

const struct pid_controller_output *anglerate_get_output_ptr(struct anglerate *self){
	return &self->output;
}

void anglerate_update(struct anglerate *self, const gyro_rates_t measured_rates, const euler_angles_t measured_att, float dt){
	self->update(self, measured_rates, measured_att, dt);
}

void anglerate_enable_plimit(struct anglerate *self, bool on){
	if(on) self->flags |= ANGLERATE_FLAG_PLIMIT;
	else self->flags &= ~ANGLERATE_FLAG_PLIMIT;
}

void anglerate_enable_antiwindup(struct anglerate *self, bool on){
	if(on) self->flags |= ANGLERATE_FLAG_ANTIWINDUP;
	else self->flags &= ~ANGLERATE_FLAG_ANTIWINDUP;
}

void anglerate_set_pid_axis_scale(struct anglerate *self, uint8_t axis, int32_t scale){
	if(axis > 2) return;
	self->pidScale[axis] = scale;
}

void anglerate_set_pid_axis_weight(struct anglerate *self, uint8_t axis, int32_t weight){
	self->PIDweight[axis] = weight;
}

