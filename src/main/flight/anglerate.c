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
#include "config/rate_profile.h"
#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

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

static int16_t _multiwii_rewrite_calc_axis(struct anglerate *self, int axis, int32_t gyroRate, int32_t angleRate, uint32_t dt_us)
{
	const int32_t rateError = angleRate - gyroRate;
	
	// dt_us must be at least 16 to avoid division by zero
	dt_us = constrain(dt_us, 100, 1000000);

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
	int32_t ITerm = self->lastITerm[axis] + ((rateError * dt_us) >> 11) * self->config->I8[axis];
	// limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
	// I coefficient (I8) moved before integration to make limiting independent from PID settings
	ITerm = constrain(ITerm, (int32_t)(-PID_MAX_I << 13), (int32_t)(PID_MAX_I << 13));
	// Anti windup protection
	// TODO: state here is defined in runtime_config.h, which is nonsense. Move it into this module when done refactoring.
	if (self->flags & ANGLERATE_FLAG_ANTIWINDUP) {
		ITerm = constrain(ITerm, -self->ITermLimit[axis], self->ITermLimit[axis]);
	} else {
		self->ITermLimit[axis] = ABS(ITerm);
	}
	self->lastITerm[axis] = ITerm;
	ITerm = ITerm >> 13; // take integer part of Q19.13 value

	// -----calculate D component
	int32_t DTerm = 0;
	if(self->config->D8[axis] != 0) {
		// delta calculated from measurement
		int32_t delta = -(gyroRate - self->lastRateForDelta[axis]);
		self->lastRateForDelta[axis] = gyroRate;
		// Divide delta by targetLooptime to get differential (ie dr/dt)
		delta = (delta * ((uint16_t)0xFFFF / (dt_us >> 4))) >> 5;
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

static void _multiwii_rewrite_update(struct anglerate *self, float dt) {
	UNUSED(dt); // TODO: maybe we should use dt?

	_anglerate_delta_state_update(self);

	// ----------PID controller----------
	for (int axis = 0; axis < 3; axis++) {
		const uint8_t rate = self->rate_config->rates[axis];

		// -----Get the desired angle rate depending on flight mode
		int32_t angleRate;
		if (axis == FD_YAW) {
			// YAW is always gyro-controlled (MAG correction is applied to user input)
			angleRate = (((int32_t)(rate + 27) * self->user[YAW]) >> 5);
		} else {
			// control is GYRO based for ACRO and HORIZON - direct sticks control is applied to rate PID
			angleRate = ((int32_t)(rate + 27) * self->user[axis]) >> 4;

			if(self->level_percent[axis] > 0){
				// calculate error angle and limit the angle to the max inclination
				// multiplication of user commands corresponds to changing the sticks scaling here
				const int32_t errorAngle = constrain(2 * self->user[axis], -((int)self->max_angle_inclination), self->max_angle_inclination)
						- self->body_angles[axis] + self->angle_trim->raw[axis];
				// blend in the angle based on level of blending
				angleRate += (errorAngle * self->config->P8[PIDLEVEL] * (uint16_t)self->level_percent[axis] / 100) >> 4;
			}
		}

		// --------low-level gyro-based PID. ----------
		const int32_t gyroRate = self->body_rates[axis] / 4;
		self->output.axis[axis] = _multiwii_rewrite_calc_axis(self, axis, gyroRate, angleRate, dt * 1e6f);
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

static void _luxfloat_update(struct anglerate *self, float dT){
	_anglerate_delta_state_update(self);

	// ----------PID controller----------
	for (int axis = 0; axis < 3; axis++) {
		const uint8_t rate = self->rate_config->rates[axis];

		// -----Get the desired angle rate depending on flight mode
		float angleRate;
		if (axis == FD_YAW) {
			// YAW is always gyro-controlled (MAG correction is applied to user input) 100dps to 1100dps max yaw rate
			angleRate = (float)((rate + 27) * self->user[YAW]) / 32.0f;
		} else {
			// control is GYRO based for ACRO and HORIZON - direct sticks control is applied to rate PID
			// also if we use blending then we lower the rate mode percentage depending on how much angle mode percentage we need
			angleRate = (float)(rate + 27) * self->user[axis] * ((100.0f - self->level_percent[axis]) / 100) / 16.0f; // 200dps to 1200dps max roll/pitch rate

			if(self->level_percent[axis] > 0){
				// calculate error angle and limit the angle to the max inclination
				// multiplication of user input corresponds to changing the sticks scaling here
				const float errorAngle = constrain(2 * self->user[axis], -((int)self->max_angle_inclination), self->max_angle_inclination)
						- self->body_angles[axis] + self->angle_trim->raw[axis];
				angleRate += errorAngle * self->config->P8[PIDLEVEL] * ((float)self->level_percent[axis] / 100.0f) / 16.0f;
			}
		}

		// --------low-level gyro-based PID. ----------
		// TODO: refactor this so that we always have gyro scaled to rad/s
		const float gyroRate = luxGyroScale * self->body_rates[axis] * (1.0f / 16.4f); // * (imu_get_gyro_scale(self->imu) / RAD); TODO: make sure that we are not dependent on gyro scale
		self->output.axis[axis] = _luxfloat_calc_axis(self, axis, gyroRate, angleRate, dT);
		//output->axis[axis] = constrain(output->axis[axis], -PID_LUX_FLOAT_MAX_PID, PID_LUX_FLOAT_MAX_PID);
	}
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
		case PID_CONTROLLER_MW23: // we no longer support the old mw23 controller
		case PID_CONTROLLER_MWREWRITE:
			self->update = _multiwii_rewrite_update;
			break;
		case PID_CONTROLLER_LUX_FLOAT:
			self->update = _luxfloat_update;
			break;
		case PID_COUNT:
			break;
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

void anglerate_update(struct anglerate *self, float dt){
	self->update(self, dt);
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
	if(axis > 2) return;
	self->PIDweight[axis] = weight;
}

void anglerate_input_body_rates(struct anglerate *self, int16_t x, int16_t y, int16_t z){
	self->body_rates[0] = x;
	self->body_rates[1] = y;
	self->body_rates[2] = z;
}

void anglerate_input_body_angles(struct anglerate *self, int16_t roll, int16_t pitch, int16_t yaw){
	self->body_angles[0] = roll;
	self->body_angles[1] = pitch;
	self->body_angles[2] = yaw;
}

void anglerate_input_user(struct anglerate *self, int16_t roll, int16_t pitch, int16_t yaw){
	self->user[0] = constrain(roll, -500, 500);
	self->user[1] = constrain(pitch, -500, 500);
	self->user[2] = constrain(yaw, -500, 500);
}

void anglerate_set_level_percent(struct anglerate *self, uint8_t roll, uint8_t pitch){
	self->level_percent[ROLL] = roll;
	self->level_percent[PITCH] = pitch;
}

/**
 * Ninjaflight Angle/Rate controller
 * =================================
 *
 * Cleanup and fixes: Martin Schröder <mkschreder.uk@gmail.com>
 * Original code: cleanflight
 *
 * The main responsibility of the anglerate controller is to control rotational
 * rate and attitude angle of the aircraft body. Inputs to the controller are
 * measured angular velocity (gyro rate) and measured attitude. Outputs are
 * desired actuator input that is proportional to desired rotational rate. Thus
 * the output of the controller controls aircraft body angular velocity (after
 * it is processed by the motor mixer).
 *
 * To understand how the controller works a few things need to be defined first:
 *
 * - coordinate system that is used for all rotations is right handed cartesian
 * system with x forward, y right and z down. Rotations are positive in cw
 * direction and negative in ccw direction.
 * - angular velocity is rotational speed of the aircraft in body frame.
 * - attitude is the angle relative to flat xy plane.
 *
 * Inputs and outputs
 * ==================
 *
 * - input: user controls. Roll, pitch, yaw. Range -500;500.
 * - input: gyro readings. Around XYZ axes. Units are gyro native units in
 * range -2^15 to +2^15. Typically this represents an angular velocity range of
 * -2000;+2000 degrees per second. gyro_scale parameter to init() determines
 * the scale.
 * - input: accel readings. A vector in body frame of aircraft linear
 * acceleration. Gravity is included in the reading although it is important to
 * realize that gravity appears as a vector that is _opposite_ of the direction
 * of gravitational pull. Acceleration is in range of acc_1G parameter passed
 * to the init function.
 * - output: frame controls. Roll, pitch, yaw. Range -500;500. These are
 * typically meant to be used in place of user input when driving the motor
 * mixer.
 *
 * This implementation of the controller supports blending between full angle
 * stabilization and full rate stabilization using a level strength value
 * betwen 0 and 100 percent. When controller is in rate mode (level correction
 * strength 0) only the angular velocity will be stabilized. When the controller
 * is in angle mode (level correction strength 100) then only the attitude
 * angle will be stabilized. Any other value between 0 and 100 causes the
 * controller to feed forward the control rates and mix them with desired angle
 * correction allowing pilot to do full spins while maintaining a degree of
 * angle correction.
 *
 * Current implementaiton includes two variations of the same basic algorithm
 * where one is using fixed point math while the other uses floating point
 * math. The original Multiwii23 controller has been removed to simplify code
 * maintenance. Even current implementation should probably be a generic one
 * without the need to resort to fixed point math.
 */
