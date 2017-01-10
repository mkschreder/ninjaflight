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

#include "config/config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#include "rx/rx.h"

#include "anglerate.h"
#include "rate_profile.h"
#include "navigation.h"
#include "mixer.h"
#include "gtune.h"

#define ANGLERATE_FLAG_ANTIWINDUP	(1 << 0)
#define ANGLERATE_FLAG_PLIMIT		(1 << 1)
#define ANGLERATE_FLAG_OPENLOOP		(1 << 2)

static void _anglerate_delta_state_update(struct anglerate *self) {
	const struct pid_config *pid = &config_get_profile(self->config)->pid;
	if (!self->_delta_state_set && pid->dterm_cut_hz) {
		for (int axis = 0; axis < 3; axis++) {
			uint8_t sample_div = constrain(self->config->imu.gyro_sample_div, 1, 255);
			BiQuadNewLpf(pid->dterm_cut_hz, &self->deltaFilterState[axis], GYRO_STANDARD_RATE / sample_div);
		}
		self->_delta_state_set = true;
	}
}

// constants to scale pidLuxFloat so output is same as pidMultiWiiRewrite
static const float luxPTermScale = 1.0f; //1.0f / 64;
static const float luxITermScale = 1.0f; //1.0f / 16.777216f;
static const float luxDTermScale = 1.0f; //0.000128f;

static int16_t _anglerate_calc_axis(struct anglerate *self, int axis, const float gyroRate, const float rateError, float dT){
	const struct config_profile *profile = config_get_profile(self->config);
	const struct pid_config *pid = &profile->pid;

	float ku = pid->P8[axis] * 0.1f;
	float tu = pid->I8[axis] * 0.01f;
	float kp = 0.6f * ku;
	float ki = 0; // for rate we use a PD controller. //(1.2f * ku) / tu;
	float kd = (0.6f * ku * tu) / 8;

	// -----calculate P component
	float PTerm = rateError * kp * self->PIDweight[axis] / 100;

	// Constrain YAW by yaw_p_limit value if not servo driven, in that case servolimits apply
	/*
	if (axis == YAW && pid->yaw_p_limit && (self->flags & ANGLERATE_FLAG_PLIMIT)) {
		PTerm = constrainf(PTerm, -pid->yaw_p_limit, pid->yaw_p_limit);
	}
	*/

	// -----calculate I component
	float ITerm = self->lastITermf[axis] + luxITermScale * rateError * dT * ki;
	// generic antiwindup
	if(ITerm < -PID_MAX_I || ITerm > PID_MAX_I)
		ITerm = ITerm - self->output.axis_I[axis];
	ITerm = constrainf(ITerm, -PID_MAX_I, PID_MAX_I);
	self->lastITermf[axis] = ITerm;
	/*
	// limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
	// I coefficient (I8) moved before integration to make limiting independent from PID settings
	// Anti windup protection
	if (self->flags & ANGLERATE_FLAG_ANTIWINDUP) {
		ITerm = constrainf(ITerm, -self->ITermLimitf[axis], self->ITermLimitf[axis]);
	} else {
		self->ITermLimitf[axis] = ABS(ITerm);
	}
	self->lastITermf[axis] = ITerm;
	*/

	// -----calculate D component
	float DTerm;
	if (kd < 1e-6f) {
		// optimisation for when D8 is zero, often used by YAW axis
		DTerm = 0;
	} else {
		// delta calculated from measurement
		float delta = -(gyroRate - self->lastRateForDelta[axis]);
		self->lastRateForDelta[axis] = gyroRate;
		// Divide delta by dT to get differential (ie dr/dt)
		delta *= (1.0f / dT);
		/*
		if (pid->dterm_cut_hz) {
			// DTerm delta low pass filter
			delta = applyBiQuadFilter(delta, &self->deltaFilterState[axis]);
		} else {
			// When DTerm low pass filter disabled apply moving average to reduce noise
			delta = filterApplyAveragef(delta, DTERM_AVERAGE_COUNT, self->deltaStatef[axis]);
		}
		*/
		DTerm = delta * kd * self->PIDweight[axis] / 100;
		DTerm = constrainf(DTerm, -PID_MAX_D, PID_MAX_D);
	}

	self->output.axis_P[axis] = PTerm;
	self->output.axis_I[axis] = ITerm;
	self->output.axis_D[axis] = DTerm;
	// -----calculate total PID output
	return lrintf(PTerm + ITerm + DTerm);
}

void anglerate_update(struct anglerate *self, float dT){
	if(self->flags & ANGLERATE_FLAG_OPENLOOP) {
		for(int c = 0; c < 3; c++){
			self->output.axis[c] = self->user[c];
		}
		return;
	}

	_anglerate_delta_state_update(self);

	const struct rate_profile *rp = config_get_rate_profile(self->config);
	const struct config_profile *profile = config_get_profile(self->config);
	const struct pid_config *pid = &profile->pid;

	// ----------PID controller----------
	for (int axis = 0; axis < 3; axis++) {
		const uint8_t rate = rp->rates[axis];

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
				const float maxangle = self->config->imu.max_angle_inclination;
				const float errorAngle = constrain(self->user[axis], -((int)maxangle), maxangle)
						- self->body_angles[axis] + profile->acc.trims.raw[axis];
				
				float ku = pid->P8[PIDLEVEL] * 0.1f;
				float tu = pid->I8[PIDLEVEL] * 0.1f;
				float kp = 0.6f * ku;
				float ki = (1.2f * ku) / tu;
				float kd = (0.6f * ku * tu) / 8;

				self->angle_I[axis] = constrainf(self->angle_I[axis] + errorAngle * dT, -256, 256);
				float ITerm = self->angle_I[axis];
				if(ITerm < -PID_MAX_I || ITerm > PID_MAX_I)
					ITerm = ITerm - self->angle_I[axis];

				float DTerm = (self->body_angles[axis] - self->angle_E[axis]);
				self->angle_E[axis] = self->body_angles[axis];

				angleRate = errorAngle * kp + ITerm * ki + DTerm * kd;
				//angleRate += errorAngle * pid->P8[PIDLEVEL] * ((float)self->level_percent[axis] / 100.0f) / 16.0f + ITerm + DTerm;
			}
		}

		// --------low-level gyro-based PID. ----------
		// gyro rates converted into deg/s
		const float gyroRate = self->body_rates[axis] * SYSTEM_GYRO_SCALE;
		self->output.axis[axis] = constrain(_anglerate_calc_axis(self, axis, gyroRate, angleRate - gyroRate, dT), -500, 500);
	}
}

void anglerate_init(struct anglerate *self,
	struct instruments *ins, const struct config const *config){
	memset(self, 0, sizeof(struct anglerate));
	self->ins = ins;
	for(int c = 0; c < 3; c++) {
		self->pidScale[c] = 100;
		self->PIDweight[c] = 100;
	}
	self->config = config;
}

void anglerate_reset_angle_i(struct anglerate *self){
	self->ITermAngle[AI_ROLL] = 0;
	self->ITermAngle[AI_PITCH] = 0;
}

void anglerate_reset_rate_i(struct anglerate *self) {
	memset(self->lastITermf, 0, sizeof(self->lastITermf));
}

const struct pid_controller_output *anglerate_get_output_ptr(struct anglerate *self){
	return &self->output;
}

void anglerate_enable_plimit(struct anglerate *self, bool on){
	if(on) self->flags |= ANGLERATE_FLAG_PLIMIT;
	else self->flags &= ~ANGLERATE_FLAG_PLIMIT;
}

void anglerate_enable_antiwindup(struct anglerate *self, bool on){
	if(on) self->flags |= ANGLERATE_FLAG_ANTIWINDUP;
	else self->flags &= ~ANGLERATE_FLAG_ANTIWINDUP;
}

void anglerate_set_openloop(struct anglerate *self, bool on){
	if(on) self->flags |= ANGLERATE_FLAG_OPENLOOP;
	else self->flags &= ~ANGLERATE_FLAG_OPENLOOP;
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
