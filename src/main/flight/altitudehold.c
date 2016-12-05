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
#include <math.h>


#include <platform.h>

#include "debug.h"

#include "common/maths.h"
#include "common/axis.h"

#include "config/config.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/sonar_hcsr04.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/sonar.h"

#include "rx/rx.h"

#include "flight/mixer.h"
#include "flight/anglerate.h"

#include "flight/altitudehold.h"

#include "ninja.h"

#include <string.h>

// 40hz update rate (20hz LPF on acc)
#define BARO_UPDATE_FREQUENCY_40HZ (1000 * 25)

#define DEGREES_80_IN_DECIDEGREES 800

void althold_init(struct althold *self, struct instruments *ins, struct rx *rx, const struct config const *config){
	memset(self, 0, sizeof(struct althold));
	self->rx = rx;
	self->ins = ins;
	self->config = config;
}

static void _apply_multi_althold(struct althold *self){
	const struct rc_controls_config *rc = &config_get_profile(self->config)->rc;
	// multirotor alt hold
	if (rc->alt_hold_fast_change) {
		// rapid alt changes
		if (ABS(rx_get_channel(self->rx, THROTTLE) - self->initialRawThrottleHold) > rc->alt_hold_deadband) {
			self->errorVelocityI = 0;
			self->isAltHoldChanged = 1;
			self->throttle = (rx_get_channel(self->rx, THROTTLE) > self->initialRawThrottleHold) ? -rc->alt_hold_deadband : rc->alt_hold_deadband;
		} else {
			if (self->isAltHoldChanged) {
				self->AltHold = self->EstAlt;
				self->isAltHoldChanged = 0;
			}
			self->throttle = constrain(self->initialThrottleHold + self->altHoldThrottleAdjustment, self->config->pwm_out.minthrottle, self->config->pwm_out.maxthrottle);
		}
	} else {
		// slow alt changes, mostly used for aerial photography
		if (ABS(rx_get_channel(self->rx, THROTTLE) - self->initialRawThrottleHold) > rc->alt_hold_deadband) {
			// set velocity proportional to stick movement +100 throttle gives ~ +50 cm/s
			self->setVelocity = (rx_get_channel(self->rx, THROTTLE) - self->initialRawThrottleHold) / 2;
			self->velocityControl = 1;
			self->isAltHoldChanged = 1;
		} else if (self->isAltHoldChanged) {
			self->AltHold = self->EstAlt;
			self->velocityControl = 0;
			self->isAltHoldChanged = 0;
		}
		self->throttle = constrain(self->initialThrottleHold + self->altHoldThrottleAdjustment, self->config->pwm_out.minthrottle, self->config->pwm_out.maxthrottle);
	}
}

void althold_apply(struct althold *self){
	// TODO: generic althold that works for both airplane and multi
	/*
	if (STATE(FIXED_WING)) {
		_apply_airplane_althold(self);
	} else {
		_apply_multi_althold(self);
	}
	*/
	_apply_multi_althold(self);
}

void althold_update(struct althold *self){
	self->AltHold = self->EstAlt;
	self->initialRawThrottleHold = rx_get_channel(self->rx, THROTTLE);
	self->initialThrottleHold = 0;
	self->errorVelocityI = 0;
	self->altHoldThrottleAdjustment = 0;
}

static int32_t _calc_althold_throttle_delta(struct althold *self, int32_t vel_tmp, float accZ_tmp, float accZ_old){
	int32_t result = 0;
	int32_t error;
	int32_t setVel;

	const struct pid_config *pid = &config_get_profile(self->config)->pid;

	bool is_thrust_downwards = ABS(ins_get_roll_dd(self->ins)) < DEGREES_80_IN_DECIDEGREES && ABS(ins_get_pitch_dd(self->ins)) < DEGREES_80_IN_DECIDEGREES;
	if(!is_thrust_downwards){
		return result;
	}

	// Altitude P-Controller

	if (!self->velocityControl) {
		error = constrain(self->AltHold - self->EstAlt, -500, 500);
		error = applyDeadband(error, 10); // remove small P parameter to reduce noise near zero position
		setVel = constrain((pid->P8[PIDALT] * error / 128), -300, +300); // limit velocity to +/- 3 m/s
	} else {
		setVel = self->setVelocity;
	}
	// Velocity PID-Controller

	// P
	error = setVel - vel_tmp;
	result = constrain((pid->P8[PIDVEL] * error / 32), -300, +300);

	// I
	self->errorVelocityI += (pid->I8[PIDVEL] * error);
	self->errorVelocityI = constrain(self->errorVelocityI, -(8192 * 200), (8192 * 200));
	result += self->errorVelocityI / 8192;	 // I in range +/-200

	// D
	result -= constrain(pid->D8[PIDVEL] * (accZ_tmp + accZ_old) / 512, -150, 150);

	return result;
}

void althold_calculate_altitude(struct althold *self, uint32_t currentTime){
	static uint32_t previousTime;
	uint32_t dTime;
	int32_t baroVel;
	//float dt;
	//float vel_acc;
	int32_t vel_tmp;
	static float accZ_old = 0.0f;
	static float vel = 0.0f;
	static float accAlt = 0.0f;
	static int32_t lastBaroAlt;

#ifdef SONAR
	int32_t sonarAlt = SONAR_OUT_OF_RANGE;
	static int32_t baroAlt_offset = 0;
	float sonarTransition;
#endif

	const struct config_profile *profile = config_get_profile(self->config);

	dTime = currentTime - previousTime;
	if (dTime < BARO_UPDATE_FREQUENCY_40HZ)
		return;

	previousTime = currentTime;

#ifdef BARO
	if (!isBaroCalibrationComplete()) {
		performBaroCalibrationCycle();
		vel = 0;
		accAlt = 0;
	}

	BaroAlt = baroCalculateAltitude();
#else
	BaroAlt = 0;
#endif

#ifdef SONAR
	sonarAlt = sonar_read(&default_sonar);

	// TODO: make this sonar calculation work after refactoring
	//sonarAlt = sonar_calc_altitude(&default_sonar, imu_get_cos_tilt_angle(&default_imu));

	if (sonarAlt > 0 && sonarAlt < default_sonar.cf_alt_cm) {
		// just use the SONAR
		baroAlt_offset = BaroAlt - sonarAlt;
		BaroAlt = sonarAlt;
	} else {
		BaroAlt -= baroAlt_offset;
		if (sonarAlt > 0  && sonarAlt <= default_sonar.max_alt_with_tilt_cm) {
			// SONAR in range, so use complementary filter
			sonarTransition = (float)(default_sonar.max_alt_with_tilt_cm - sonarAlt) / (default_sonar.max_alt_with_tilt_cm - default_sonar.cf_alt_cm);
			BaroAlt = sonarAlt * sonarTransition + BaroAlt * (1.0f - sonarTransition);
		}
	}
#endif

/* TODO: make this kind of thing work without so much interaction with the internals of imu!
 
	dt = imu_get_velocity_integration_time(&default_imu); 
	vel_acc = imu_get_est_vertical_vel_cms(&default_imu); 

	// Integrator - Altitude in cm
	accAlt += (vel_acc * 0.5f) * dt + vel * dt;																 // integrate velocity to get distance (x= a/2 * t^2)
	accAlt = accAlt * barometerConfig()->baro_cf_alt + (float)BaroAlt * (1.0f - barometerConfig()->baro_cf_alt);	// complementary filter for altitude estimation (baro & acc)
	vel += vel_acc;

#ifdef DEBUG_ALT_HOLD
	debug[1] = accSum[2] / accSumCount; // acceleration
	debug[2] = vel;					 // velocity
	debug[3] = accAlt;				  // height
#endif

	imu_reset_velocity_estimate(&default_imu);
*/
#ifdef BARO
	if (!isBaroCalibrationComplete()) {
		return;
	}
#endif

#ifdef SONAR
	// TODO: pass sonar object to althold and read altitude from there
	if (sonarAlt > 0 && sonarAlt < default_sonar.cf_alt_cm) {
		// the sonar has the best range
		self->EstAlt = BaroAlt;
	} else {
		self->EstAlt = accAlt;
	}
#else
	self->EstAlt = accAlt;
#endif

	baroVel = (BaroAlt - lastBaroAlt) * 1000000.0f / dTime;
	lastBaroAlt = BaroAlt;

	baroVel = constrain(baroVel, -1500, 1500);  // constrain baro velocity +/- 1500cm/s
	baroVel = applyDeadband(baroVel, 10);	   // to reduce noise near zero

	// apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
	// By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
	vel = vel * profile->baro.baro_cf_vel + baroVel * (1.0f - profile->baro.baro_cf_vel);
	vel_tmp = lrintf(vel);

	// set vario
	self->vario = applyDeadband(vel_tmp, 5);

	// TODO: below line gets average vertical acceleration. It is nonsense. We should fix this once done refactoring (when the change can be tested)

	// TODO: make this code work after refactoring
	float acc_z = 0; //imu_get_avg_vertical_accel_cmss(&default_imu); 
	self->altHoldThrottleAdjustment = _calc_althold_throttle_delta(self, vel_tmp, acc_z, accZ_old);

	accZ_old = acc_z;
}

int32_t althold_get_est_alt(struct althold *self){
	return self->EstAlt;
}


