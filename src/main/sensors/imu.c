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

// Inertial Measurement Unit (IMU)

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "common/maths.h"

#include "build_config.h"
#include <platform.h>
#include "debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/quaternion.h"

#include "config/runtime_config.h"
#include "config/parameter_group_ids.h"
#include "config/parameter_group.h"
#include "config/config.h"
#include "config/config_reset.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"

#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/sonar.h"

#include "flight/mixer.h"
#include "flight/anglerate.h"

#include "io/gps.h"

enum {
	IMU_FLAG_USE_ACC = (1 << 0),
	IMU_FLAG_USE_MAG = (1 << 1),
	IMU_FLAG_USE_YAW = (1 << 2),
	IMU_FLAG_DCM_CONVERGE_FASTER = (1 << 3)
};

// the limit (in degrees/second) beyond which we stop integrating
// omega_I. At larger spin rates the DCM PI controller can get 'dizzy'
// which results in false gyro drift. See
// http://gentlenav.googlecode.com/files/fastRotations.pdf
#define SPIN_RATE_LIMIT 20
static void _imu_update_dcm(struct imu *self){
	float q0q0 = sq(self->q.w);
	float q1q1 = sq(self->q.x);
	float q2q2 = sq(self->q.y);
	float q3q3 = sq(self->q.z);

	float q0q1 = self->q.w * self->q.x;
	float q0q2 = self->q.w * self->q.y;
	float q0q3 = self->q.w * self->q.z;
	float q1q2 = self->q.x * self->q.y;
	float q1q3 = self->q.x * self->q.z;
	float q2q3 = self->q.y * self->q.z;

	self->rMat[0][0] = q0q0 + q1q1 - q2q2 - q3q3;
	self->rMat[0][1] = 2.0f * (q1q2 - q0q3);
	self->rMat[0][2] = 2.0f * (q1q3 + q0q2);

	self->rMat[1][0] = 2.0f * (q1q2 + q0q3);
	self->rMat[1][1] = q0q0 - q1q1 + q2q2 - q3q3;
	self->rMat[1][2] = 2.0f * (q2q3 - q0q1);

	self->rMat[2][0] = 2.0f * (q1q3 - q0q2);
	self->rMat[2][1] = 2.0f * (q2q3 + q0q1);
	self->rMat[2][2] = q0q0 - q1q1 - q2q2 + q3q3;
}

void imu_reset(struct imu *self){
	self->q.w = 1.0f;
	self->q.x = 0.0f;
	self->q.y = 0.0f;
	self->q.z = 0.0f;

	_imu_update_dcm(self);

	// reset the sensor flags (these will then be set as new samples arrive!)
	self->flags &= ~(IMU_FLAG_USE_ACC | IMU_FLAG_USE_MAG);
}

void imu_init(struct imu *self,
	struct imu_config *imu_config,
	struct accelerometer_config *acc_config,
	struct throttle_correction_config *thr_config,
	float gyro_scale,
	uint16_t acc_1G
){
	memset(self, 0, sizeof(struct imu));

	self->config = imu_config;
	self->acc_config = acc_config;
	self->thr_config = thr_config;
	self->acc_1G = acc_1G;
	self->gyroScale = gyro_scale * (M_PIf / 180.0f);  // gyro output scaled to rad per second
	self->accVelScale = (9.80665f / self->acc_1G) * 100.0f; // acc vel scaled to cm/s

	imu_reset(self);
}

void imu_reset_velocity_estimate(struct imu *self){
	self->accSum[0] = 0;
	self->accSum[1] = 0;
	self->accSum[2] = 0;
	self->accSumCount = 0;
	self->accTimeSum = 0;
}

static void _imu_vector_bf_to_ef(struct imu *self, t_fp_vector * v){
	float x,y,z;

	/* From body frame to earth frame */
	x = self->rMat[0][0] * v->V.X + self->rMat[0][1] * v->V.Y + self->rMat[0][2] * v->V.Z;
	y = self->rMat[1][0] * v->V.X + self->rMat[1][1] * v->V.Y + self->rMat[1][2] * v->V.Z;
	z = self->rMat[2][0] * v->V.X + self->rMat[2][1] * v->V.Y + self->rMat[2][2] * v->V.Z;

	v->V.X = x;
	v->V.Y = -y;
	v->V.Z = z;
}

// rotate acc into Earth frame and calculate acceleration in it
static void _imu_update_acceleration(struct imu *self, float dT){
	static int32_t accZoffset = 0;
	static float accz_smooth = 0;
	t_fp_vector accel_ned;

	accel_ned.V.X = self->accSmooth[0];
	accel_ned.V.Y = self->accSmooth[1];
	accel_ned.V.Z = self->accSmooth[2];

	_imu_vector_bf_to_ef(self, &accel_ned);

	if(0){
		// TODO: remove this and make sure we initiate calibration outside and that it is done in instruments module
		if (self->acc_config->acc_unarmedcal == 1) {
			if (!ARMING_FLAG(ARMED)) {
				accZoffset -= accZoffset / 64;
				accZoffset += accel_ned.V.Z;
			}
			accel_ned.V.Z -= accZoffset / 64;  // compensate for gravitation on z-axis
		} else
			accel_ned.V.Z -= self->acc_1G;
	} else {
		accel_ned.V.Z -= self->acc_1G;
	}

	float fc_acc = 0.5f / (M_PIf * self->acc_config->accz_lpf_cutoff);
	accz_smooth = accz_smooth + (dT / (fc_acc + dT)) * (accel_ned.V.Z - accz_smooth); // low pass filter

	// apply Deadband to reduce integration drift and vibration influence
	self->accSum[X] += applyDeadband(lrintf(accel_ned.V.X), self->acc_config->accDeadband.xy);
	self->accSum[Y] += applyDeadband(lrintf(accel_ned.V.Y), self->acc_config->accDeadband.xy);
	self->accSum[Z] += applyDeadband(lrintf(accz_smooth), self->acc_config->accDeadband.z);

	// sum up Values for later integration to get velocity and distance
	self->accTimeSum += dT;
	self->accSumCount++;
}

static void _imu_mahony_update(struct imu *self, float dt, bool useAcc, bool useMag, bool useYaw, float yawError) {
	static float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki
	float recipNorm;
	float hx, hy, bx;
	float ex = 0, ey = 0, ez = 0;
	float gx, gy, gz;
	float ax, ay, az;
	float mx, my, mz;

	if(self->gyroScale > 0){
		gx = self->gyroScale * self->gyro[X];
		gy = self->gyroScale * self->gyro[Y];
		gz = self->gyroScale * self->gyro[Z];
	} else {
		gx = self->gyro[X];
		gy = self->gyro[Y];
		gz = self->gyro[Z];
	}

	// we scale here to keep precision when later squaring scale can be any number
	ax = self->accSmooth[X] * 0.001f;
	ay = self->accSmooth[Y] * 0.001f;
	az = self->accSmooth[Z] * 0.001f;

	mx = self->mag[X];
	my = self->mag[Y];
	mz = self->mag[Z];

	// Calculate general spin rate (rad/s)
	float spin_rate = sqrtf(sq(gx) + sq(gy) + sq(gz));

	// Use raw heading error (from GPS or whatever else)
	if (useYaw) {
		while (yawError >  M_PIf) yawError -= (2.0f * M_PIf);
		while (yawError < -M_PIf) yawError += (2.0f * M_PIf);

		ez += sin_approx(yawError / 2.0f);
	}

	// Use measured magnetic field vector
	recipNorm = sq(mx) + sq(my) + sq(mz);
	if (useMag && recipNorm > 0.01f) {
		// Normalise magnetometer measurement
		recipNorm = 1.0f / sqrtf(recipNorm);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
		// This way magnetic field will only affect heading and wont mess roll/pitch angles

		// (hx; hy; 0) - measured mag field vector in EF (assuming Z-component is zero)
		// (bx; 0; 0) - reference mag field vector heading due North in EF (assuming Z-component is zero)
		hx = self->rMat[0][0] * mx + self->rMat[0][1] * my + self->rMat[0][2] * mz;
		hy = self->rMat[1][0] * mx + self->rMat[1][1] * my + self->rMat[1][2] * mz;
		bx = sqrtf(hx * hx + hy * hy);

		// magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
		float ez_ef = -(hy * bx);

		// Rotate mag error vector back to BF and accumulate
		ex += self->rMat[2][0] * ez_ef;
		ey += self->rMat[2][1] * ez_ef;
		ez += self->rMat[2][2] * ez_ef;
	}

	// Use measured acceleration vector
	recipNorm = sq(ax) + sq(ay) + sq(az);
	if (useAcc && recipNorm > 1e-6f) {
		// Normalise accelerometer measurement
		recipNorm = 1.0f / sqrtf(recipNorm);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Error is sum of cross product between estimated direction and measured direction of gravity
		ex += (ay * self->rMat[2][2] - az * self->rMat[2][1]);
		ey += (az * self->rMat[2][0] - ax * self->rMat[2][2]);
		ez += (ax * self->rMat[2][1] - ay * self->rMat[2][0]);
	}

	// Compute and apply integral feedback if enabled
	float dcm_ki = self->config->dcm_ki / 10000.0f;
	if(dcm_ki > 0.0f) {
		// Stop integrating if spinning beyond the certain limit
		if (spin_rate < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)) {
			integralFBx += dcm_ki * ex * dt;	// integral error scaled by Ki
			integralFBy += dcm_ki * ey * dt;
			integralFBz += dcm_ki * ez * dt;
		}
	}
	else {
		integralFBx = 0.0f;	// prevent integral windup
		integralFBy = 0.0f;
		integralFBz = 0.0f;
	}

	// pgain is larger if not armed and just starting up (why?)
	float p_gain = (self->flags & IMU_FLAG_DCM_CONVERGE_FASTER)?10.0f:1.0f;

	// Calculate kP gain. If we are acquiring initial attitude (not armed and within 20 sec from powerup) scale the kP to converge faster
	float dcmKpGain = (self->config->dcm_kp / 10000.0f) * p_gain;

	// Apply proportional and integral feedback
	gx += dcmKpGain * ex + integralFBx;
	gy += dcmKpGain * ey + integralFBy;
	gz += dcmKpGain * ez + integralFBz;

	// rate integration is done using quaternion integration formula
	// qnew = qold + (qold * q(0, w, w, w) * 0.5) * dt;
	quat_t qdelta = { 0, gx, gy, gz };
	qdelta = quat_scale(&qdelta, 0.5f * dt);

	quat_t qnew = quat_mul(&self->q, &qdelta);
	self->q = quat_add(&self->q, &qnew);
	self->q = quat_normalize(&self->q);

	// Pre-compute rotation matrix from quaternion
	_imu_update_dcm(self);
}

static void _imu_update_euler_angles(struct imu *self){
	/* Compute pitch/roll angles */
	float px = self->rMat[2][0];
	float py = self->rMat[2][1];
	float pz = self->rMat[2][2];
	float sign_z = (pz > 0) - (pz < 0);

	self->attitude.values.roll = lrintf(atan2_approx(py, sign_z * sqrtf(pz * pz + 0.01f * px * px)) * (1800.0f / M_PIf));
	self->attitude.values.pitch = lrintf(atan2_approx(-px, sqrtf(py * py + pz * pz)) * (1800.0f / M_PIf));
	// TODO: fix the magnetic declination
	self->attitude.values.yaw = lrintf(-atan2_approx(self->rMat[1][0], self->rMat[0][0]) * (1800.0f / M_PIf));// + self->magneticDeclination);

	// yaw range is 0 to 3600
	if (self->attitude.values.yaw < 0)
		self->attitude.values.yaw += 3600;
}

bool imu_is_leveled(struct imu *self, uint8_t max_angle){
	/* Update small angle state */

	float armingAngleCosZ = cos_approx(degreesToRadians(max_angle));

	return (self->rMat[2][2] > armingAngleCosZ);
}

static bool _imu_mag_healthy(struct imu *self){
	UNUSED(self);
	return (self->mag[X] != 0) && (self->mag[Y] != 0) && (self->mag[Z] != 0);
}

void imu_update(struct imu *self, float dt){
	static filterStatePt1_t accLPFState[3];
	float rawYawError = 0;
	int32_t axis;

	// update smoothed acceleration
	// Smooth and use only valid accelerometer readings
	for (axis = 0; axis < 3; axis++) {
		if (self->acc_config->acc_cut_hz > 0) {
			self->accSmooth[axis] = filterApplyPt1(self->acc[axis], &accLPFState[axis], self->acc_config->acc_cut_hz, dt);
		} else {
			self->accSmooth[axis] = self->acc[axis];
		}
	}

	bool useAcc = (self->flags & IMU_FLAG_USE_ACC);
	bool useMag = (self->flags & IMU_FLAG_USE_MAG) && _imu_mag_healthy(self);
	bool useYaw = (self->flags & IMU_FLAG_USE_YAW);

	if(useYaw)
		rawYawError = DECIDEGREES_TO_RADIANS(self->attitude.values.yaw - self->yaw);

	// updates quaternion from sensors and then updates dcm
	_imu_mahony_update(self, dt,
		useAcc,
		useMag,
		useYaw, rawYawError);

	// reset yaw after we are done because we only use it if latest value has been provided
	self->flags &= ~IMU_FLAG_USE_YAW;

	// updates attitude from dcm
	_imu_update_euler_angles(self);

	// updates accSum from accSmooth
	_imu_update_acceleration(self, dt); // rotate acc vector into earth frame
}

void imu_input_accelerometer(struct imu *self, int16_t x, int16_t y, int16_t z){
	self->acc[X] = x;
	self->acc[Y] = y;
	self->acc[Z] = z;
	self->flags |= IMU_FLAG_USE_ACC;
}

void imu_input_magnetometer(struct imu *self, int16_t x, int16_t y, int16_t z){
	self->mag[X] = x;
	self->mag[Y] = y;
	self->mag[Z] = z;
	self->flags |= IMU_FLAG_USE_MAG;
}

void imu_input_gyro(struct imu *self, int16_t x, int16_t y, int16_t z){
	self->gyro[X] = x;
	self->gyro[Y] = y;
	self->gyro[Z] = z;
}

void imu_input_yaw_dd(struct imu *self, int16_t yaw){
	self->yaw = yaw;
	self->flags |= IMU_FLAG_USE_YAW;
}

void imu_get_rotation(struct imu *self, quat_t *q){
	*q = self->q;
}

float imu_get_cos_tilt_angle(struct imu *self){
	return self->rMat[2][2];
}

// TODO: this should probably be placed somewhere else because it is not strictly an imu function
int16_t imu_calc_throttle_angle_correction(struct imu *self, uint8_t throttle_correction_value){
	/*
	* Use 0 as the throttle angle correction if we are inverted, vertical or with a
	* small angle < 0.86 deg
	* TODO: Define this small angle in config.
	*/
	if (self->rMat[2][2] <= 0.015f) {
		return 0;
	}
	float throttleAngleScale = (1800.0f / M_PIf) * (900.0f / self->thr_config->throttle_correction_angle);
	int angle = lrintf(acos_approx(self->rMat[2][2]) * throttleAngleScale);
	if (angle > 900)
		angle = 900;
	return lrintf(throttle_correction_value * sin_approx(angle / (900.0f * M_PIf / 2.0f)));
}

void imu_get_attitude_dd(struct imu *self, union attitude_euler_angles *att){
	memcpy(att, &self->attitude, sizeof(union attitude_euler_angles));
}

void imu_get_gyro(struct imu *self, int16_t gyro[3]){
	gyro[0] = self->gyro[0];
	gyro[1] = self->gyro[1];
	gyro[2] = self->gyro[2];
}

float imu_get_gyro_scale(struct imu *self){
	return self->gyroScale;
}

void imu_get_raw_accel(struct imu *self, union imu_accel_reading *acc){
	acc->values.x = self->accSmooth[X];
	acc->values.y = self->accSmooth[Y];
	acc->values.z = self->accSmooth[Z];
}

int16_t imu_get_roll_dd(struct imu *self){
	return self->attitude.values.roll;
}

int16_t imu_get_pitch_dd(struct imu *self){
	return self->attitude.values.pitch;
}

int16_t imu_get_yaw_dd(struct imu *self){
	return self->attitude.values.yaw;
}

float imu_get_velocity_integration_time(struct imu *self){
	return self->accTimeSum; // delta acc reading time in seconds
}

// TODO: remove this function completely once althold has been refactored
float imu_get_avg_vertical_accel_cmss(struct imu *self){
	float acc_z = 0;
	// Integrator - velocity, cm/sec
	if (self->accSumCount) {
		acc_z = (float)self->accSum[2] / (float)self->accSumCount;
	}
	return acc_z;
}

float imu_get_est_vertical_vel_cms(struct imu *self){
	return imu_get_avg_vertical_accel_cmss(self) * self->accVelScale * (float)self->accTimeSum;
}

void imu_enable_fast_dcm_convergence(struct imu *self, bool on){
	if(on) self->flags |= IMU_FLAG_DCM_CONVERGE_FASTER;
	else self->flags &= ~IMU_FLAG_DCM_CONVERGE_FASTER;
}
