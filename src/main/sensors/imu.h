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

#pragma once

#include <stdbool.h>

#include "common/axis.h"
#include "common/maths.h"
#include "common/quaternion.h"
#include "common/filter.h"
#include "../config/imu.h"

union imu_accel_reading {
	int16_t raw[3];
	struct {
		int16_t x, y, z;
	} values;
};

union attitude_euler_angles {
    int16_t raw[XYZ_AXIS_COUNT];
    struct {
        // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
    } values;
};

// TODO: move these into math lib
typedef int32_t gyro_rates_t[3];
typedef int32_t acc_rates_t[3];
typedef union attitude_euler_angles euler_angles_t;

struct imu {
	int16_t acc[XYZ_AXIS_COUNT];
	int16_t gyro[XYZ_AXIS_COUNT];
	int16_t mag[XYZ_AXIS_COUNT];

	int16_t accSmooth[XYZ_AXIS_COUNT];
	int32_t accSum[XYZ_AXIS_COUNT];

	float accTimeSum;        // keep track for integration of acc
	int accSumCount;
	float accVelScale;

	uint8_t flags;

	quat_t q;

	float rMat[3][3];

	filterStatePt1_t accLPFState[3];

	union attitude_euler_angles attitude;     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800

	// TODO: investigate if we can refactor this so that we can pass already precomputed mag data without need to speficy this
	float magneticDeclination;

	// this is yaw that we are currently inputing from gps or some other source
	int16_t yaw;

	//float gyroScale;
	//int16_t acc_1G;

	const struct config *config;
};

void imu_init(struct imu *self, const struct config *config);

void imu_reload_config(struct imu *self);
void imu_input_accelerometer(struct imu *self, int16_t x, int16_t y, int16_t z);
void imu_input_gyro(struct imu *self, int16_t x, int16_t y, int16_t z);
void imu_input_magnetometer(struct imu *self, int16_t x, int16_t y, int16_t z);
void imu_input_yaw_dd(struct imu *self, int16_t yaw);
void imu_update(struct imu *self, float dt);

void imu_reset(struct imu *self);
int16_t imu_calc_throttle_angle_correction(struct imu *self, uint8_t throttle_correction_value);
int16_t imu_calc_heading(struct imu *self, t_fp_vector *vec);
float imu_get_cos_tilt_angle(struct imu *self);
bool imu_is_leveled(struct imu *self, uint8_t max_angle);

void imu_get_attitude_dd(struct imu *self, union attitude_euler_angles *att);
void imu_get_raw_accel(struct imu *self, union imu_accel_reading *acc);

//void imu_set_acc_scale(struct imu *self, int16_t acc_1G);
//void imu_set_gyro_scale(struct imu *self, float gyro_scale);

// TODO: replace with get_angular_velocity once we have refactored pids to use radians/s
//void imu_get_gyro(struct imu *self, int16_t gyro[3]);

// TODO: this needs to be removed since it is only used in anglerate controller. Leaving for now since don't want to make too many changes.
//float imu_get_gyro_scale(struct imu *self);

// helper functions to extract a specific component from the attitude
int16_t imu_get_roll_dd(struct imu *self);
int16_t imu_get_pitch_dd(struct imu *self);
int16_t imu_get_yaw_dd(struct imu *self);

void imu_get_rotation(struct imu *self, quat_t *q);

void imu_enable_fast_dcm_convergence(struct imu *self, bool on);

float imu_get_avg_vertical_accel_cmss(struct imu *self);
float imu_get_est_vertical_vel_cms(struct imu *self);
float imu_get_velocity_integration_time(struct imu *self);
void imu_reset_velocity_estimate(struct imu *self);

