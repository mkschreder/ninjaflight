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

#include "common/maths.h"

struct imu_quaternion {
	float w, x, y, z; 
}; 

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

struct imu_config {
    // IMU configuration
    uint16_t looptime;                      // imu loop time in us
    uint8_t gyroSync;                       // Enable interrupt based loop
    uint8_t gyroSyncDenominator;            // Gyro sync Denominator
    uint16_t dcm_kp;                        // DCM filter proportional gain ( x 10000)
    uint16_t dcm_ki;                        // DCM filter integral gain ( x 10000)
    uint8_t small_angle;                    // Angle used for mag hold threshold.
    uint16_t max_angle_inclination;         // max inclination allowed in angle (level) mode. default 500 (50 degrees).
};


struct throttle_correction_config {
    uint16_t throttle_correction_angle;     // the angle when the throttle correction is maximal. in 0.1 degres, ex 225 = 22.5 ,30.0, 450 = 45.0 deg
    uint8_t throttle_correction_value;      // the correction that will be applied at throttle_correction_angle.
};

struct imu {
	int16_t acc[XYZ_AXIS_COUNT]; 
	int16_t gyro[XYZ_AXIS_COUNT]; 

	int16_t accSmooth[XYZ_AXIS_COUNT];
	int32_t accSum[XYZ_AXIS_COUNT];

	float accTimeSum;        // keep track for integration of acc
	int accSumCount;
	float accVelScale;

	float throttleAngleScale;
	float fc_acc;
	float smallAngleCosZ;

	bool isAccelUpdatedAtLeastOnce;

	struct imu_config *config; 
	accelerometerConfig_t *acc_config;

	// TODO: replace with a math library quaternion 
	struct imu_quaternion q; 
	float rMat[3][3];

	union attitude_euler_angles attitude;     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800

	float gyroScale;
	uint16_t acc_1G; 
}; 

// TODO: remove once we are done refactoring
extern struct imu default_imu; 

void imu_init(struct imu *self);
void imu_configure(
	struct imu *self, 
	struct imu_config *config, 
	accelerometerConfig_t *acc_config,
	struct throttle_correction_config *thr_config,
	// TODO: refactor these
	float gyro_scale, 
	uint16_t acc_1G
);

void imu_input_accelerometer(struct imu *self, int16_t x, int16_t y, int16_t z);
void imu_input_gyro(struct imu *self, int16_t x, int16_t y, int16_t z);
void imu_update(struct imu *self);

int16_t imu_calc_throttle_angle_correction(struct imu *self, uint8_t throttle_correction_value);
int16_t imu_calc_heading(struct imu *self, t_fp_vector *vec);
float imu_get_cos_tilt_angle(struct imu *self);
bool imu_is_leveled(struct imu *self, uint8_t max_angle);

void imu_get_attitude_dd(struct imu *self, union attitude_euler_angles *att); 
void imu_get_raw_accel(struct imu *self, union imu_accel_reading *acc); 

// helper functions to extract a specific component from the attitude
int16_t imu_get_roll_dd(struct imu *self); 
int16_t imu_get_pitch_dd(struct imu *self); 
int16_t imu_get_yaw_dd(struct imu *self); 

float imu_get_avg_vertical_accel_cmss(struct imu *self); 
float imu_get_est_vertical_vel_cms(struct imu *self); 
float imu_get_velocity_integration_time(struct imu *self); 
void imu_reset_velocity_estimate(struct imu *self);

