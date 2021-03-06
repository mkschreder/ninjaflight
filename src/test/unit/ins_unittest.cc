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

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include <limits.h>

#define BARO

extern "C" {
    #include <platform.h>
    #include "build_config.h"
    #include "debug.h"

    #include "common/axis.h"
    #include "common/maths.h"

	#include "config/config.h"

    #include "drivers/sensor.h"
    #include "drivers/accgyro.h"
    #include "drivers/compass.h"

    #include "sensors/gyro.h"
    #include "sensors/compass.h"
    #include "sensors/acceleration.h"
    #include "sensors/barometer.h"

    #include "config/config.h"

    #include "rx/rx.h"

    #include "flight/mixer.h"
    #include "flight/anglerate.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static struct config_store config;

#define ACC_45DEG (cos(M_PI/4) * SYSTEM_ACCEL_1G)

void input_accel(struct instruments *self, int16_t x, int16_t y, int16_t z){
	ins_reset_imu(self);
	for(unsigned int c = 0; c < 4000UL; c++){
		ins_process_acc(self, x, y, z);
		ins_process_gyro(self, 0, 0, 0);
		ins_update(self, 0.02);
	}
}

void input_mag(struct instruments *self, int16_t x, int16_t y, int16_t z){
	ins_reset_imu(self);

	for(unsigned int c = 0; c < 4000UL; c++){
		ins_process_gyro(self, 0, 0, 0);
		ins_process_mag(self, x, y, z);
		ins_update(self, 0.05);
	}
}


void input_gyro(struct instruments *self, int16_t x, int16_t y, int16_t z, unsigned int duration){
	ins_reset_imu(self);
	int16_t xx = 32768.0f / (20000.0f / x);
	int16_t yy = 32768.0f / (20000.0f / y);
	int16_t zz = 32768.0f / (20000.0f / z);
	ins_process_gyro(self, xx, yy, zz);
	for(unsigned int c = 0; c < duration; c++){
		ins_update(self, 0.001);
	}
}

void reset_trims(void){
	config_get_profile_rw(&config.data)->acc.trims.raw[0] = 0;
	config_get_profile_rw(&config.data)->acc.trims.raw[1] = 0;
	config_get_profile_rw(&config.data)->acc.trims.raw[2] = 0;
	config.data.sensors.trims.accZero.raw[0] = 0;
	config.data.sensors.trims.accZero.raw[1] = 0;
	config.data.sensors.trims.accZero.raw[2] = 0;
}

TEST(InsUnitTest, TestAccCalibration){
	struct instruments ins; 

	config_reset(&config);

	config.data.imu.dcm_kp = 2500;
    config.data.imu.looptime = 2000;
    config.data.imu.small_angle = 25;
    config.data.imu.max_angle_inclination = 500;

	// set zero such that we are tilten in roll 10 deg
	int16_t zx = 0;
	int16_t zy = sin(10 * RAD) * SYSTEM_ACCEL_1G;
	int16_t zz = cos(10 * RAD) * SYSTEM_ACCEL_1G;

	ins_init(&ins, &config.data);

	EXPECT_EQ(0, ins_get_acc_x(&ins));
	EXPECT_EQ(0, ins_get_acc_y(&ins));
	EXPECT_EQ(SYSTEM_ACCEL_1G, ins_get_acc_z(&ins));

	ins_reset_imu(&ins);
	ins_start_acc_calibration(&ins);
	EXPECT_EQ(false, ins_is_calibrated(&ins));
	for(int c = 0; c < 1000; c++){
		ins_process_acc(&ins, zx, zy, zz + SYSTEM_ACCEL_1G);
		ins_process_gyro(&ins, 0, 0, 0);
		ins_update(&ins, 0.001);
	}
	EXPECT_EQ(true, ins_is_calibrated(&ins));

	EXPECT_EQ(0, ins_get_acc_x(&ins));
	EXPECT_EQ(0, ins_get_acc_y(&ins));
	EXPECT_EQ(SYSTEM_ACCEL_1G, ins_get_acc_z(&ins));

	// test a few variations of the acceleration vector to make sure we get correct results
	input_accel(&ins, zx + SYSTEM_ACCEL_1G, zy, zz);

	EXPECT_EQ(SYSTEM_ACCEL_1G, ins_get_acc_x(&ins));
	EXPECT_EQ(0, ins_get_acc_y(&ins));
	EXPECT_EQ(0, ins_get_acc_z(&ins));

    EXPECT_EQ(0, ins_get_roll_dd(&ins));
    EXPECT_EQ(-900, ins_get_pitch_dd(&ins));
    EXPECT_EQ(0, ins_get_yaw_dd(&ins));

	int16_t tv = sin(45.0f * RAD) * SYSTEM_ACCEL_1G;
    input_accel(&ins, zx + tv, zy, zz + tv);

	EXPECT_EQ(tv, ins_get_acc_x(&ins));
	EXPECT_EQ(0, ins_get_acc_y(&ins));
	EXPECT_EQ(tv, ins_get_acc_z(&ins));

    EXPECT_EQ(0, ins_get_roll_dd(&ins));
    EXPECT_EQ(-450, ins_get_pitch_dd(&ins));
    EXPECT_EQ(0, ins_get_yaw_dd(&ins));

    input_accel(&ins, zx, zy + tv, zz + tv);

    EXPECT_EQ(0, ins_get_acc_x(&ins));
    EXPECT_EQ(tv, ins_get_acc_y(&ins));
    EXPECT_EQ(tv, ins_get_acc_z(&ins));

    EXPECT_EQ(450, ins_get_roll_dd(&ins));
    EXPECT_EQ(0, ins_get_pitch_dd(&ins));
    EXPECT_EQ(0, ins_get_yaw_dd(&ins));

    input_accel(&ins, zx, zy + cos(M_PI/4) * SYSTEM_ACCEL_1G, zz + cos(M_PI/4) * SYSTEM_ACCEL_1G);

    EXPECT_EQ(0, ins_get_acc_x(&ins));
    EXPECT_EQ((int16_t)(cos(M_PI/4) * SYSTEM_ACCEL_1G), ins_get_acc_y(&ins));
    EXPECT_EQ((int16_t)(cos(M_PI/4) * SYSTEM_ACCEL_1G), ins_get_acc_z(&ins));

	// this is now 45 deg since we have standardized acc_1G
    EXPECT_EQ(450, ins_get_roll_dd(&ins));
    EXPECT_EQ(0, ins_get_pitch_dd(&ins));
    EXPECT_EQ(0, ins_get_yaw_dd(&ins));

	// reset trims since they will be used in other unit tests
	reset_trims();
}

TEST(InsUnitTest, TestGyroCalibration){
	struct instruments ins; 

	config_reset(&config);
	config.data.imu.dcm_kp = 2500;
    config.data.imu.looptime = 2000;
    config.data.imu.small_angle = 25;
    config.data.imu.max_angle_inclination = 500;

	ins_init(&ins, &config.data);

	EXPECT_EQ(0, ins_get_acc_x(&ins));
	EXPECT_EQ(0, ins_get_acc_y(&ins));
	EXPECT_EQ(SYSTEM_ACCEL_1G, ins_get_acc_z(&ins));

	ins_reset_imu(&ins);
	ins_start_gyro_calibration(&ins);
	EXPECT_EQ(false, ins_is_calibrated(&ins));

	// a rand noise will generate gyroZero of around ns/2
	int16_t ns = 600;
	const int16_t gyro_noise_err = 4; // allow for a little error room since we are using pseudorandom numbers

	// first try calibrating while moving.
	// set move threshold deviation to 50 ( with ns = 300 the deviation will be around 90)
	config.data.gyro.move_threshold = 50;
	for(int c = 0; c < 1000; c++){
		ins_process_acc(&ins, 0, 0, SYSTEM_ACCEL_1G);
		ins_process_gyro(&ins, rand() % ns, rand() % ns, rand() % ns);
		ins_update(&ins, 0.001);
	}

	printf("gyro move threshold: %d, deviation: %f\n", config.data.gyro.move_threshold, devStandardDeviation(&ins.gyro.var[0]));
	// expect calibration failed
	EXPECT_EQ(false, ins_is_calibrated(&ins));

	// this will reset the deviation
	//ins_start_gyro_calibration(&ins);
	// now set ns to 100 and recalibrate (now deviation will be around 30)
	ns = 100;
	for(int c = 0; c < 1000; c++){
		ins_process_acc(&ins, 0, 0, SYSTEM_ACCEL_1G);
		ins_process_gyro(&ins, rand() % ns, rand() % ns, rand() % ns);
		ins_update(&ins, 0.001);
	}
	EXPECT_EQ(true, ins_is_calibrated(&ins));

	printf("gyro move threshold: %d, deviation: %f\n", config.data.gyro.move_threshold, devStandardDeviation(&ins.gyro.var[0]));
	printf("gyro noise offset: %d %d %d\n", ins.gyro.gyroZero[0], ins.gyro.gyroZero[1], ins.gyro.gyroZero[2]);

	EXPECT_EQ(true, ABS(ins.gyro.gyroZero[0] - (ns / 2)) < gyro_noise_err);
	EXPECT_EQ(true, ABS(ins.gyro.gyroZero[1] - (ns / 2)) < gyro_noise_err);
	EXPECT_EQ(true, ABS(ins.gyro.gyroZero[2] - (ns / 2)) < gyro_noise_err);
}

void print_ins(struct instruments *self){
	printf("roll: %d, pitch: %d, yaw: %d\n", ins_get_roll_dd(self), ins_get_pitch_dd(self), ins_get_yaw_dd(self));
}

TEST(InsUnitTest, TestGyroIntegration){
	struct instruments ins;

	// error margin for gyro integration: 0.2 deg
	const int margin = 2;

	config_reset(&config);
	config.data.imu.dcm_kp = 2500;
    config.data.imu.looptime = 2000;
    config.data.imu.small_angle = 25;
    config.data.imu.max_angle_inclination = 500;

	// this is just so we get it initialized during init and we then actually disable it
	config.data.gyro.soft_gyro_lpf_hz = 500;

	ins_init(&ins, &config.data);

	// simulate calibration
	ins_reset_imu(&ins);
	for(int c = 0; c < 1000; c++){
		ins_process_acc(&ins, 0, 0, 1024);
		ins_process_gyro(&ins, 0, 0, 0);
		ins_update(&ins, 0.001);
	}
	EXPECT_EQ(true, ins_is_calibrated(&ins));

	// we specified soft lpf so we expect filter to be enabled
	EXPECT_EQ(true, ins.gyro.use_filter);
	ins_set_gyro_filter_hz(&ins, 0); // disable the filter

	// check that it was disabled
	EXPECT_EQ(false, ins.gyro.use_filter);

	input_gyro(&ins, 0, 0, 0, 1000);
    EXPECT_EQ(0, ins_get_roll_dd(&ins));
    EXPECT_EQ(0, ins_get_pitch_dd(&ins));
    EXPECT_EQ(0, ins_get_yaw_dd(&ins));

	// simulate a 90 deg counter clockwise rotation around Z axis using just gyro
	input_gyro(&ins, 0, 0, 900, 1000);
    EXPECT_EQ(0, ins_get_roll_dd(&ins));
    EXPECT_EQ(0, ins_get_pitch_dd(&ins));
    EXPECT_EQ(true, ABS(ins_get_yaw_dd(&ins) - 2700) < margin);

	// simulate a 45 deg counter clockwise rotation around y axis
	input_gyro(&ins, 0, 450, 0, 1000);
    EXPECT_EQ(0, ins_get_roll_dd(&ins));
    EXPECT_EQ(true, ABS(ins_get_pitch_dd(&ins) - 450) < margin);
    EXPECT_EQ(0, ins_get_yaw_dd(&ins));

	// simulate a 120 deg counterclockwise roll around x axis
	input_gyro(&ins, 1200, 0, 0, 1000);
    EXPECT_EQ(true, ABS(ins_get_roll_dd(&ins) - 1200) < margin);
    EXPECT_EQ(0, ins_get_pitch_dd(&ins));
    EXPECT_EQ(0, ins_get_yaw_dd(&ins));

	// now test with filter
	ins_set_gyro_filter_hz(&ins, 500);
	EXPECT_EQ(true, ins.gyro.use_filter);

	input_gyro(&ins, 0, 0, 0, 1000);
    EXPECT_EQ(0, ins_get_roll_dd(&ins));
    EXPECT_EQ(0, ins_get_pitch_dd(&ins));
    EXPECT_EQ(0, ins_get_yaw_dd(&ins));

	// simulate a 90 deg counter clockwise rotation around Z axis using just gyro
	input_gyro(&ins, 0, 0, 900, 1000);
    EXPECT_EQ(0, ins_get_roll_dd(&ins));
    EXPECT_EQ(0, ins_get_pitch_dd(&ins));
    EXPECT_EQ(true, ABS(ins_get_yaw_dd(&ins) - 2700) < margin);
	print_ins(&ins);

	// simulate a 45 deg counter clockwise rotation around y axis
	input_gyro(&ins, 0, 450, 0, 1000);
    EXPECT_EQ(0, ins_get_roll_dd(&ins));
    EXPECT_EQ(true, ABS(ins_get_pitch_dd(&ins) - 450) < margin);
    EXPECT_EQ(0, ins_get_yaw_dd(&ins));
	print_ins(&ins);

	// simulate a 120 deg counterclockwise roll around x axis
	input_gyro(&ins, 1200, 0, 0, 1000);
    EXPECT_EQ(true, ABS(ins_get_roll_dd(&ins) - 1200) < margin);
    EXPECT_EQ(0, ins_get_pitch_dd(&ins));
    EXPECT_EQ(0, ins_get_yaw_dd(&ins));
	print_ins(&ins);
}

TEST(InsUnitTest, TestEulerAngleCalculation){
	struct instruments ins; 

	config_reset(&config);
	config.data.imu.dcm_kp = 2500;
    config.data.imu.looptime = 2000;
    config.data.imu.small_angle = 25;
    config.data.imu.max_angle_inclination = 500;

	ins_init(&ins, &config.data);

	// simulate calibration
	for(int c = 0; c < 1000; c++){
		input_accel(&ins, 0, 0, 1024);
	}
	EXPECT_EQ(true, ins_is_calibrated(&ins));

    input_accel(&ins, 0, 0, SYSTEM_ACCEL_1G);
    EXPECT_EQ(0, ins_get_pitch_dd(&ins));
    EXPECT_EQ(0, ins_get_yaw_dd(&ins));

    input_accel(&ins, SYSTEM_ACCEL_1G, 0, 0);
	printf("acc: %d %d %d\n", ins_get_acc_x(&ins), ins_get_acc_y(&ins), ins_get_acc_z(&ins));
    EXPECT_EQ(0, ins_get_roll_dd(&ins));
    EXPECT_EQ(-900, ins_get_pitch_dd(&ins));
    EXPECT_EQ(0, ins_get_yaw_dd(&ins));

    input_accel(&ins, ACC_45DEG, 0, ACC_45DEG);
    EXPECT_EQ(0, ins_get_roll_dd(&ins));
    EXPECT_EQ(-450, ins_get_pitch_dd(&ins));
    EXPECT_EQ(0, ins_get_yaw_dd(&ins));

    input_accel(&ins, 0, ACC_45DEG, ACC_45DEG);
    EXPECT_EQ(450, ins_get_roll_dd(&ins));
    EXPECT_EQ(0, ins_get_pitch_dd(&ins));
    EXPECT_EQ(0, ins_get_yaw_dd(&ins));

    input_accel(&ins, 0, SYSTEM_ACCEL_1G, 0);
    EXPECT_EQ(900, ins_get_roll_dd(&ins));
    EXPECT_EQ(0, ins_get_pitch_dd(&ins));
    EXPECT_EQ(0, ins_get_yaw_dd(&ins)); 

    input_accel(&ins, ACC_45DEG, ACC_45DEG, 0);
    EXPECT_EQ(900, ins_get_roll_dd(&ins));
    EXPECT_EQ(-450, ins_get_pitch_dd(&ins));
    EXPECT_EQ(449, ins_get_yaw_dd(&ins)); // << note the approximation error from the euler calculator

	// pointing straight down
    input_accel(&ins, SYSTEM_ACCEL_1G, 0, 0);
    EXPECT_EQ(1, ins_get_roll_dd(&ins)); // << subject to approximation error
    EXPECT_EQ(-900, ins_get_pitch_dd(&ins));
    EXPECT_EQ(0, ins_get_yaw_dd(&ins));

	// and straight up
    input_accel(&ins, SYSTEM_ACCEL_1G, 0, 0);
    EXPECT_EQ(0, ins_get_roll_dd(&ins));
    EXPECT_EQ(-900, ins_get_pitch_dd(&ins));
    EXPECT_EQ(0, ins_get_yaw_dd(&ins));
}

TEST(InsUnitTest, TestMagYawAngleCalculation){
	struct instruments ins;

	config_reset(&config);
	config.data.imu.dcm_kp = 2500;
    config.data.imu.looptime = 2000;
    config.data.imu.small_angle = 25;
    config.data.imu.max_angle_inclination = 500;

	reset_trims();

	ins_init(&ins, &config.data);

	// simulate calibration
	int16_t zx = 124, zy = -150, zz = 34;
	ins_start_mag_calibration(&ins);
	for(int c = 0; c < 1000; c++){
		ins_process_gyro(&ins, 0, 0, 0);
		ins_process_mag(&ins, 
			zx + (rand() % 2048 - 1024),
			zy + (rand() % 2048 - 1024),
			zz + (rand() % 2048 - 1024)
		);
		ins_update(&ins, 0.05);
	}

	EXPECT_TRUE(ins_is_calibrated(&ins));

	ins_mag_save_trims(&ins.mag, &config.data);

	int thr = 10; // give a little error room because we use random numbers (less error is achieved if calibration cycles count is increased in compass.c)
	// mag biases should be the zero points
	printf("mag bias: %d %d %d\n", config.data.sensors.trims.magZero.raw[0], config.data.sensors.trims.magZero.raw[1], config.data.sensors.trims.magZero.raw[2]);
	printf("mag min: %d %d %d\n", ins.mag.mag_min[0], ins.mag.mag_min[1], ins.mag.mag_min[2]);
	printf("mag max: %d %d %d\n", ins.mag.mag_max[0], ins.mag.mag_max[1], ins.mag.mag_max[2]);

	EXPECT_EQ(true, ABS(config.data.sensors.trims.magZero.raw[0] - zx) < thr);
	EXPECT_EQ(true, ABS(config.data.sensors.trims.magZero.raw[1] - zy) < thr);
	EXPECT_EQ(true, ABS(config.data.sensors.trims.magZero.raw[2] - zz) < thr);

	EXPECT_EQ(true, ins_is_calibrated(&ins));

    input_mag(&ins, zx + 707, zy + 707, zz + 1024);
    EXPECT_EQ(0, ins_get_roll_dd(&ins));
    EXPECT_EQ(0, ins_get_pitch_dd(&ins));
    EXPECT_EQ(true, ABS(ins_get_yaw_dd(&ins) - 450) < thr);

	input_mag(&ins, zx -707, zy + 707, zz + 1024);
    EXPECT_EQ(0, ins_get_roll_dd(&ins));
    EXPECT_EQ(0, ins_get_pitch_dd(&ins));
    EXPECT_EQ(true, ABS(ins_get_yaw_dd(&ins) - 1350) < thr);

	input_mag(&ins, zx -707, zy -707, zz + 1024);
    EXPECT_EQ(0, ins_get_roll_dd(&ins));
    EXPECT_EQ(0, ins_get_pitch_dd(&ins));
    EXPECT_EQ(true, ABS(ins_get_yaw_dd(&ins) - 2250) < thr);

	input_mag(&ins, zx + 707, zy -707, zz -1024);
    EXPECT_EQ(0, ins_get_roll_dd(&ins));
    EXPECT_EQ(0, ins_get_pitch_dd(&ins));
    EXPECT_EQ(true, ABS(ins_get_yaw_dd(&ins) - 3150) < thr);

	config.data.sensors.trims.magZero.raw[0] = 0;
	config.data.sensors.trims.magZero.raw[1] = 0;
	config.data.sensors.trims.magZero.raw[2] = 0;
}

