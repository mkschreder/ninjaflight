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

extern "C" {
    #include "platform.h"
    #include "drivers/system.h"
	#include "ninja.h"
}

#include <stdio.h>
#include <time.h>

uint16_t mock_motor_pwm[8];
uint16_t mock_servo_pwm[8];
uint16_t mock_pwm_errors = 0;
int16_t mock_acc[3];
int16_t mock_gyro[3];

void _write_motor(const struct system_calls_pwm *pwm, uint8_t id, uint16_t value){
	(void)pwm;
	if(value < 1000 || value > 2000) {
		mock_pwm_errors++;
		printf("write motor out of range: %d %d\n", id, value);
	}
	mock_motor_pwm[id] = value;
}

void _write_servo(const struct system_calls_pwm *pwm, uint8_t id, uint16_t value){
	(void)pwm;
	if(value < 1000 || value > 2000) {
		printf("write servo out of range: %d %d\n", id, value);
		mock_pwm_errors++;
	}
	mock_servo_pwm[id] = value;
}

#include <string.h>
static int32_t _micros(const struct system_calls_time *time){
	(void)time;
	struct timespec ts;
	static struct timespec start_ts = {0, 0};
	clock_gettime(CLOCK_MONOTONIC, &ts);
	if(start_ts.tv_sec == 0) memcpy(&start_ts, &ts, sizeof(start_ts));
	int32_t t = (ts.tv_sec - start_ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;
	//printf("read time: %d\n", t);
	return t;
}

static int _read_gyro(const struct system_calls_imu *imu, int16_t output[3]){
	(void)imu;
	(void)output;
	//printf("read gyro\n");
	memcpy(output, mock_gyro, sizeof(mock_gyro));
	return 0;
}

static int _read_acc(const struct system_calls_imu *imu, int16_t output[3]){
	(void)imu;
	(void)output;
	//printf("read acc\n");
	memcpy(output, mock_acc, sizeof(mock_gyro));
	return 0;
}

static void _led_on(const struct system_calls_leds *leds, uint8_t id, bool on){
	(void)leds;
	printf("led %d %s\n", id, (on)?"on":"off");
	fflush(stdout);
}

static void _led_toggle(const struct system_calls_leds *leds, uint8_t id){
	(void)leds;
	printf("toggle led %d\n", id);
	fflush(stdout);
}

static const struct system_calls syscalls {
	.pwm = {
		.write_motor = _write_motor,
		.write_servo = _write_servo
	},
	.imu = {
		.read_gyro = _read_gyro,
		.read_acc = _read_acc
	},
	.leds = {
		.on = _led_on,
		.toggle = _led_toggle
	},
	.time = {
		.micros = _micros
	}
};

const struct system_calls *mock_syscalls(){
	return &syscalls;
}

#include <string.h>

void mock_system_reset(){
	memset(mock_motor_pwm, 0, sizeof(mock_motor_pwm));
	memset(mock_servo_pwm, 0, sizeof(mock_servo_pwm));
	mock_pwm_errors = 0;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"
/*
void failureMode(uint8_t mode)
{
    UNUSED(mode);
    EXPECT_TRUE(false);
}
*/
uint16_t cycleTime;
