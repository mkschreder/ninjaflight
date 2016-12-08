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
uint16_t mock_rc_pwm[RX_MAX_SUPPORTED_RC_CHANNELS];
uint16_t mock_pwm_errors = 0;
int16_t mock_acc[3];
int16_t mock_gyro[3];
uint32_t mock_eeprom_written = 0;
uint16_t mock_eeprom_pages = 2;
uint16_t mock_eeprom_page_size = 512;
uint8_t mock_eeprom_erase_byte = 0xff;

#define MOCK_EEPROM_MAX_SIZE 4096

char mock_eeprom_data[MOCK_EEPROM_MAX_SIZE];

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

uint16_t _read_pwm(const struct system_calls_pwm *pwm, uint8_t id){
	(void)pwm;
	return mock_rc_pwm[id];
}

uint16_t _read_ppm(const struct system_calls_pwm *pwm, uint8_t id){
	(void)pwm;
	return mock_rc_pwm[id];
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

static void _beeper_on(const struct system_calls_beeper *beeper, bool on){
	(void)beeper;
	printf("beeper %s\n", (on)?"on":"off");
	fflush(stdout);
}

static int _eeprom_read(const struct system_calls_eeprom *self, void *dst, uint16_t addr, size_t size){
	(void)self;
	//printf("EEPROM read from %04x, size %lu\n", addr, size);
	if(addr >= (mock_eeprom_pages * mock_eeprom_page_size)) return -1;
	if((size + addr) >= (uint32_t)(mock_eeprom_pages * mock_eeprom_page_size))
		size = (mock_eeprom_pages * mock_eeprom_page_size) - addr;
	memcpy(dst, mock_eeprom_data + addr, size);
	fflush(stdout);
	return size;
}

static int _eeprom_erase_page(const struct system_calls_eeprom *self, uint16_t addr){
	(void)self;
	// addr should be at page boundary
	addr = (addr / mock_eeprom_page_size) * mock_eeprom_page_size;
	memset(mock_eeprom_data + addr, mock_eeprom_erase_byte, mock_eeprom_page_size);
	usleep(100);
	return 0;
}

static int _eeprom_write(const struct system_calls_eeprom *self, uint16_t addr, const void *data, size_t size){
	(void)self;
	// simulate erase
	if(addr >= (mock_eeprom_pages * mock_eeprom_page_size)) return -1;
	if((size + addr) > (uint32_t)(mock_eeprom_pages * mock_eeprom_page_size))
		size = (mock_eeprom_pages * mock_eeprom_page_size) - addr;
		/*
	printf("EEPROM write to %04x (%d), size %lu: ", addr, addr, size);
	for(size_t c = 0; c < size; c++) printf("%02x ", (int)((char*)data)[c] & 0xff);
	printf("\n");
	*/
	// simulate various eeprom/flash behaviours
	for(size_t c = 0; c < size; c++){
		uint8_t value = ((char*)data)[c];
		if(mock_eeprom_erase_byte == 0x00) // writing ones
			mock_eeprom_data[addr + c] |= value;
		if(mock_eeprom_erase_byte == 0xff) // writing zeros
			mock_eeprom_data[addr + c] &= value;
		else
			mock_eeprom_data[addr + c] = value;
		mock_eeprom_written++;
	}
	fflush(stdout);
	return size;
}

static void _eeprom_get_info(const struct system_calls_eeprom *self, struct system_eeprom_info *info){
	(void)self;
	info->page_size = mock_eeprom_page_size;
	info->num_pages = mock_eeprom_pages;
}

static const struct system_calls syscalls {
	.pwm = {
		.write_motor = _write_motor,
		.write_servo = _write_servo,
		.read_pwm = _read_pwm, 
		.read_ppm = _read_ppm
	},
	.imu = {
		.read_gyro = _read_gyro,
		.read_acc = _read_acc
	},
	.leds = {
		.on = _led_on,
		.toggle = _led_toggle
	},
	.beeper = {
		.on = _beeper_on
	},
	.time = {
		.micros = _micros
	},
	.eeprom = {
		.read = _eeprom_read,
		.write = _eeprom_write,
		.erase_page = _eeprom_erase_page,
		.get_info = _eeprom_get_info
	}
};

const struct system_calls *mock_syscalls(){
	return &syscalls;
}

#include <string.h>

extern "C" {
struct ninja;
void ninja_config_reset(struct ninja *self);
}

void mock_eeprom_erase(){
	memset(mock_eeprom_data, mock_eeprom_erase_byte, sizeof(mock_eeprom_data));
}

void mock_system_reset(){
	//ninja_config_reset(NULL);
	memset(mock_motor_pwm, 0, sizeof(mock_motor_pwm));
	memset(mock_servo_pwm, 0, sizeof(mock_servo_pwm));
	memset(mock_rc_pwm, 0, sizeof(mock_rc_pwm));
	memset(mock_eeprom_data, mock_eeprom_erase_byte, sizeof(mock_eeprom_data));
	mock_pwm_errors = 0;
	mock_eeprom_written = 0;
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
