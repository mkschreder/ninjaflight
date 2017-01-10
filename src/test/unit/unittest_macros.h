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

/**
 * @defgroup SPEC SPEC
 * @page SPEC
 * @ingroup SPEC
 *
 * This is automatically generated specification for all unit tested
 * components. It consists of documentation for each unit test based on
 * requirement that that unit test satisfies.
 */

#include "rx/rx.h"

#define MOCK_EEPROM_MAX_SIZE 4096
#define MOCK_LOGGER_BUF_SIZE (1024 * 2000)

extern uint16_t mock_motor_pwm[8];
extern uint16_t mock_servo_pwm[8];
extern uint16_t mock_rc_pwm[RX_MAX_SUPPORTED_RC_CHANNELS];
extern uint16_t mock_pwm_errors;
extern int16_t mock_acc[3];
extern int16_t mock_gyro[3];
extern uint32_t mock_eeprom_written;
extern uint16_t mock_eeprom_pages;
extern uint16_t mock_eeprom_page_size;
extern uint8_t mock_eeprom_erase_byte;
extern char mock_eeprom_data[];
extern int32_t mock_time_micros;
extern bool mock_beeper_is_on;

extern char mock_logger_data[MOCK_LOGGER_BUF_SIZE];
extern uint16_t mock_logger_pos;

struct system_calls;
struct system_calls *mock_syscalls();
void mock_system_reset();
void mock_eeprom_erase();

typedef enum {
	MOCK_CLOCK_REALTIME,
	MOCK_CLOCK_MANUAL
} mock_clock_mode_t;

void mock_system_clock_mode(mock_clock_mode_t mode);

#define UNUSED(x) (void)(x)
