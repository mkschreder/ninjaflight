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

extern uint16_t mock_motor_pwm[8];
extern uint16_t mock_servo_pwm[8];
extern uint16_t mock_rc_pwm[RX_MAX_SUPPORTED_RC_CHANNELS];
extern uint16_t mock_pwm_errors;
extern int16_t mock_acc[3];
extern int16_t mock_gyro[3];

struct system_calls;
const struct system_calls *mock_syscalls();
void mock_system_reset();

#define UNUSED(x) (void)(x)
