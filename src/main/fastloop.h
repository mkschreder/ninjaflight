/*
 * This file is part of Ninjaflight.
 *
 * Copyright 2016-2017, Martin Schröder <mkschreder.uk@gmail.com>
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

#include "flight/anglerate.h"
#include "flight/mixer.h"
#include "sensors/instruments.h"
#include "system_calls.h"

#include <FreeRTOS.h>
#include <queue.h>

struct fastloop_input {
	int8_t level_pc[2];
	int16_t roll, pitch, yaw, throttle;
	int16_t tilt[2];
	int16_t rc[8];
};

struct fastloop_output {
	int32_t loop_time;
	int16_t w[3]; // omega in deci-deg per sec
	int16_t gyr[3];
	int16_t acc[3];
	int16_t mag[3];
	int16_t roll, pitch, yaw;
};

struct fastloop {
	struct instruments ins;
	struct mixer mixer;
	struct anglerate ctrl;

	sys_micros_t next_acc_read_time;

	QueueHandle_t in_queue, out_queue;

	const struct config *config;
	const struct system_calls *system;
};

void fastloop_write_controls(struct fastloop *self, const struct fastloop_input *in);
void fastloop_read_outputs(struct fastloop *self, struct fastloop_output *out);
void fastloop_init(struct fastloop *self, const struct system_calls *system, const struct config *config);
void fastloop_start(struct fastloop *self);
