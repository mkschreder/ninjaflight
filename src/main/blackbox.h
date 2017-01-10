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

#include "config/blackbox.h"
#include "system_calls.h"
#include "common/packer.h"
#include "common/axis.h"
#include "common/pt.h"

struct blackbox_frame_header {
	uint8_t flags;
	uint8_t data[];
};

struct blackbox_frame {
	struct blackbox_frame_header header;

	int32_t time;

	int16_t gyr[3];
	int16_t acc[3];
	int16_t mag[3];
	int16_t roll, pitch, yaw;
	int16_t motors[8];
	int16_t servos[8];

	int16_t command[4];

	uint16_t vbat;
	uint16_t current;

	int32_t altitude;
	int32_t sonar_alt;
	uint16_t rssi;
} __attribute__((__packed__));

struct blackbox {
	uint8_t *cur_frame, *prev_frame;
	uint8_t buffers[2][sizeof(struct blackbox_frame)];
	uint8_t delta_buffer[sizeof(struct blackbox_frame) + sizeof(struct blackbox_frame) / 2];

	QueueHandle_t state_queue;
	const struct config *config;
	const struct system_calls *system;
};

void blackbox_init(struct blackbox *self, const struct config * config, const struct system_calls *system);
void blackbox_start(struct blackbox *self);
void blackbox_write(struct blackbox *self, const struct blackbox_frame *frame);
void blackbox_flush(struct blackbox *self);
int blackbox_parse(const void *data, size_t size, struct blackbox_frame *out, size_t *consumed);
