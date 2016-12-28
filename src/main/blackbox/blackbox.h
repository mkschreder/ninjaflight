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

	int16_t gyro[XYZ_AXIS_COUNT];
	int16_t acc[XYZ_AXIS_COUNT];

	int16_t command[4];

	uint16_t vbat;
	uint16_t current;

	int32_t altitude;
	int16_t mag[XYZ_AXIS_COUNT];
	int32_t sonar_alt;
	uint16_t rssi;

	uint32_t motor[8];
	uint32_t servo[8];
} __attribute__((__packed__));

struct blackbox_slow_frame {
	struct blackbox_frame_header header;

	uint16_t flightModeFlags;
	uint8_t stateFlags;
	uint8_t failsafePhase;
	bool rxSignalReceived;
	bool rxFlightChannelsValid;
} __attribute__((__packed__));

struct blackbox {
	uint32_t iteration;
	uint8_t *cur_frame, *prev_frame;
	uint8_t buffers[2][sizeof(struct blackbox_frame)];
	uint8_t delta_buffer[sizeof(struct blackbox_frame) + sizeof(struct blackbox_frame) / 2];
	int16_t delta_size;
	uint8_t *delta_ptr;
	struct pt flash_writer;
	//struct packer packer;

	const struct config *config;
	const struct system_calls *system;
};

void blackbox_init(struct blackbox *self, const struct config * config, const struct system_calls *system);

void blackbox_write_frame(struct blackbox *self, const struct blackbox_frame *frame);

void blackbox_update(struct blackbox *self);
void blackbox_start(struct blackbox *self);
void blackbox_stop(struct blackbox *self);

bool blackbox_is_running(struct blackbox *self);
