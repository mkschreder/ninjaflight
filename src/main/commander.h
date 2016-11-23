/*
 * This file is part of Ninjaflight.
 *
 * Copyright 2016 Martin Schröder <mkschreder.uk@gmail.com>
 *
 * Uses code from cleanflight project.
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

#include <stdint.h>

typedef enum {
	COMMANDER_EVENT_NONE,
} commander_event_t;

//! 8 channel commander input. Always in range [-500;500]
struct commander_input {
	int16_t roll, pitch, yaw, throttle, aux1, aux2, aux3, aux4;
};

//! commander module that translates user input into flight controller commands
struct commander {
	int16_t rcCommand[4];
	int16_t lastCommand[4];
    int16_t deltaRC[4];
    int16_t factor;
	int16_t refresh_rate; //!< time in microseconds between rc readings (TODO: make this dynamic)

	void (*event_callback)(struct commander *self, commander_event_t ev, int16_t data);

	uint8_t flags; //!< internal flags
};

//! initializes the commander module
void commander_init(struct commander *self, void (*process_event)(struct commander *self, commander_event_t ev, int16_t data));

//! processes input controls and sends out flight control events using the callback supplied to init
void commander_process_controls(struct commander *self, const struct commander_input *controls);
