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

#include "common/streambuf.h"

typedef struct mspPacket_s {
    sbuf_t buf;
    int16_t cmd;
    int16_t result;
} mspPacket_t;

struct msp {
	//! mask of enabled IDs, calculated on start based on enabled features. boxId_e is used as bit index.
	uint32_t activeBoxIds;

	//! cause reboot after MSP processing complete
	bool isRebootScheduled;
	//! switch to 4wayIf (on current port)
	#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
	bool mspEnterEsc4way;
	#endif

	struct ninja *ninja;
	struct config *config;
};

struct ninja;

void msp_init(struct msp *self, struct ninja *ninja, struct config *config);
int msp_process(struct msp *self, mspPacket_t *command, mspPacket_t *reply);
