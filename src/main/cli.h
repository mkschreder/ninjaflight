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

#include "drivers/serial.h"
#include "common/buf_writer.h"

struct ninja;
struct cli {
	uint8_t cliMode;

	serialPort_t *cliPort;
	bufWriter_t *cliWriter;

	// buffer
	char cliBuffer[48];
	uint32_t bufferIndex;
	uint8_t cliWriteBuffer[48];
	//uint8_t cliWriteBuffer[sizeof(bufWriter_t) + 16];

	struct ninja *ninja;
};

void cli_init(struct cli *self, struct ninja *ninja);
void cli_start(struct cli *self, serialPort_t *serialPort);
void cli_update(struct cli *self);
bool cli_is_active(struct cli *self);
bool cli_uses_port(struct cli *self, serialPort_t *serialPort);
