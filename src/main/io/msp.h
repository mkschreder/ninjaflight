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

extern bool isRebootScheduled;
extern bool mspEnterEsc4way;

struct ninja;
void mspInit(struct ninja *ninja);

int mspProcess(mspPacket_t *command, mspPacket_t *reply);
uint8_t pgMatcherForMSPSet(const pgRegistry_t *candidate, const void *criteria); 
uint8_t pgMatcherForMSP(const pgRegistry_t *candidate, const void *criteria); 
