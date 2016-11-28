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

#include "blackbox/blackbox_fielddefs.h"
#include "config/blackbox.h"

struct blackbox {
	struct ninja *ninja;
};

void blackbox_init(struct blackbox *self, struct ninja *owner);

void blackboxLogEvent(FlightLogEvent event, flightLogEventData_t *data);

void blackbox_update(struct blackbox *self);
void blackbox_start(struct blackbox *self);
void blackbox_stop(struct blackbox *self);

bool blackboxMayEditConfig(void);
