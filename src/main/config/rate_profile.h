/*
 * This file is part of Ninjaflight.
 *
 * Copyright: collective.
 * Cleanup: Martin Schröder <mkschreder.uk@gmail.com>
 * Original source from Cleanflight.
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

#define MAX_CONTROL_RATE_PROFILE_COUNT 3

struct rate_profile {
    uint8_t rcRate8;
    uint8_t rcExpo8;
    uint8_t thrMid8;
    uint8_t thrExpo8;
    uint8_t rates[3];
    uint8_t dynThrPID;
    uint8_t rcYawExpo8;
    uint16_t tpa_breakpoint;                // Breakpoint at which TPA is activated
};

struct rate_profile_selection {
    uint8_t profile_id;
};

struct rate_config {
	struct rate_profile profile[MAX_CONTROL_RATE_PROFILE_COUNT];
};

