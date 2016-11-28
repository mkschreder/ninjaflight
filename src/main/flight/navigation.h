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

#include "../config/navigation.h"

// navigation mode
typedef enum {
    NAV_MODE_NONE = 0,
    NAV_MODE_POSHOLD,
    NAV_MODE_WP
} navigationMode_e;

// FIXME ap_mode is badly named, it's a value that is compared to rcCommand, not a flag at it's name implies.
extern int16_t GPS_angle[ANGLE_INDEX_COUNT];                // it's the angles that must be applied for GPS correction

extern int32_t GPS_home[2];
extern int32_t GPS_hold[2];

extern uint16_t GPS_distanceToHome;        // distance to home point in meters
extern int16_t GPS_directionToHome;        // direction to home or hol point in degrees

extern navigationMode_e navi_mode;          // Navigation mode

void navigationInit(struct pid_config *pidProfile);
void GPS_reset_home_position(void);
void GPS_reset_nav(void);
void GPS_set_next_wp(int32_t* lat, int32_t* lon);
void gpsUsePIDs(struct pid_config *pidProfile);
void updateGpsStateForHomeAndHoldMode(void);
void updateGpsWaypointsAndMode(void);

void onGpsNewData(void);
