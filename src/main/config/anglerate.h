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

#define PID_MAX_I 256
#define PID_MAX_D 512
#define PID_MAX_TOTAL_PID 1000

#define GYRO_I_MAX 256                      // Gyro I limiter
#define YAW_P_LIMIT_MIN 100                 // Maximum value for yaw P limiter
#define YAW_P_LIMIT_MAX 500                 // Maximum value for yaw P limiter

#define DTERM_AVERAGE_COUNT 4

typedef enum {
    PIDROLL,
    PIDPITCH,
    PIDYAW,
    PIDALT,
    PIDPOS,
    PIDPOSR,
    PIDNAVR,
    PIDLEVEL,
    PIDMAG,
    PIDVEL,
    PID_ITEM_COUNT
} pid_index_t;

typedef enum {
	PID_CONTROLLER_MW23 = 0,
    PID_CONTROLLER_MWREWRITE,
    PID_CONTROLLER_LUX_FLOAT,
    PID_COUNT
} pid_controller_type_t;

struct pid_config {
    uint8_t P8[PID_ITEM_COUNT];
    uint8_t I8[PID_ITEM_COUNT];
    uint8_t D8[PID_ITEM_COUNT];
    uint8_t pidController;
    uint16_t yaw_p_limit;                   // set P term limit (fixed value was 300)
    uint16_t dterm_cut_hz;                  // dterm filtering
};

