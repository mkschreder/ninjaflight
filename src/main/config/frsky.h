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

typedef enum {
    FRSKY_FORMAT_DMS = 0,
    FRSKY_FORMAT_NMEA
} frskyGpsCoordFormat_e;

typedef enum {
    FRSKY_UNIT_METRICS = 0,
    FRSKY_UNIT_IMPERIALS
} frskyUnit_e;

typedef struct frskyTelemetryConfig_s {
    float gpsNoFixLatitude;
    float gpsNoFixLongitude;
    frskyGpsCoordFormat_e frsky_coordinate_format;
    frskyUnit_e frsky_unit;
    uint8_t frsky_vfas_precision;
} frskyTelemetryConfig_t;

PG_DECLARE(frskyTelemetryConfig_t, frskyTelemetryConfig);
