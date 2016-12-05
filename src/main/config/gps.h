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
    GPS_NMEA = 0,
    GPS_UBLOX,
	GPS_PROVIDER_MAX
} gpsProvider_e;

typedef enum {
    GPS_AUTOCONFIG_OFF = 0,
    GPS_AUTOCONFIG_ON,
} gpsAutoConfig_e;

typedef enum {
    GPS_AUTOBAUD_OFF = 0,
    GPS_AUTOBAUD_ON
} gpsAutoBaud_e;

typedef enum {
    SBAS_AUTO = 0,
    SBAS_EGNOS,
    SBAS_WAAS,
    SBAS_MSAS,
    SBAS_GAGAN
} sbasMode_e;

struct gps_config {
    gpsProvider_e provider;
    sbasMode_e sbasMode;
    gpsAutoConfig_e autoConfig;
    gpsAutoBaud_e autoBaud;
};

