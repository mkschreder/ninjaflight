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

//! serial port identifiers
typedef enum {
    SERIAL_PORT_NONE = -1,
    SERIAL_PORT_UART1 = 0,
    SERIAL_PORT_UART2,
    SERIAL_PORT_UART3,
    SERIAL_PORT_UART4,
    SERIAL_PORT_UART5,
    SERIAL_PORT_USB_VCP = 20,
    SERIAL_PORT_SOFTSERIAL1 = 30,
    SERIAL_PORT_SOFTSERIAL2,
    SERIAL_PORT_IDENTIFIER_MAX = SERIAL_PORT_SOFTSERIAL2
} serialPortIdentifier_e;

typedef enum {
    PORTSHARING_UNUSED = 0,
    PORTSHARING_NOT_SHARED,
    PORTSHARING_SHARED
} portSharing_e;

typedef enum {
    FUNCTION_NONE                = 0,
    FUNCTION_MSP                 = (1 << 0), // 1
    FUNCTION_GPS                 = (1 << 1), // 2
    FUNCTION_TELEMETRY_FRSKY     = (1 << 2), // 4
    FUNCTION_TELEMETRY_HOTT      = (1 << 3), // 8
    FUNCTION_TELEMETRY_LTM       = (1 << 4), // 16
    FUNCTION_TELEMETRY_SMARTPORT = (1 << 5), // 32
    FUNCTION_RX_SERIAL           = (1 << 6), // 64
    FUNCTION_BLACKBOX            = (1 << 7), // 128
    FUNCTION_TELEMETRY_MAVLINK   = (1 << 8)  // 256
} serialPortFunction_e;

typedef enum {
    BAUD_AUTO = 0,
    BAUD_9600,
    BAUD_19200,
    BAUD_38400,
    BAUD_57600,
    BAUD_115200,
    BAUD_230400,
    BAUD_250000,
} baudRate_e;


typedef struct serialPortConfig_s {
    serialPortIdentifier_e identifier;
    uint16_t functionMask;
    uint8_t msp_baudrateIndex;
    uint8_t gps_baudrateIndex;
    uint8_t blackbox_baudrateIndex;
    uint8_t telemetry_baudrateIndex; // not used for all telemetry systems, e.g. HoTT only works at 19200.
} serialPortConfig_t;

typedef struct serialConfig_s {
    uint8_t reboot_character;               // which byte is used to reboot. Default 'R', could be changed carefully to something else.
    serialPortConfig_t portConfigs[SERIAL_PORT_COUNT];
} serialConfig_t;

PG_DECLARE(serialConfig_t, serialConfig);

