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

#include "../config/gps.h"
#include "../system_calls.h"
#include "../drivers/serial.h"

#define LAT 0
#define LON 1

#define GPS_DEGREES_DIVIDER 10000000L
#define SBAS_MODE_MAX SBAS_GAGAN

typedef enum {
    GPS_BAUDRATE_115200 = 0,
    GPS_BAUDRATE_57600,
    GPS_BAUDRATE_38400,
    GPS_BAUDRATE_19200,
    GPS_BAUDRATE_9600
} gpsBaudRate_e;

#define GPS_BAUDRATE_MAX GPS_BAUDRATE_9600
typedef struct gpsCoordinateDDDMMmmmm_s {
    int16_t dddmm;
    int16_t mmmm;
} gpsCoordinateDDDMMmmmm_t;


typedef enum {
    GPS_MESSAGE_STATE_IDLE = 0,
    GPS_MESSAGE_STATE_INIT,
    GPS_MESSAGE_STATE_SBAS,
    GPS_MESSAGE_STATE_MAX = GPS_MESSAGE_STATE_SBAS
} gpsMessageState_e;

#define GPS_MESSAGE_STATE_ENTRY_COUNT (GPS_MESSAGE_STATE_MAX + 1)

typedef struct gpsData_s {
    uint8_t state;                  // GPS thread state. Used for detecting cable disconnects and configuring attached devices
    uint8_t baudrateIndex;          // index into auto-detecting or current baudrate
    uint32_t errors;                // gps error counter - crc error/lost of data/sync etc..
    uint32_t timeouts;
    uint32_t lastMessage;           // last time valid GPS data was received (millis)
    uint32_t lastLastMessage;       // last-last valid GPS message. Used to calculate delta.

    uint32_t state_position;        // incremental variable for loops
    sys_millis_t state_ts;              // timestamp for last state_position increment
    gpsMessageState_e messageState;
} gpsData_t;

#define GPS_PACKET_LOG_ENTRY_COUNT 21 // To make this useful we should log as many packets as we can fit characters a single line of a OLED display.

struct gps {
	char gpsPacketLog[GPS_PACKET_LOG_ENTRY_COUNT];
	char *gpsPacketLogChar;

	gpsData_t gpsData;
	int32_t GPS_coord[2];               // LAT/LON

	uint8_t GPS_numSat;
	uint16_t GPS_hdop;                  // GPS signal quality
	uint8_t GPS_update;                 // it's a binary toogle to distinct a GPS position update
	uint32_t GPS_packetCount;
	uint32_t GPS_svInfoReceivedCount;
	uint16_t GPS_altitude;              // altitude in 0.1m
	uint16_t GPS_speed;                 // speed in 0.1m/s
	uint16_t GPS_ground_course;         // degrees * 10
	uint8_t GPS_numCh;                  // Number of channels
	uint8_t GPS_svinfo_chn[16];         // Channel number
	uint8_t GPS_svinfo_svid[16];        // Satellite ID
	uint8_t GPS_svinfo_quality[16];     // Bitfield Qualtity
	uint8_t GPS_svinfo_cno[16];         // Carrier to Noise Ratio (Signal Strength)

	uint32_t GPS_garbageByteCount;

	serialPort_t *gpsPort;
	const struct system_calls *system;
	const struct config *config;
};

#define GPS_DBHZ_MIN 0
#define GPS_DBHZ_MAX 55

void gps_init(struct gps *self, const struct system_calls *system, const struct config *config);

void gps_enable_passthrough(struct gps *self, struct serialPort_s *gpsPassthroughPort); 
void gps_update(struct gps *self);
bool gps_process_char(struct gps *self, uint8_t c);
