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
#include "system_calls.h"

typedef enum BlackboxState {
	BLACKBOX_STATE_DISABLED = 0,
	BLACKBOX_STATE_STOPPED,
	BLACKBOX_STATE_PREPARE_LOG_FILE,
	BLACKBOX_STATE_SEND_HEADER,
	BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER,
	BLACKBOX_STATE_SEND_GPS_H_HEADER,
	BLACKBOX_STATE_SEND_GPS_G_HEADER,
	BLACKBOX_STATE_SEND_SLOW_HEADER,
	BLACKBOX_STATE_SEND_SYSINFO,
	BLACKBOX_STATE_PAUSED,
	BLACKBOX_STATE_RUNNING,
	BLACKBOX_STATE_SHUTTING_DOWN
} BlackboxState;

#define BLACKBOX_FIRST_HEADER_SENDING_STATE BLACKBOX_STATE_SEND_HEADER
#define BLACKBOX_LAST_HEADER_SENDING_STATE BLACKBOX_STATE_SEND_SYSINFO

typedef struct blackboxMainState_s {
	uint32_t time;

	int32_t axisPID_P[XYZ_AXIS_COUNT], axisPID_I[XYZ_AXIS_COUNT], axisPID_D[XYZ_AXIS_COUNT];

	int16_t rcCommand[4];
	int16_t gyroADC[XYZ_AXIS_COUNT];
	int16_t accSmooth[XYZ_AXIS_COUNT];

	uint16_t vbatLatest;
	uint16_t amperageLatest;

#ifdef BARO
	int32_t BaroAlt;
#endif
	int16_t magADC[XYZ_AXIS_COUNT];
#ifdef SONAR
	int32_t sonarRaw;
#endif
	uint16_t rssi;
} blackboxMainState_t;

typedef struct blackboxGpsState_s {
	int32_t GPS_home[2], GPS_coord[2];
	uint8_t GPS_numSat;
} blackboxGpsState_t;

// This data is updated really infrequently:
typedef struct blackboxSlowState_s {
	uint16_t flightModeFlags;
	uint8_t stateFlags;
	uint8_t failsafePhase;
	bool rxSignalReceived;
	bool rxFlightChannelsValid;
} __attribute__((__packed__)) blackboxSlowState_t; // We pack this struct so that padding doesn't interfere with memcmp()


struct blackbox_xmit_state {
	uint32_t headerIndex;

	/* Since these fields are used during different blackbox states (never simultaneously) we can
	 * overlap them to save on RAM
	 */
	union {
		int fieldIndex;
		sys_millis_t startTime;
	} u;
};

struct blackbox {
	struct ninja *ninja;
	
	uint32_t blackboxIteration;
	uint16_t blackboxPFrameIndex, blackboxIFrameIndex;
	uint16_t blackboxSlowFrameIterationTimer;
	bool blackboxLoggedAnyFrames;
	struct blackbox_xmit_state xmitState;

	BlackboxState blackboxState;
	/*
	 * We store voltages in I-frames relative to this, which was the voltage when the blackbox was activated.
	 * This helps out since the voltage is only expected to fall from that point and we can reduce our diffs
	 * to encode:
	 */
	uint16_t vbatReference;

	blackboxGpsState_t gpsHistory;
	blackboxSlowState_t slowHistory;

	// Keep a history of length 2, plus a buffer for MW to store the new values into
	blackboxMainState_t blackboxHistoryRing[3];

	// These point into blackboxHistoryRing, use them to know where to store history of a given age (0, 1 or 2 generations old)
	blackboxMainState_t* blackboxHistory[3];

	bool blackboxModeActivationConditionPresent;

	// Cache for FLIGHT_LOG_FIELD_CONDITION_* test results:
	uint32_t blackboxConditionCache;

	const struct config *config;
};

void blackbox_init(struct blackbox *self, struct ninja *owner, const struct config * config);

void blackboxLogEvent(struct blackbox *self, FlightLogEvent event, flightLogEventData_t *data);

void blackbox_update(struct blackbox *self);
void blackbox_start(struct blackbox *self);
void blackbox_stop(struct blackbox *self);

bool blackbox_is_running(struct blackbox *self);
