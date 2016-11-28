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

#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"

#include <platform.h>
#include "build_config.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/parameter_group.h"
#include "config/feature.h"

#include "io/rc_controls.h"

//#include "../drivers/sound_beeper.h"
#include "../drivers/system.h"
#include "../sensors/battery.h"

#include "io/statusindicator.h"

#ifdef GPS
#include "io/gps.h"
#endif

#include "io/beeper.h"

#define BEEPER_COMMAND_REPEAT 0xFE
#define BEEPER_COMMAND_STOP   0xFF

/* Beeper Sound Sequences: (Square wave generation)
 * Sequence must end with 0xFF or 0xFE. 0xFE repeats the sequence from
 * start when 0xFF stops the sound when it's completed.
 *
 * "Sound" Sequences are made so that 1st, 3rd, 5th.. are the delays how
 * long the beeper is on and 2nd, 4th, 6th.. are the delays how long beeper
 * is off. Delays are in milliseconds/10 (i.e., 5 => 50ms).
 */
// short fast beep
static const uint8_t beep_shortBeep[] = {
	10, 10, BEEPER_COMMAND_STOP
};
// arming beep
static const uint8_t beep_armingBeep[] = {
	30, 5, 5, 5, BEEPER_COMMAND_STOP
};
// armed beep (first pause, then short beep)
static const uint8_t beep_armedBeep[] = {
	0, 245, 10, 5, BEEPER_COMMAND_STOP
};
// disarming beeps
static const uint8_t beep_disarmBeep[] = {
	15, 5, 15, 5, BEEPER_COMMAND_STOP
};
// beeps while stick held in disarm position (after pause)
static const uint8_t beep_disarmRepeatBeep[] = {
	0, 100, 10, BEEPER_COMMAND_STOP
};
// Long beep and pause after that
static const uint8_t beep_lowBatteryBeep[] = {
	25, 50, BEEPER_COMMAND_STOP
};
// critical battery beep
static const uint8_t beep_critBatteryBeep[] = {
	50, 2, BEEPER_COMMAND_STOP
};

// transmitter-signal-lost tone
static const uint8_t beep_txLostBeep[] = {
	50, 50, BEEPER_COMMAND_STOP
};
// SOS morse code:
static const uint8_t beep_sos[] = {
	10, 10, 10, 10, 10, 40, 40, 10, 40, 10, 40, 40, 10, 10, 10, 10, 10, 70, BEEPER_COMMAND_STOP
};
// Arming when GPS is fixed
static const uint8_t beep_armedGpsFix[] = {
	5, 5, 15, 5, 5, 5, 15, 30, BEEPER_COMMAND_STOP
};
// Ready beeps. When gps has fix and copter is ready to fly.
static const uint8_t beep_readyBeep[] = {
	4, 5, 4, 5, 8, 5, 15, 5, 8, 5, 4, 5, 4, 5, BEEPER_COMMAND_STOP
};
// 2 fast short beeps
static const uint8_t beep_2shortBeeps[] = {
	5, 5, 5, 5, BEEPER_COMMAND_STOP
};
// 2 longer beeps
static const uint8_t beep_2longerBeeps[] = {
	20, 15, 35, 5, BEEPER_COMMAND_STOP
};
// 3 beeps
static const uint8_t beep_gyroCalibrated[] = {
	20, 10, 20, 10, 20, 10, BEEPER_COMMAND_STOP
};

#define BEEPER_CONFIRMATION_BEEP_DURATION 2
#define BEEPER_CONFIRMATION_BEEP_GAP_DURATION 20

#ifdef BEEPER_NAMES
#define BEEPER_ENTRY(a,b,c,d) a,b,c,d
#else
#define BEEPER_ENTRY(a,b,c,d) a,b,c
#endif

#define BEEPER_MAX_MULTI_BEEPS 20   //size limit for 'beep_multiBeeps[]'

//! Array used for variable # of beeps (reporting GPS sat count, etc)
uint8_t beep_multiBeeps[BEEPER_MAX_MULTI_BEEPS + 2];

static const beeperTableEntry_t beeperTable[] = {
	{ BEEPER_ENTRY(BEEPER_GYRO_CALIBRATED,	   0, beep_gyroCalibrated,   "GYRO_CALIBRATED") },
	{ BEEPER_ENTRY(BEEPER_RX_LOST_LANDING,	   1, beep_sos,			  "RX_LOST_LANDING") },
	{ BEEPER_ENTRY(BEEPER_RX_LOST,			   2, beep_txLostBeep,	   "RX_LOST") },
	{ BEEPER_ENTRY(BEEPER_DISARMING,			 3, beep_disarmBeep,	   "DISARMING") },
	{ BEEPER_ENTRY(BEEPER_ARMING,				4, beep_armingBeep,	   "ARMING")  },
	{ BEEPER_ENTRY(BEEPER_ARMING_GPS_FIX,		5, beep_armedGpsFix,	  "ARMING_GPS_FIX") },
	{ BEEPER_ENTRY(BEEPER_BAT_CRIT_LOW,		  6, beep_critBatteryBeep,  "BAT_CRIT_LOW") },
	{ BEEPER_ENTRY(BEEPER_BAT_LOW,			   7, beep_lowBatteryBeep,   "BAT_LOW") },
	{ BEEPER_ENTRY(BEEPER_GPS_STATUS,			8, beep_multiBeeps,	   NULL) },
	{ BEEPER_ENTRY(BEEPER_RX_SET,				9, beep_shortBeep,		"RX_SET") },
	{ BEEPER_ENTRY(BEEPER_ACC_CALIBRATION,	   10, beep_2shortBeeps,	 "ACC_CALIBRATION") },
	{ BEEPER_ENTRY(BEEPER_ACC_CALIBRATION_FAIL,  11, beep_2longerBeeps,	"ACC_CALIBRATION_FAIL") },
	{ BEEPER_ENTRY(BEEPER_READY_BEEP,			12, beep_readyBeep,	   "READY_BEEP") },
	{ BEEPER_ENTRY(BEEPER_MULTI_BEEPS,		   13, beep_multiBeeps,	  NULL) }, // FIXME having this listed makes no sense since the beep array will not be initialised.
	{ BEEPER_ENTRY(BEEPER_DISARM_REPEAT,		 14, beep_disarmRepeatBeep, "DISARM_REPEAT") },
	{ BEEPER_ENTRY(BEEPER_ARMED,				 15, beep_armedBeep,	   "ARMED") },
};

#define BEEPER_TABLE_ENTRY_COUNT (sizeof(beeperTable) / sizeof(beeperTableEntry_t))

/*
 * Called to activate/deactivate beeper, using the given "BEEPER_..." value.
 * This function returns immediately (does not block).
 */
void beeper_start(struct beeper *self, beeper_command_t mode)
{
	if (mode == BEEPER_SILENCE) {
		beeper_stop(self);
		return;
	}

	const beeperTableEntry_t *selectedCandidate = NULL;
	for (uint32_t i = 0; i < BEEPER_TABLE_ENTRY_COUNT; i++) {
		const beeperTableEntry_t *candidate = &beeperTable[i];
		if (candidate->mode != mode) {
			continue;
		}

		if (!self->currentBeeperEntry) {
			selectedCandidate = candidate;
			break;
		}

		if (candidate->priority < self->currentBeeperEntry->priority) {
			selectedCandidate = candidate;
		}

		break;
	}

	if (!selectedCandidate) {
		return;
	}

	self->currentBeeperEntry = selectedCandidate;

	self->beeperPos = 0;
	self->beeperNextToggleTime = 0;
}

void beeper_stop(struct beeper *self){
	sys_beeper_off(self->system);

	self->beeperIsOn = 0;

	self->beeperNextToggleTime = 0;
	self->beeperPos = 0;

	self->currentBeeperEntry = NULL;
}

/**
 * Emits the given number of 20ms beeps (with 200ms spacing).
 * This function returns immediately (does not block).
 */
void beeper_multi_beeps(struct beeper *self, uint8_t beepCount){
	int i;
	int cLimit;

	i = 0;
	cLimit = beepCount * 2;
	if(cLimit > BEEPER_MAX_MULTI_BEEPS)
		cLimit = BEEPER_MAX_MULTI_BEEPS;  //stay within array size
	do {
		beep_multiBeeps[i++] = BEEPER_CONFIRMATION_BEEP_DURATION;	   // 20ms beep
		beep_multiBeeps[i++] = BEEPER_CONFIRMATION_BEEP_GAP_DURATION;   // 200ms pause
	} while (i < cLimit);
	beep_multiBeeps[i] = BEEPER_COMMAND_STOP;	 //sequence end
	beeper_start(self, BEEPER_MULTI_BEEPS);	//initiate sequence
}

#if 0
// TODO: beeper gps status. Need to do this outside of beeper code
static void beeperGpsStatus(void)
{
	// if GPS fix then beep out number of satellites
	if (STATE(GPS_FIX) && GPS_numSat >= 5) {
		uint8_t i = 0;
		do {
			beep_multiBeeps[i++] = 5;
			beep_multiBeeps[i++] = 10;
		} while (i < BEEPER_MAX_MULTI_BEEPS && GPS_numSat > i / 2);

		beep_multiBeeps[i-1] = 50; // extend last pause
		beep_multiBeeps[i] = BEEPER_COMMAND_STOP;

		beeper_start(self, BEEPER_MULTI_BEEPS);	//initiate sequence
	} else {
		beeper_start(self, BEEPER_RX_SET);
	}
}
#endif

/**
 * Beeper handler function to be called periodically in loop. Updates beeper
 * state via time schedule.
 */
void beeper_update(struct beeper *self){
	// Beeper routine doesn't need to update if there aren't any sounds ongoing
	if (self->currentBeeperEntry == NULL) {
		return;
	}

	uint32_t now = sys_millis(self->system);
	if (self->beeperNextToggleTime > now) {
		return;
	}

	if (!self->beeperIsOn) {
		self->beeperIsOn = 1;
		if (self->currentBeeperEntry->sequence[self->beeperPos] != 0) {
			sys_beeper_on(self->system);
			// if this was arming beep then mark time (for blackbox)
			if (
				self->beeperPos == 0
				&& (self->currentBeeperEntry->mode == BEEPER_ARMING || self->currentBeeperEntry->mode == BEEPER_ARMING_GPS_FIX)
			) {
				self->armingBeepTimeMicros = sys_micros(self->system);
			}
		}
	} else {
		self->beeperIsOn = 0;
		if (self->currentBeeperEntry->sequence[self->beeperPos] != 0) {
			sys_beeper_off(self->system);
		}
	}

	// Calculates array position when next to change beeper state is due.
	if (self->currentBeeperEntry->sequence[self->beeperPos] == BEEPER_COMMAND_REPEAT) {
		self->beeperPos = 0;
	} else if (self->currentBeeperEntry->sequence[self->beeperPos] == BEEPER_COMMAND_STOP) {
		beeper_stop(self);
	} else {
		// Otherwise advance the sequence and calculate next toggle time
		self->beeperNextToggleTime = sys_millis(self->system) + 10 * self->currentBeeperEntry->sequence[self->beeperPos];
		self->beeperPos++;
	}
}

/**
 * Returns the 'beeperMode_e' value for the given beeper-table index,
 * or BEEPER_SILENCE if none.
 */
beeper_command_t beeperModeForTableIndex(int idx){
	return (idx >= 0 && idx < (int)BEEPER_TABLE_ENTRY_COUNT) ? beeperTable[idx].mode : BEEPER_SILENCE;
}

/**
 * Returns the name for the given beeper-table index, or NULL if none.
 */
const char *beeperNameForTableIndex(int idx){
#ifndef BEEPER_NAMES
	UNUSED(idx);
	return NULL;
#else
	return (idx >= 0 && idx < (int)BEEPER_TABLE_ENTRY_COUNT) ? beeperTable[idx].name : NULL;
#endif
}

/**
 * Returns the number of entries in the beeper-sounds table.
 */
int beeperTableEntryCount(void){
	return (int)BEEPER_TABLE_ENTRY_COUNT;
}


