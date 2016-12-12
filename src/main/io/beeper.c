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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

#include <string.h>
#include <platform.h>
#include "build_config.h"

#include "common/pt.h"
#include "utype/cbuf.h"

#include "config/config.h"
#include "config/feature.h"

//#include "../drivers/sound_beeper.h"
#include "../drivers/system.h"
#include "../sensors/battery.h"

#include "io/statusindicator.h"

#include "io/beeper.h"

#define MORSE_DOT_LENGTH	50
#define MORSE_DASH_LENGTH	(3 * MORSE_DOT_LENGTH)
#define MORSE_PART_PAUSE	(MORSE_DOT_LENGTH)
#define MORSE_LETTER_PAUSE	(3 * MORSE_DOT_LENGTH)
#define MORSE_WORD_PAUSE	(7 * MORSE_DOT_LENGTH)

#define BEEPER_COMMAND_REPEAT 0xFE
#define BEEPER_COMMAND_STOP   0xFF

enum {
	MDA,
	MDI,
	MEND
};

struct morse_letter {
	char letter;
	uint8_t code[6];
};

static const struct morse_letter morse_letters[] = {
	{ 'A', { MDI, MDA,	MEND }},
	{ 'B', { MDA, MDI, MDI, MDI, MEND }},
	{ 'C', { MDA, MDI, MDA, MDI, MEND }},
	{ 'D', { MDA, MDI, MDI, MEND }},
	{ 'E', { MDI, MEND }},
	{ 'F', { MDI, MDI, MDA, MDI, MEND }},
	{ 'G', { MDA, MDA, MDI, MEND }},
	{ 'H', { MDI, MDI, MDI, MDI, MEND }},
	{ 'I', { MDI, MDI, MEND }},
	{ 'J', { MDI, MDA, MDA, MDA, MEND }},
	{ 'K', { MDA, MDI, MDA, MEND }},
	{ 'L', { MDI, MDA, MDI, MDI, MEND }},
	{ 'M', { MDA, MDA, MEND }},
	{ 'N', { MDA, MDI, MEND }},
	{ 'O', { MDA, MDA, MDA, MEND }},
	{ 'P', { MDI, MDA, MDA, MDI, MEND }},
	{ 'Q', { MDA, MDA, MDI, MDA, MEND }},
	{ 'R', { MDI, MDA, MDI, MEND }},
	{ 'S', { MDI, MDI, MDI, MEND }},
	{ 'T', { MDA, MEND }},
	{ 'U', { MDI, MDI, MDA, MEND }},
	{ 'V', { MDI, MDI, MDI, MDA, MEND }},
	{ 'W', { MDI, MDA, MDA, MEND }},
	{ 'X', { MDA, MDI, MDI, MDA, MEND }},
	{ 'Y', { MDA, MDI, MDA, MDA, MEND }},
	{ 'Z', { MDA, MDA, MDI, MDI, MEND }},
	// Do not put any characters here unless you also modify the processing code
	{ '0', { MDA, MDA, MDA, MDA, MDA, MEND }},
	{ '1', { MDI, MDA, MDA, MDA, MDA, MEND }},
	{ '2', { MDI, MDI, MDA, MDA, MDA, MEND }},
	{ '3', { MDI, MDI, MDI, MDA, MDA, MEND }},
	{ '4', { MDI, MDI, MDI, MDI, MDA, MEND }},
	{ '5', { MDI, MDI, MDI, MDI, MDI, MEND }},
	{ '6', { MDA, MDI, MDI, MDI, MDI, MEND }},
	{ '7', { MDA, MDA, MDI, MDI, MDI, MEND }},
	{ '8', { MDA, MDA, MDA, MDI, MDI, MEND }},
	{ '9', { MDA, MDA, MDA, MDA, MDI, MEND }},
};

void beeper_init(struct beeper *self, const struct system_calls *system){
	memset(self, 0, sizeof(*self));
	self->system = system;
	cbuf_init(&self->buffer, self->buffer_data, sizeof(self->buffer_data));
	PT_INIT(&self->state);
}

/*
 * Called to activate/deactivate beeper, using the given "BEEPER_..." value.
 * This function returns immediately (does not block).
 */
void beeper_start(struct beeper *self, beeper_command_t mode){
	if (mode == BEEPER_SILENCE) {
		beeper_stop(self);
		return;
	}

	const char *text = NULL;

	switch(mode){
		case BEEPER_GYRO_CALIBRATED: text = "GCAL"; break;
		case BEEPER_RX_LOST_LANDING: text = "RXLOS LAND"; break;
		case BEEPER_RX_LOST: text = "RXLOS"; break;
		case BEEPER_DISARMING: text = "DIS"; break;
		case BEEPER_DISARM_REPEAT: text = "E"; break;
		case BEEPER_ARMING: text = "ARM"; break;
		case BEEPER_ARMING_GPS_FIX: text = "ARMGPS"; break;
		case BEEPER_BAT_CRIT_LOW: text = "BATSOS"; break;
		case BEEPER_BAT_LOW: text = "BATLOW"; break;
		case BEEPER_GPS_STATUS: text = "GPS"; break;
		case BEEPER_RX_SET: text = "E"; break;
		case BEEPER_ACC_CALIBRATION: text = "ACCCAL"; break;
		case BEEPER_ACC_CALIBRATION_FAIL: text = "FAIL ACCAL"; break;
		case BEEPER_READY_BEEP: text = "RDY"; break;
		case BEEPER_ARMED: text = "WARN"; break;
		case BEEPER_SILENCE: text = " "; break;
		default: text = NULL;
	}
	if(text)
		cbuf_write(&self->buffer, text, strlen(text));
	
	// TODO: do we need priority?
	/*
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
	*/
}

void beeper_write(struct beeper *self, const char *text){
	size_t len = strlen(text);
	if(!len) return;
	cbuf_write(&self->buffer, text, len);
}

void beeper_stop(struct beeper *self){
	sys_beeper_off(self->system);
	// clear any pending beeps
	cbuf_clear(&self->buffer);
	// reset the state machine
	PT_INIT(&self->state);
}

static PT_THREAD(_beeper_fsm(struct beeper *self)){
	PT_BEGIN(&self->state);
	while(true){
		PT_WAIT_UNTIL(&self->state, cbuf_cnt(&self->buffer) > 0);
		uint8_t ch = toupper(cbuf_get(&self->buffer) & 0xff);
		//printf("beep: %c\n", ch);
		//fflush(stdout);
		int idx = 0;
		if(isdigit(ch)) idx = ('Z' - 'A') + (ch - '0');
		else if(ch >= 'A' && ch <= 'Z') idx = ch - 'A';
		else if(ch == ' '){
			// do a word pause and continue
			self->timeout = sys_millis(self->system) + MORSE_WORD_PAUSE;
			PT_WAIT_UNTIL(&self->state, self->timeout < sys_millis(self->system));
			continue;
		} else {
			// unknown character: skip
			continue;
		}
		self->cur_letter = &morse_letters[idx];
		self->part_idx = 0;
		while(self->cur_letter->code[self->part_idx] != MEND){
			uint8_t part = self->cur_letter->code[self->part_idx];
			if(part == MDA) self->timeout = sys_millis(self->system) + MORSE_DASH_LENGTH;
			else if(part == MDI) self->timeout = sys_millis(self->system) + MORSE_DOT_LENGTH;
			// turn on the beeper and wait for the timeout
			sys_beeper_on(self->system);
			PT_WAIT_UNTIL(&self->state, sys_millis(self->system) > self->timeout);
			sys_beeper_off(self->system);
			self->timeout = sys_millis(self->system) + MORSE_PART_PAUSE;
			PT_WAIT_UNTIL(&self->state, sys_millis(self->system) > self->timeout);
			self->part_idx++;
		}
		self->timeout = sys_millis(self->system) + MORSE_LETTER_PAUSE;
		PT_WAIT_UNTIL(&self->state, sys_millis(self->system) > self->timeout);
	}
	PT_END(&self->state);
}

/**
 * Beeper handler function to be called periodically in loop. Updates beeper
 * state via time schedule.
 */
void beeper_update(struct beeper *self){
	_beeper_fsm(self);
}
