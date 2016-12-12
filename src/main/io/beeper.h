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

#include "system_calls.h"
#include "common/pt.h"

#include "utype/cbuf.h"

/**
 * @defgroup BEEPER Beeper
 * @ingroup INDICATORS
 *
 * @brief Beeper is used for auditory indication of various states and events
 *
 * The role of the beeper module is to provide a more advanced interface to the
 * system beeper function (which can only turn the beeper on or off). This
 * module provides several standard beep sequences that can be started by user
 * code and run asynchronously by periodically calling the update function.
 */

typedef enum {
    // IMPORTANT: these are in priority order, 0 = Highest
    BEEPER_NONE = 0,             //!< Silence, see beeperSilence()
    BEEPER_GYRO_CALIBRATED,
    BEEPER_RX_LOST_LANDING,         //!< Beeps SOS when armed and TX is turned off or signal lost (autolanding/autodisarm)
    BEEPER_RX_LOST,                 //!< Beeps when TX is turned off or signal lost (repeat until TX is okay)
    BEEPER_DISARMING,               //!< Beep when disarming the board
    BEEPER_ARMING,                  //!< Beep when arming the board
    BEEPER_ARMING_GPS_FIX,          //!< Beep a special tone when arming the board and GPS has fix
    BEEPER_BAT_CRIT_LOW,            //!< Longer warning beeps when battery is critically low (repeats)
    BEEPER_BAT_LOW,                 //!< Warning beeps when battery is getting low (repeats)
    BEEPER_GPS_STATUS,				//!< Beep used for indicating changed gps status (such as sattelites being aquired)
    BEEPER_RX_SET,                  //!< Beeps when aux channel is set for beep or beep sequence how many satellites has found if GPS enabled
	BEEPER_MULTI_BEEPS,				//!< multiple 20ms beeps
    BEEPER_DISARM_REPEAT,           //!< Beeps sounded while stick held in disarm position
    BEEPER_ACC_CALIBRATION,         //!< ACC inflight calibration completed confirmation
    BEEPER_ACC_CALIBRATION_FAIL,    //!< ACC inflight calibration failed
    BEEPER_READY_BEEP,              //!< Ring a tone when GPS is locked and ready
    BEEPER_ARMED,                   //!< Warning beeps when board is armed (repeats until board is disarmed or throttle is increased)
	BEEPER_MORSE,					//!< use the morse buffer to telegraph text
} beeper_command_t;

struct beeper_entry {
    uint8_t mode;
    uint8_t priority; // 0 = Highest
    const uint8_t *sequence;
#ifdef BEEPER_NAMES
    const char *name;
#endif
} ;

struct morse_letter;
struct beeper {
	int entry_pos;
	const struct beeper_entry *cur_entry;
	int multibeeps;

	struct pt state;
	struct cbuf buffer;
	char buffer_data[16];

	const struct morse_letter *cur_letter;
	uint8_t part_idx;

	sys_millis_t timeout;
	const struct system_calls *system;
};

//! Initializes defaults for beeper function
void beeper_init(struct beeper *self, const struct system_calls *system);
//! Write any text to the beeper (will be sent as morse)
void beeper_write(struct beeper *self, const char *text);
//! Puts beeper into a new beep state
bool beeper_start(struct beeper *self, beeper_command_t cmd);
//! Aborts current beeper function
void beeper_stop(struct beeper *self);
//! Updates beeper state
void beeper_update(struct beeper *self);
/*
uint32_t getArmingBeepTimeMicros(void);
beeperMode_e beeperModeForTableIndex(int idx);
const char *beeperNameForTableIndex(int idx);
int beeperTableEntryCount(void);
*/
