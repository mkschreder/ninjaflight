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

typedef struct rollAndPitchTrims_s {
    int16_t roll;
    int16_t pitch;
} __attribute__((packed)) rollAndPitchTrims_t_def;

typedef union rollAndPitchTrims_u {
    int16_t raw[2];
    rollAndPitchTrims_t_def values;
} __attribute__((packed)) rollAndPitchTrims_t;

typedef struct accDeadband_s {
    uint8_t xy;                 // set the acc deadband for xy-Axis
    uint8_t z;                  // set the acc deadband for z-Axis, this ignores small accelerations
} __attribute__((packed))  accDeadband_t;

struct accelerometer_config {
    rollAndPitchTrims_t trims; // accelerometer trim

    // sensor-related stuff
    uint8_t acc_cut_hz;                     // Set the Low Pass Filter factor for ACC. Reducing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time. Zero = no filter
    float accz_lpf_cutoff;                  // cutoff frequency for the low pass filter used on the acc z-axis for althold in Hz
    accDeadband_t accDeadband;
    uint8_t acc_unarmedcal;                 // turn automatic acc compensation on/off
} __attribute__((packed)) ;

