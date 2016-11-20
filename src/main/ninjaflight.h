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

#include "config/accelerometer.h"

#include "ninja.h"

void ninja_calibrate_acc(void);
void ninja_calibrate_mag(void);

void applyAndSaveAccelerometerTrimsDelta(rollAndPitchTrims_t *rollAndPitchTrimsDelta);
void handleInflightCalibrationStickPosition(void);

void mwDisarm(void);
void mwArm(void);

bool isCalibrating(void);

void ninja_update_transponder(struct ninja *self);
void ninja_run_pid_loop(struct ninja *self, float dT);
