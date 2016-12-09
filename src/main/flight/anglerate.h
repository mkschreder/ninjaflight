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

#include <stdint.h>
#include <stdbool.h>

#include "common/axis.h"
#include "common/filter.h"

#include "sensors/acceleration.h"
#include "sensors/instruments.h"

#include "rx/rx.h"

#include "rate_profile.h"

#include "../config/anglerate.h"

union rollAndPitchTrims_u;
struct rxConfig_s;

struct pid_controller_output{
	int16_t axis[3];

	// TODO: remove these intermediate values. For now leaving them here because unit tests rely on them.
	// unit tests need to be rewritten to not rely on any kinf of intermediate crap and just test expected behavior!
	float axis_P[3];
	float axis_I[3];
	float axis_D[3];
};

struct anglerate {
	// PIDweight is a scale factor for PIDs which is derived from the throttle and TPA setting, and 100 = 100% scale means no PID reduction
	uint8_t PIDweight[3];

	int32_t lastITerm[3], ITermLimit[3];
	float lastITermf[3], ITermLimitf[3];

	biquad_t deltaFilterState[3];

	// update outputs based on current attitude information
	void (*update)(struct anglerate *self, float dT);

	int16_t body_rates[3];
	int16_t body_angles[3];
	int16_t user[3]; //!< user input command

	// used for luxfloat
	float lastRateForDelta[3];
    float deltaStatef[3][DTERM_AVERAGE_COUNT];

	// used for mwii23
	int32_t ITermAngle[2];
	uint8_t pidScale[3];

	// used for mwiirewrite
	int32_t lastRateForDeltai[3];
	int32_t deltaStatei[3][DTERM_AVERAGE_COUNT];

	struct pid_controller_output output;

	bool _delta_state_set;
	uint16_t max_angle_inclination;

	uint8_t level_percent[2];
	uint8_t flags;

	struct instruments *ins;
	const struct config * config;
};

#define IS_PID_CONTROLLER_FP_BASED(pidController) (pidController == PID_CONTROLLER_LUX_FLOAT)
//float pidScaleITermToRcInput(int axis);
//void pidFilterIsSetCheck(const struct pid_config *pidProfile);

void anglerate_init(struct anglerate *self,
	struct instruments *ins,
	const struct config * config);
void anglerate_set_algo(struct anglerate *self, pid_controller_type_t type);
void anglerate_reset_angle_i(struct anglerate *self);
void anglerate_reset_rate_i(struct anglerate *self);
const struct pid_controller_output *anglerate_get_output_ptr(struct anglerate *self);

void anglerate_input_body_rates(struct anglerate *self, int16_t x, int16_t y, int16_t z);
void anglerate_input_body_angles(struct anglerate *self, int16_t roll, int16_t pitch, int16_t yaw);
void anglerate_input_user(struct anglerate *self, int16_t roll, int16_t pitch, int16_t yaw);

static inline int16_t anglerate_get_roll(struct anglerate *self) { return self->output.axis[0]; }
static inline int16_t anglerate_get_pitch(struct anglerate *self) { return self->output.axis[1]; }
static inline int16_t anglerate_get_yaw(struct anglerate *self) { return self->output.axis[2]; }

void anglerate_update(struct anglerate *self, float dT);

void anglerate_enable_antiwindup(struct anglerate *self, bool on);
void anglerate_enable_plimit(struct anglerate *self, bool on);

// TODO: unify pid scaling so we have just one call for all pid controllers (axis scale is only used in mw23)
void anglerate_set_pid_axis_scale(struct anglerate *self, uint8_t axis, int32_t scale);
void anglerate_set_pid_axis_weight(struct anglerate *self, uint8_t axis, int32_t weight);
void anglerate_set_level_percent(struct anglerate *self, uint8_t roll, uint8_t pitch);
