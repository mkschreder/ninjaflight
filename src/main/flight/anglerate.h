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

#define PID_MAX_I 256
#define PID_MAX_D 512
#define PID_MAX_TOTAL_PID 1000

#define GYRO_I_MAX 256                      // Gyro I limiter
#define YAW_P_LIMIT_MIN 100                 // Maximum value for yaw P limiter
#define YAW_P_LIMIT_MAX 500                 // Maximum value for yaw P limiter

#define DTERM_AVERAGE_COUNT 4

#include <stdint.h>
#include <stdbool.h>

#include "common/axis.h"
#include "common/filter.h"

#include "sensors/acceleration.h"
#include "imu.h"

#include "rx/rx.h"

#include "rate_profile.h"

typedef enum {
    PIDROLL,
    PIDPITCH,
    PIDYAW,
    PIDALT,
    PIDPOS,
    PIDPOSR,
    PIDNAVR,
    PIDLEVEL,
    PIDMAG,
    PIDVEL,
    PID_ITEM_COUNT
} pid_index_t;

typedef enum {
	PID_CONTROLLER_MW23 = 0,
    PID_CONTROLLER_MWREWRITE,
    PID_CONTROLLER_LUX_FLOAT,
    PID_COUNT
} pid_controller_type_t;

struct pid_config {
    uint8_t P8[PID_ITEM_COUNT];
    uint8_t I8[PID_ITEM_COUNT];
    uint8_t D8[PID_ITEM_COUNT];
    uint8_t pidController;
    uint16_t yaw_p_limit;                   // set P term limit (fixed value was 300)
    uint16_t dterm_cut_hz;                  // dterm filtering
};

union rollAndPitchTrims_u;
struct rxConfig_s;

struct pid_controller_output{
	int16_t axis[3];
#ifdef BLACKBOX
	float axis_P[3];
	float axis_I[3];
	float axis_D[3];
#endif
};

struct anglerate {
	// PIDweight is a scale factor for PIDs which is derived from the throttle and TPA setting, and 100 = 100% scale means no PID reduction
	uint8_t PIDweight[3];

	int32_t lastITerm[3], ITermLimit[3];
	float lastITermf[3], ITermLimitf[3];

	biquad_t deltaFilterState[3];

	// update outputs based on current attitude information
	void (*update)(struct anglerate *self, union attitude_euler_angles *att, float dT);

	// used for luxfloat
	float lastRateForDelta[3];
    float deltaStatef[3][DTERM_AVERAGE_COUNT];

	// used for mwii23
	int32_t ITermAngle[2];
	uint8_t dynP8[3], dynI8[3], dynD8[3];

	// used for mwiirewrite
	int32_t lastRateForDeltai[3];
	int32_t deltaStatei[3][DTERM_AVERAGE_COUNT];

	struct pid_controller_output output;

	bool _delta_state_set;
	const struct rate_config *rate_config;
	const struct pid_config *config;
	uint16_t max_angle_inclination;
	const rollAndPitchTrims_t *angle_trim;
	const rxConfig_t *rx_config;

	// gyro data used for stabilization
	int16_t gyro[3];

	uint8_t flags;
};

// TODO: remove when done refactoring. This should be a member of a higher level struct.
extern struct anglerate default_controller;

#define IS_PID_CONTROLLER_FP_BASED(pidController) (pidController == PID_CONTROLLER_LUX_FLOAT)
//float pidScaleITermToRcInput(int axis);
//void pidFilterIsSetCheck(const struct pid_config *pidProfile);

void anglerate_init(struct anglerate *self);
void anglerate_set_algo(struct anglerate *self, pid_controller_type_t type);
void anglerate_reset_angle_i(struct anglerate *self);
void anglerate_reset_rate_i(struct anglerate *self);
const struct pid_controller_output *anglerate_get_output_ptr(struct anglerate *self);
void anglerate_input_gyro(struct anglerate *self, int16_t x, int16_t y, int16_t z);
void anglerate_update(struct anglerate *self, union attitude_euler_angles *att, float dT);

void anglerate_enable_antiwindup(struct anglerate *self, bool on);
void anglerate_enable_plimit(struct anglerate *self, bool on);
void anglerate_set_pid_axis_scale(struct anglerate *self, uint8_t axis, int32_t scale);
void anglerate_set_pid_axis_weight(struct anglerate *self, uint8_t axis, int32_t weight);

// TODO: this should be removed
void anglerate_set_configs(struct anglerate *self,
	const struct pid_config *config,
	const struct rate_config *rate_config,
	uint16_t max_angle_inclination,
	const rollAndPitchTrims_t *angleTrim,
	const rxConfig_t *rxConfig);
