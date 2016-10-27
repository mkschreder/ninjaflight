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

typedef struct {
	int16_t axis[3]; 
#ifdef BLACKBOX
	float axis_P[3]; 
	float axis_I[3]; 
	float axis_D[3]; 
#endif
} pid_controller_output_t; 

struct anglerate_controller {
	// PIDweight is a scale factor for PIDs which is derived from the throttle and TPA setting, and 100 = 100% scale means no PID reduction
	uint8_t PIDweight[3];

	int32_t lastITerm[3], ITermLimit[3];
	float lastITermf[3], ITermLimitf[3];

	biquad_t deltaFilterState[3];

	// update outputs based on current attitude information
	void (*update)(struct anglerate_controller *self, union attitude_euler_angles *att); 
	
	// used for luxfloat
	float lastRateForDelta[3];
    float deltaStatef[3][DTERM_AVERAGE_COUNT];

	// used for mwii23
	int32_t ITermAngle[2];
	uint8_t dynP8[3], dynI8[3], dynD8[3];

	// used for mwiirewrite
	int32_t lastRateForDeltai[3];
	int32_t deltaStatei[3][DTERM_AVERAGE_COUNT]; 

	pid_controller_output_t output; 

	bool _delta_state_set; 
	const struct rate_config *rate_config; 
	const struct pid_config *config; 
	uint16_t max_angle_inclination;  
	const rollAndPitchTrims_t *angle_trim;  
	const rxConfig_t *rx_config; 
}; 

// TODO: remove when done refactoring. This should be a member of a higher level struct.  
extern struct anglerate_controller default_controller; 

#define IS_PID_CONTROLLER_FP_BASED(pidController) (pidController == PID_CONTROLLER_LUX_FLOAT)
//float pidScaleITermToRcInput(int axis);
//void pidFilterIsSetCheck(const struct pid_config *pidProfile);

void anglerate_controller_init(struct anglerate_controller *self); 
void anglerate_controller_set_algo(struct anglerate_controller *self, pid_controller_type_t type);
void anglerate_controller_reset_angle_i(struct anglerate_controller *self);
void anglerate_controller_reset_rate_i(struct anglerate_controller *self);
const pid_controller_output_t *anglerate_controller_get_output_ptr(struct anglerate_controller *self); 
void anglerate_controller_update(struct anglerate_controller *self, union attitude_euler_angles *att); 

void anglerate_controller_set_pid_axis_scale(struct anglerate_controller *self, uint8_t axis, int32_t scale); 
void anglerate_controller_set_pid_axis_weight(struct anglerate_controller *self, uint8_t axis, int32_t weight); 

// TODO: this should be removed
void anglerate_controller_set_configs(struct anglerate_controller *self,
	const struct pid_config *config,
	const struct rate_config *rate_config, 
	uint16_t max_angle_inclination, 
	const rollAndPitchTrims_t *angleTrim, 
	const rxConfig_t *rxConfig); 
