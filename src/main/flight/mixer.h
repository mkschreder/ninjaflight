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

#include "anglerate.h"

#if defined(USE_QUAD_MIXER_ONLY)
#define MAX_SUPPORTED_MOTORS 4

#elif defined(TARGET_MOTOR_COUNT)
#define MAX_SUPPORTED_MOTORS TARGET_MOTOR_COUNT

#else
#define MAX_SUPPORTED_MOTORS 12
#endif

#define YAW_JUMP_PREVENTION_LIMIT_LOW 80
#define YAW_JUMP_PREVENTION_LIMIT_HIGH 500

// Note: this is called MultiType/MULTITYPE_* in baseflight.
typedef enum mixerMode
{
    MIXER_TRI = 1,
    MIXER_QUADP = 2,
    MIXER_QUADX = 3,
    MIXER_BICOPTER = 4,
    MIXER_GIMBAL = 5,
    MIXER_Y6 = 6,
    MIXER_HEX6 = 7,
    MIXER_FLYING_WING = 8,
    MIXER_Y4 = 9,
    MIXER_HEX6X = 10,
    MIXER_OCTOX8 = 11,
    MIXER_OCTOFLATP = 12,
    MIXER_OCTOFLATX = 13,
    MIXER_AIRPLANE = 14,        // airplane / singlecopter / dualcopter (not yet properly supported)
    MIXER_HELI_120_CCPM = 15,
    MIXER_HELI_90_DEG = 16,
    MIXER_VTAIL4 = 17,
    MIXER_HEX6H = 18,
    MIXER_PPM_TO_SERVO = 19,    // PPM -> servo relay
    MIXER_DUALCOPTER = 20,
    MIXER_SINGLECOPTER = 21,
    MIXER_ATAIL4 = 22,
    MIXER_QUADX_TILT1 = 23,
    MIXER_QUADX_TILT2 = 24,
    MIXER_CUSTOM,
    MIXER_CUSTOM_AIRPLANE,
    MIXER_CUSTOM_TRI
} mixerMode_e;

// Custom mixer data per motor
struct motor_mixer {
    float throttle;
    float roll;
    float pitch;
    float yaw;
};


// Custom mixer configuration
struct mixer_mode {
    uint8_t motorCount;
    uint8_t useServo;
    const struct motor_mixer *motor;
}; 

struct mixer {
	uint8_t motorCount; 

	int16_t motor[MAX_SUPPORTED_MOTORS];
	int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];

	bool motorLimitReached;

	struct motor_mixer currentMixer[MAX_SUPPORTED_MOTORS];

	struct motor_mixer *customMixers;
	
	int16_t motor_pitch; 
	int16_t tilt_pwm; 

	bool mode3d;
};

// TODO: this is very bad way so remove this later once refactoring is done.
extern struct mixer default_mixer; 

struct mixer_config {
    uint8_t mixerMode;
    uint8_t pid_at_min_throttle;            // when enabled pids are used at minimum throttle
    int8_t yaw_motor_direction;
    uint16_t yaw_jump_prevention_limit;      // make limit configurable (original fixed value was 100)
#ifdef USE_SERVOS
    uint8_t tri_unarmed_servo;              // send tail servo correction pulses even when unarmed
    float servo_lowpass_freq;             // lowpass servo filter frequency selection; 1/1000ths of loop freq
    int8_t servo_lowpass_enable;            // enable/disable lowpass filter
#endif
};

typedef enum {
	MIXER_TILT_MODE_STATIC,
	MIXER_TILT_MODE_DYNAMIC
} mixer_tilt_mode_t; 

#define MIXER_TILT_COMPENSATE_THRUST (1 << 0)
#define MIXER_TILT_COMPENSATE_TILT (1 << 1)
#define MIXER_TILT_COMPENSATE_BODY (1 << 2)

struct mixer_tilt_config {
	uint8_t mode; 
	uint8_t compensation_flags;  
	uint8_t control_channel; 
	int8_t servo_angle_min; 
	int8_t servo_angle_max; 
}; 


struct motor_3d_config {
    uint16_t deadband3d_low;                // min 3d value
    uint16_t deadband3d_high;               // max 3d value
    uint16_t neutral3d;                     // center 3d value
};

#define CHANNEL_FORWARDING_DISABLED (uint8_t)0xFF

void mixer_init(struct mixer *self, struct motor_mixer *custom_mixers, uint8_t count);
void mixer_set_all_motors_pwm(struct mixer *self, int16_t mc);
void mixer_load_motor_mixer(struct mixer *self, int index, struct motor_mixer *custom_mixers);
void mixer_reset_disarmed_pwm_values(struct mixer *self);
void mixer_update(struct mixer *self, const struct pid_controller_output *pid_axis);
void mixer_update_servos(struct mixer *self, const struct pid_controller_output *pid_axis); 
void mixer_write_pwm(struct mixer *self);
void mixer_stop_motors(struct mixer *self);
void mixer_stop_pwm_all_motors(struct mixer *self);
void mixer_init_servo_filtering(struct mixer *self, uint32_t targetLooptime);

void mixer_enable_3d_mode(struct mixer *self, bool on);
void mixer_input_motor_pitch_angle(struct mixer *self, int16_t pitch_angle_dd); 
void mixer_set_motor_disarmed_pwm(struct mixer *self, uint8_t id, int16_t value); 
struct pwmIOConfiguration_s; // TODO: remove this kind of dependency 
void mixer_use_pwmio_config(struct mixer *self, struct pwmIOConfiguration_s *pwmIOConfiguration); 
int16_t mixer_get_motor_disarmed_pwm(struct mixer *self, uint8_t id); 
uint16_t mixer_get_motor_value(struct mixer *self, uint8_t id); 
bool mixer_motor_limit_reached(struct mixer *self); 
uint8_t mixer_get_motor_count(struct mixer *self); 
