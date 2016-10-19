/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

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
    MIXER_CUSTOM = 23,
    MIXER_CUSTOM_AIRPLANE = 24,
    MIXER_CUSTOM_TRI = 25
} mixerMode_e;

// Custom mixer data per motor
struct motor_mixer {
    float throttle;
    float roll;
    float pitch;
    float yaw;
};

PG_DECLARE_ARR(struct motor_mixer, MAX_SUPPORTED_MOTORS, customMotorMixer);

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

PG_DECLARE(struct mixer_config, mixerConfig);

struct motor_3d_config {
    uint16_t deadband3d_low;                // min 3d value
    uint16_t deadband3d_high;               // max 3d value
    uint16_t neutral3d;                     // center 3d value
};

PG_DECLARE(struct motor_3d_config, motor3DConfig);

#define CHANNEL_FORWARDING_DISABLED (uint8_t)0xFF

void mixer_init(struct mixer *self, struct motor_mixer *custom_mixers, uint8_t count);
void writeAllMotors(struct mixer *self, int16_t mc);
void mixer_load_motor_mixer(struct mixer *self, int index, struct motor_mixer *custom_mixers);
void mixerResetDisarmedMotors(struct mixer *self);
void mixTable(struct mixer *self);
void servoMixTable(struct mixer *self);
void writeMotors(struct mixer *self);
void stopMotors(struct mixer *self);
void StopPwmAllMotors(struct mixer *self);
void mixerInitialiseServoFiltering(struct mixer *self, uint32_t targetLooptime);
void mixer_set_motor_disarmed_pwm(struct mixer *self, uint8_t id, int16_t value); 
int16_t mixer_get_motor_disarmed_pwm(struct mixer *self, uint8_t id); 
uint16_t mixer_get_motor_value(struct mixer *self, uint8_t id); 
bool mixer_motor_limit_reached(struct mixer *self); 
uint8_t mixer_get_motor_count(struct mixer *self); 
