#pragma once

#include <stdint.h>

//! mixer tilt mode type
typedef enum {
	MIXER_TILT_MODE_STATIC,
	MIXER_TILT_MODE_DYNAMIC
} mixer_tilt_mode_t;

#define MIXER_TILT_COMPENSATE_THRUST (1 << 0)
#define MIXER_TILT_COMPENSATE_TILT (1 << 1)
#define MIXER_TILT_COMPENSATE_BODY (1 << 2)

struct tilt_config {
	uint8_t mode;
	uint8_t compensation_flags;
	uint8_t control_channel;
	int8_t servo_angle_min;
	int8_t servo_angle_max;
};

struct tilt_input_params {
	int16_t motor_pitch_dd;
	int16_t body_pitch_dd;
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
	int16_t throttle;
};

struct tilt_output_params {
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
	int16_t throttle;
};

void tilt_calculate_compensation(const struct tilt_config *tilt, const struct tilt_input_params *input, struct tilt_output_params *output);
