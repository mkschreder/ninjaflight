#pragma once

#include <stdint.h>
#include "../config/tilt.h"

//! mixer tilt mode type
typedef enum {
	MIXER_TILT_MODE_STATIC,
	MIXER_TILT_MODE_DYNAMIC
} mixer_tilt_mode_t;

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
