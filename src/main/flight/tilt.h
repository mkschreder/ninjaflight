#pragma once

#include <stdint.h>
#include "../config/tilt.h"

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
