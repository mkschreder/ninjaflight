#include <stdint.h>
#include <math.h>

#include <platform.h>

#include "common/maths.h"

#include "tilt.h"

void tilt_calculate_compensation(const struct tilt_config *tilt, const struct tilt_input_params *input, struct tilt_output_params *output) {
	int16_t throttle = constrain(input->throttle, 0, 1000);
    float motor_pitch_rad = degreesToRadians(DECIDEGREES_TO_DEGREES(input->motor_pitch_dd));
	float bodyPitch = degreesToRadians(DECIDEGREES_TO_DEGREES(input->body_pitch_dd));
	int16_t combined = (motor_pitch_rad - bodyPitch);
	float sign = (0 < combined) - (combined < 0);
    float tmpCosine = cos_approx(motor_pitch_rad - bodyPitch);

	output->throttle = throttle;
	output->pitch = 0;
	output->roll = 0;
	output->yaw = 0;

	// if static mode then for now we just set servos to middle and exit
	if(tilt->mode == MIXER_TILT_MODE_STATIC)
		return;

    if (tilt->compensation_flags & MIXER_TILT_COMPENSATE_THRUST && tmpCosine > 0) {
		output->throttle = constrain(throttle / tmpCosine, 0, 1000);
    }

	// compensate pitch tilt in terms of yaw and roll controls
    if (tilt->compensation_flags & MIXER_TILT_COMPENSATE_TILT) {
        float rollCompensation = input->roll * tmpCosine;
        float rollCompensationInv = input->roll - rollCompensation;
        float yawCompensation = input->yaw * tmpCosine;
        float yawCompensationInv = input->yaw - yawCompensation;

        output->roll = sign * (yawCompensationInv + rollCompensation);
        output->yaw = sign * (yawCompensation + rollCompensationInv);
    }
}
