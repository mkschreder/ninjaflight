#include <string.h>

#include <math.h>
#include "common/maths.h"
#include "ninja.h"
#include "ns_armed.h"

#include <stdio.h>

static void _state_enter(struct ninja_state *state, struct ninja *ninja){
	(void)state;

	printf("armed\n");
	ninja->is_armed = true;
	mixer_enable_armed(&ninja->mixer, true);
/*
 * 	// TODO: start blackbox?
	if (feature(FEATURE_BLACKBOX)) {
		serialPort_t *sharedBlackboxAndMspPort = findSharedSerialPort(FUNCTION_BLACKBOX, FUNCTION_MSP);
		if (sharedBlackboxAndMspPort) {
			mspSerialReleasePortIfAllocated(sharedBlackboxAndMspPort);
		}
		startBlackbox();
	}
*/
	//beep to indicate arming
	beeper_start(&ninja->beeper, BEEPER_ARMING);
}

static void _state_leave(struct ninja_state *state, struct ninja *ninja){
	(void)state;
	printf("disarmed\n");
	mixer_enable_armed(&ninja->mixer, false);
/*
	// TODO: stop blackbox?
	if (feature(FEATURE_BLACKBOX)) {
		finishBlackbox();
	}
*/

	beeper_start(&ninja->beeper, BEEPER_DISARMING);	  // emit disarm tone
	ninja->is_armed = false;
}

static struct ninja_state * _state_run(struct ninja_state *_state, struct ninja *self){
	(void)_state;
	//struct ns_armed *state = container_of(_state, struct ns_armed, state);

	if(rc_key_state(&self->rc, RC_KEY_FUNC_ARM) == RC_KEY_RELEASED){
		return &self->ns_idle.state;
	}

	sys_micros_t dt_us = self->loop_time;

	ninja_handle_input(self);

	// TODO: set pid algo when config is applied
	//anglerate_set_algo(&self->ctrl, pidProfile()->pidController);
	anglerate_set_algo(&self->ctrl, PID_CONTROLLER_LUX_FLOAT);

	if (rc_key_state(&self->rc, RC_KEY_FUNC_LEVEL) == RC_KEY_PRESSED) {
		anglerate_set_level_percent(&self->ctrl, 100, 100);
	} else if(rc_key_state(&self->rc, RC_KEY_FUNC_BLEND) == RC_KEY_PRESSED){
		int16_t hp_roll = 100-ABS(rx_get_channel(&self->rx, ROLL) - 1500) / 5;
		int16_t hp_pitch = 100-ABS(rx_get_channel(&self->rx, PITCH) - 1500) / 5;
		if(ABS(ins_get_pitch_dd(&self->ins)) > 800)
			hp_roll = hp_pitch = 0;
		int16_t strength = MIN(hp_roll, hp_pitch);
		anglerate_set_level_percent(&self->ctrl, strength, strength);
	} else {
		anglerate_set_level_percent(&self->ctrl, 0, 0);
	}
	//_process_tilt_controls(self);

	int16_t roll = rc_get_command(&self->rc, ROLL);
	int16_t pitch =  rc_get_command(&self->rc, PITCH);
	int16_t yaw = rc_get_command(&self->rc, YAW);

	// prevent spinup when just armed
	if(rc_get_command(&self->rc, THROTTLE) < -480) yaw = 0;

	float combined = degreesToRadians(DECIDEGREES_TO_DEGREES(ins_get_pitch_dd(&self->ins)));
    float tmpCosine = cos_approx(combined);
	float rollCompensation = roll * tmpCosine;
	float rollCompensationInv = roll - rollCompensation;
	float yawCompensation = yaw * tmpCosine;
	float yawCompensationInv = yaw - yawCompensation;

	roll = (yawCompensationInv + rollCompensation);
	yaw = -(yawCompensation + rollCompensationInv);

	anglerate_input_user(&self->ctrl, roll, pitch, yaw);
	anglerate_input_body_rates(&self->ctrl, ins_get_gyro_x(&self->ins), ins_get_gyro_y(&self->ins), ins_get_gyro_z(&self->ins));
	anglerate_input_body_angles(&self->ctrl, ins_get_roll_dd(&self->ins), ins_get_pitch_dd(&self->ins), ins_get_yaw_dd(&self->ins));
	anglerate_update(&self->ctrl, dt_us * 1e-6f);

	mixer_set_throttle_range(&self->mixer, 1500, motorAndServoConfig()->minthrottle, motorAndServoConfig()->maxthrottle);
	mixer_input_command(&self->mixer, MIXER_INPUT_G0_THROTTLE, rc_get_command(&self->rc, THROTTLE));

	// TODO: flight modes
	//if (FLIGHT_MODE(PASSTHRU_MODE)) {
		// Direct passthru from RX
		//mixer_input_command(&self->mixer, MIXER_INPUT_G0_ROLL, rcCommand[ROLL]);
		//mixer_input_command(&self->mixer, MIXER_INPUT_G0_PITCH, rcCommand[PITCH]);
		//mixer_input_command(&self->mixer, MIXER_INPUT_G0_YAW, rcCommand[YAW]);
	//} else {
		// Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
		mixer_input_command(&self->mixer, MIXER_INPUT_G0_ROLL, anglerate_get_roll(&self->ctrl));
		mixer_input_command(&self->mixer, MIXER_INPUT_G0_PITCH, anglerate_get_pitch(&self->ctrl));
		mixer_input_command(&self->mixer, MIXER_INPUT_G0_YAW, -anglerate_get_yaw(&self->ctrl));
	//}

	//printf("out: %d %d %d\n", anglerate_get_roll(&self->ctrl), anglerate_get_pitch(&self->ctrl), anglerate_get_yaw(&self->ctrl));
	// center the RC input value around the RC middle value
	// by subtracting the RC middle value from the RC input value, we get:
	// data - middle = input
	// 2000 - 1500 = +500
	// 1500 - 1500 = 0
	// 1000 - 1500 = -500
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_ROLL, rx_get_channel(&self->rx, ROLL) - rxConfig()->midrc);
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_PITCH, rx_get_channel(&self->rx, PITCH)	- rxConfig()->midrc);
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_YAW, rx_get_channel(&self->rx, YAW)	  - rxConfig()->midrc);
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_THROTTLE, rx_get_channel(&self->rx, THROTTLE) - rxConfig()->midrc);
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_AUX1, rx_get_channel(&self->rx, AUX1)	 - rxConfig()->midrc);
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_AUX2, rx_get_channel(&self->rx, AUX2)	 - rxConfig()->midrc);
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_AUX3, rx_get_channel(&self->rx, AUX3)	 - rxConfig()->midrc);

	mixer_update(&self->mixer);

	//self->disarmAt = millis() + armingConfig()->auto_disarm_delay * 1000;   // start disarm timeout, will be extended when throttle is nonzero
	return NULL;
}

void ns_armed_init(struct ns_armed *self, struct ninja_state *parent){
	memset(self, 0, sizeof(*self));
	self->state = (struct ninja_state){
		.enter = _state_enter,
		.leave = _state_leave,
		.run = _state_run,
		.on_key_event = NULL,
		.parent = parent
	};
}

