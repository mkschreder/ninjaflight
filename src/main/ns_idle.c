#include <string.h>
#include "ninja.h"
#include "ns_calibration.h"

static void _state_enter(struct ninja_state *state, struct ninja *ninja){
	(void)state; (void)ninja;
	printf("idle enter\n");
	fflush(stdout);
}

static void _state_leave(struct ninja_state *state, struct ninja *ninja){
	(void)state; (void)ninja;
	printf("idle leave\n");
	fflush(stdout);
}

static struct ninja_state* _state_run(struct ninja_state *state, struct ninja *ninja){
	struct ns_idle *self = container_of(state, struct ns_idle, state);

	for(int c = 0; c < MIXER_MAX_MOTORS; c++){
		sys_motor_write(ninja->system, c, 1000);
	}
	for(int c = 0; c < MIXER_MAX_SERVOS; c++){
		sys_servo_write(ninja->system, c, 1500);
	}

	if(!ins_is_calibrated(&ninja->ins)){
		return &ninja->ns_calibration.state;
	}

	if(self->arm_timeout == 0){
		if(rc_key_active(&ninja->rc, RC_KEY_ARM)){
			self->arm_timeout = sys_millis(ninja->system) + 500;
		}
	} else {
		bool still_arming = rc_key_active(&ninja->rc, RC_KEY_ARM);
		if(still_arming && (sys_millis(ninja->system) - self->arm_timeout > 0)) {
			self->arm_timeout = 0;
			return &ninja->ns_armed.state;
		} else if(!still_arming){
			self->arm_timeout = 0;
		}
	}
	return NULL;
}

void ns_idle_init(struct ns_idle *self, struct ninja_state *parent){
	memset(self, 0, sizeof(*self));
	self->state = (struct ninja_state){
		.enter = _state_enter,
		.leave = _state_leave,
		.run = _state_run,
		.parent = parent
	};
}

