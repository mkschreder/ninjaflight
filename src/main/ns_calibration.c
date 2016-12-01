#include <string.h>
#include "ninja.h"
#include "ns_calibration.h"

#include <stdio.h>

static void _state_enter(struct ninja_state *state, struct ninja *ninja){
	(void)state; (void)ninja;
	printf("calib enter\n");
	fflush(stdout);
}

static void _state_leave(struct ninja_state *state, struct ninja *ninja){
	(void)state; (void)ninja;
	printf("calib leave\n");
	fflush(stdout);
}

static struct ninja_state* _state_run(struct ninja_state *state, struct ninja *ninja){
	(void)state;
	// if we are done with the calibration then we go to idle state
	if(ins_is_calibrated(&ninja->ins)){
		return &ninja->ns_idle.state;
	}
	return NULL;
}

void ns_calibration_init(struct ns_calibration *self, struct ninja_state *parent){
	memset(self, 0, sizeof(*self));
	self->state = (struct ninja_state){
		.enter = _state_enter,
		.leave = _state_leave,
		.run = _state_run,
		.parent = parent
	};
}

