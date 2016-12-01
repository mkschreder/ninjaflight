#pragma once

struct ns_calibration {
	struct ninja_state state;
	// place state variables here..
};

void ns_calibration_init(struct ns_calibration *self, struct ninja_state *parent);
