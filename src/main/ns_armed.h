#pragma once

#include "ns_state.h"

struct ns_armed {
	struct ninja_state state;
	sys_micros_t prev_time;
};

void ns_armed_init(struct ns_armed *self, struct ninja_state *parent);
