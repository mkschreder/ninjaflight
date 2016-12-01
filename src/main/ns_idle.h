#pragma once

#include "ns_state.h"

struct ns_idle {
	struct ninja_state state;
	sys_millis_t arm_timeout;
};

void ns_idle_init(struct ns_idle *self, struct ninja_state *parent);
