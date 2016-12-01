#pragma once

struct ninja;
struct ninja_state {
	void (*enter)(struct ninja_state *self, struct ninja *ninja);
	void (*leave)(struct ninja_state *self, struct ninja *ninja);
	struct ninja_state *(*run)(struct ninja_state *self, struct ninja *ninja);
	void (*on_key_event)(struct ninja_state *self, struct ninja *ninja, rc_key_t key, rc_key_state_t state);
	struct ninja_state *parent;
};
