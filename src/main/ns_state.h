#pragma once

struct ninja;
struct ninja_state {
	void (*enter)(struct ninja_state *self, struct ninja *ninja);
	void (*leave)(struct ninja_state *self, struct ninja *ninja);
	struct ninja_state *(*run)(struct ninja_state *self, struct ninja *ninja);
	struct ninja_state *parent;
};
