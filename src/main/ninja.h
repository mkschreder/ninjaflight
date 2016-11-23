#pragma once

#include "sensors/instruments.h"
#include "flight/mixer.h"
#include "sensors/battery.h"
#include "system_calls.h"
#include "flight/anglerate.h"
#include "ninja_sched.h"
#include "ninja_input.h"
#include "rc_commands.h"

struct ninja;

struct ninja_rc_input {
	int16_t raw[8];
};

struct ninja_state;
struct ninja {
	struct instruments ins;
	struct mixer mixer;
	struct anglerate ctrl;
	struct battery bat;
	struct rc_command rc_command;

	bool isRXDataNew;

	int16_t magHold;
	int16_t headFreeModeHold;

	uint8_t motorControlEnable;

	int16_t telemTemperature1;      // gyro sensor temperature
	uint32_t disarmAt;     // Time of automatic disarm when "Don't spin the motors when armed" is enabled and auto_disarm_delay is nonzero

	uint16_t filteredCycleTime;
	uint16_t cycleTime;

	struct ninja_rc_input rc_input;
	const struct ninja_state *state;

	struct ninja_sched sched;

	const struct system_calls *syscalls;
};

void ninja_init(struct ninja *self, const struct system_calls *syscalls);

void ninja_arm(struct ninja *self);
void ninja_disarm(struct ninja *self);

void ninja_heartbeat(struct ninja *self);
void ninja_input_rc(struct ninja *self, const struct ninja_rc_input *rc);
void ninja_input_gyro(struct ninja *self, int32_t x, int32_t y, int32_t z);
void ninja_input_acc(struct ninja *self, int32_t x, int32_t y, int32_t z);
