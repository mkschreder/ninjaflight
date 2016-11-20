#pragma once

#include "sensors/instruments.h"
#include "flight/mixer.h"
#include "sensors/battery.h"

struct ninja {
	struct instruments ins;
	struct mixer mixer;
	struct anglerate ctrl;
	struct battery bat;

	bool isRXDataNew;

	int16_t magHold;
	int16_t headFreeModeHold;

	uint8_t motorControlEnable;

	int16_t telemTemperature1;      // gyro sensor temperature
	uint32_t disarmAt;     // Time of automatic disarm when "Don't spin the motors when armed" is enabled and auto_disarm_delay is nonzero

	uint16_t filteredCycleTime;
	uint16_t cycleTime;
};

// TODO: remove later
extern struct ninja ninja;

void ninja_init(struct ninja *self);
void ninja_update_controller(struct ninja *self, float dt);
void ninja_process_rc_sticks(struct ninja *self, rxConfig_t *rxConfig, throttleStatus_e throttleStatus, bool retarded_arm, bool disarm_kill_switch);
void ninja_process_rx(struct ninja *self);
