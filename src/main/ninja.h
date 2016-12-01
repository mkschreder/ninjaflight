#pragma once

#include <stdio.h>

#include "sensors/instruments.h"
#include "flight/mixer.h"
#include "sensors/battery.h"
#include "system_calls.h"
#include "flight/anglerate.h"
#include "ninja_sched.h"
#include "ninja_input.h"
#include "io/rc_adjustments.h"
#include "flight/failsafe.h"
#include "io/ledstrip.h"
#include "io/beeper.h"
#include "cli.h"
#include "sensors/gps.h"
#include "rx/rc.h"

struct ninja;

typedef enum {
	NINJA_SENSOR_GYRO		= (1 << 0),
	NINJA_SENSOR_ACC		= (1 << 1),
	NINJA_SENSOR_MAG		= (1 << 2),
	NINJA_SENSOR_GPS		= (1 << 3),
	NINJA_SENSOR_BARO		= (1 << 4),
	NINJA_SENSOR_SONAR		= (1 << 5)
} sensor_mask_t;

struct ninja_rc_input {
	int16_t raw[8];
};

#include "ns_state.h"
#include "ns_idle.h"
#include "ns_calibration.h"
#include "ns_armed.h"

struct ninja_state;
struct ninja {
	struct instruments ins;
	struct mixer mixer;
	struct anglerate ctrl;
	struct battery bat;
	struct rc rc;
	struct rx rx;
	struct rc_adj rc_adj;
	struct failsafe failsafe;
	struct ledstrip ledstrip;
	struct beeper beeper;
	struct cli cli;
	struct gps gps;

	bool isRXDataNew;

	int16_t magHold;
	int16_t headFreeModeHold;

	uint8_t motorControlEnable;

	int16_t telemTemperature1;      // gyro sensor temperature
	uint32_t disarmAt;     // Time of automatic disarm when "Don't spin the motors when armed" is enabled and auto_disarm_delay is nonzero

	uint16_t filteredCycleTime;
	uint16_t cycleTime;

	struct ninja_rc_input rc_input;

	struct ninja_state *state;

	struct ns_armed ns_armed;
	struct ns_idle ns_idle;
	struct ns_calibration ns_calibration;

	bool is_armed;
	uint32_t sensors;

	sys_micros_t loop_time;

	struct ninja_sched sched;

	//! this is used for directly controlling motors while we are disarmed (from gcs)
	uint16_t direct_outputs[MIXER_OUTPUT_COUNT];

	//uint8_t armingFlags;
	//uint8_t stateFlags;
	//uint16_t flightModeFlags;
	//uint32_t enabledSensors;

	const struct system_calls *system;
};

void ninja_init(struct ninja *self, const struct system_calls *syscalls);

void ninja_arm(struct ninja *self);
void ninja_disarm(struct ninja *self);

uint32_t ninja_has_sensors(struct ninja *self, sensor_mask_t sensor_mask);
bool ninja_is_armed(struct ninja *self);

void ninja_heartbeat(struct ninja *self);
void ninja_input_rc(struct ninja *self, const struct ninja_rc_input *rc);
void ninja_input_gyro(struct ninja *self, int32_t x, int32_t y, int32_t z);
void ninja_input_acc(struct ninja *self, int32_t x, int32_t y, int32_t z);

void ninja_run_pid_loop(struct ninja *self, uint32_t dt_us);
