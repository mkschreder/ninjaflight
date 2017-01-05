#pragma once

#include <stdio.h>

#include "blackbox/blackbox.h"

#include "flight/anglerate.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/mixer.h"

#include "io/rc_adjustments.h"
#include "io/ledstrip.h"
#include "io/beeper.h"
#include "io/serial_msp.h"

#include "rx/rc.h"

#include "sensors/battery.h"
#include "sensors/gps.h"
#include "sensors/instruments.h"

#include "cli.h"
#include "msp.h"
#include "common/pt.h"
#include "system_calls.h"
#include "ninja_sched.h"
#include "ninja_config.h"
#include "fastloop.h"

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

struct ninja_state;
struct ninja {
	struct battery bat;
	struct blackbox blackbox;
	struct rc rc;
	struct rc_event_listener rc_evl;
	struct rx rx;
	struct rc_adj rc_adj;
	struct failsafe failsafe;
	struct ledstrip ledstrip;
	struct beeper beeper;
	struct cli cli;
	struct gps gps;
	struct serial_msp serial_msp;
	struct msp msp;
	struct althold althold;

	bool isRXDataNew;

	int16_t magHold;
	int16_t headFreeModeHold;

	uint8_t motorControlEnable;

	int16_t telemTemperature1;      // gyro sensor temperature
	uint32_t disarmAt;     // Time of automatic disarm when "Don't spin the motors when armed" is enabled and auto_disarm_delay is nonzero

	uint16_t filteredCycleTime;

	struct ninja_rc_input rc_input;

	struct fastloop_output fout;

	struct pt	state_ctrl;

	struct pt	state_arming;
	sys_millis_t arming_delay;
	sys_millis_t disarm_timeout;

	bool is_armed;
	uint32_t sensors;

	struct fastloop *fastloop;
	//sys_micros_t loop_time;

	struct ninja_sched sched;

	//! this is used for directly controlling motors while we are disarmed (from gcs)
	//uint16_t direct_outputs[MIXER_OUTPUT_COUNT];

	//uint8_t armingFlags;
	//uint8_t stateFlags;
	//uint16_t flightModeFlags;
	//uint32_t enabledSensors;

	const struct system_calls *system;

	//! pointer to current configuration
	struct config_store *config_store;
	struct config *config;
};

void ninja_init(struct ninja *self, struct fastloop *fl, const struct system_calls *syscalls, struct config_store *config);

void ninja_arm(struct ninja *self);
void ninja_disarm(struct ninja *self);

uint32_t ninja_has_sensors(struct ninja *self, sensor_mask_t sensor_mask);
bool ninja_is_armed(struct ninja *self);

void ninja_heartbeat(struct ninja *self);
void ninja_input_rc(struct ninja *self, const struct ninja_rc_input *rc);
void ninja_input_gyro(struct ninja *self, int32_t x, int32_t y, int32_t z);
void ninja_input_acc(struct ninja *self, int32_t x, int32_t y, int32_t z);

