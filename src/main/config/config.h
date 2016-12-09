/*
 * This file is part of Ninjaflight.
 *
 * Ninjaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Ninjaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Ninjaflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

/**
 * @defgroup config Configuration
 * @{
 */

#include <stdint.h>
#include <stdbool.h>


#include "accelerometer.h"
#include "altitudehold.h"
#include "anglerate.h"
#include "blackbox.h"
#include "battery.h"
#include "compass.h"
#include "failsafe.h"
#include "gps.h"
#include "imu.h"
#include "mixer.h"
#include "navigation.h"
#include "rate_profile.h"
#include "sensors.h"
#include "tilt.h"
#include "gimbal.h"
#include "rx.h"
#include "rc_controls.h"
#include "rc_adjustments.h"
#include "transponder.h"
#include "boardalignment.h"
#include "ledstrip.h"
#include "frsky.h"
#include "hott.h"
#include "barometer.h"
#include "gyro.h"
#include "pwm_rx.h"
#include "gtune.h"
#include "telemetry.h"
#include "serial.h"
#include "profile.h"
#include "system.h"
#include "feature.h"

#define MAX_PROFILE_COUNT 1
#define ONESHOT_FEATURE_CHANGED_DELAY_ON_BOOT_MS 1500

struct config_profile {
	struct pid_config pid;
	struct accelerometer_config acc;
	struct mag_config mag;
	struct barometer_config baro;
	struct gimbal_config gimbal;
	struct gtune_config gtune;
	struct throttle_correction_config throttle;
	struct servo_profile servos;
	struct gps_profile gps;
	struct rate_profile_selection rate;
	struct rc_adjustment_profile rc_adj;
	struct rc_function_profile rc_funcs;
	struct rc_controls_config rc;
} __attribute__((packed)) ;

struct config {
	struct profile_config profile;
	struct gyro_config gyro;
	struct config_profile profiles[MAX_PROFILE_COUNT];
	struct airplane_althold_config airplane_althold;
	struct battery_config bat;
	struct blackbox_config blackbox;
	struct board_alignment_config alignment;
	struct system_config system;
	struct failsafe_config failsafe;
	struct feature_config feature;
	struct frsky_telemetry_config frsky;
	struct hott_telemetry_config hott;
	struct gps_config gps;
	struct imu_config imu;
	struct ledstrip_config ledstrip;
	struct mixer_config mixer;
	struct motor_3d_config motor_3d;
	struct pwm_output_config pwm_out;
	struct pwm_input_config pwm_in;
	struct rate_config rate;
	struct arming_config arm;
	struct rx_config rx;
	struct rx_output_config rx_output;
	struct sensor_config sensors;
	struct serial_config serial;
	struct telemetry_config telemetry;
	struct tilt_config tilt;
	struct transponder_config transponder;
} __attribute__((packed,aligned(2))) ;

//! structure used for storing config in the eeprom
struct config_store {
	struct config data;
	uint16_t	crc;
} __attribute__((aligned(4)));

struct system_calls;

struct config_profile const * config_get_profile(const struct config * const self);
struct config_profile *config_get_profile_rw(struct config *self);
struct rate_profile const * config_get_rate_profile(const struct config * const self);
struct rate_profile *config_get_rate_profile_rw(struct config *self);

int config_save(const struct config *self, const struct system_calls *system);
int config_load(struct config *self, const struct system_calls *system);
void config_reset(struct config *self);
bool config_fixup(struct config *config);

/** @} */
