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

#include <stdbool.h>
#include "config/parameter_group.h"
#include "../config/rc_controls.h"
#include "target.h"
#include "system_calls.h"

#include "../config/rx.h"

#ifndef DEFAULT_RX_FEATURE
#define DEFAULT_RX_FEATURE FEATURE_RX_PARALLEL_PWM
#endif

#define STICK_CHANNEL_COUNT 4

#define PWM_RANGE_ZERO 0 // FIXME should all usages of this be changed to use PWM_RANGE_MIN?
#define PWM_RANGE_MIN 1000
#define PWM_RANGE_MAX 2000
#define PWM_RANGE_MIDDLE (PWM_RANGE_MIN + ((PWM_RANGE_MAX - PWM_RANGE_MIN) / 2))

#define PWM_PULSE_MIN   750       // minimum PWM pulse width which is considered valid
#define PWM_PULSE_MAX   2250      // maximum PWM pulse width which is considered valid

#define RXFAIL_STEP_TO_CHANNEL_VALUE(step) (PWM_PULSE_MIN + 25 * step)
#define CHANNEL_VALUE_TO_RXFAIL_STEP(channelValue) ((constrain(channelValue, PWM_PULSE_MIN, PWM_PULSE_MAX) - PWM_PULSE_MIN) / 25)
#define MAX_RXFAIL_RANGE_STEP ((PWM_PULSE_MAX - PWM_PULSE_MIN) / 25)

#define DEFAULT_SERVO_MIN 1000
#define DEFAULT_SERVO_MIDDLE 1500
#define DEFAULT_SERVO_MAX 2000
#define DEFAULT_SERVO_MIN_ANGLE 45
#define DEFAULT_SERVO_MAX_ANGLE 45

typedef enum {
    SERIAL_RX_FRAME_PENDING = 0,
    SERIAL_RX_FRAME_COMPLETE = (1 << 0),
    SERIAL_RX_FRAME_FAILSAFE = (1 << 1)
} serialrxFrameState_t;

typedef enum {
    SERIALRX_SPEKTRUM1024 = 0,
    SERIALRX_SPEKTRUM2048 = 1,
    SERIALRX_SBUS = 2,
    SERIALRX_SUMD = 3,
    SERIALRX_SUMH = 4,
    SERIALRX_XBUS_MODE_B = 5,
    SERIALRX_XBUS_MODE_B_RJ01 = 6,
    SERIALRX_IBUS = 7,
    SERIALRX_PROVIDER_MAX = SERIALRX_IBUS
} SerialRXType;

#define SERIALRX_PROVIDER_COUNT (SERIALRX_PROVIDER_MAX + 1)

#define MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT 12
#define MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT 8

#define MAX_AUX_CHANNEL_COUNT (MAX_SUPPORTED_RC_CHANNEL_COUNT - NON_AUX_CHANNEL_COUNT)

#if MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT > MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT
#define MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT
#else
#define MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT
#endif

#define RSSI_SCALE_MIN 1
#define RSSI_SCALE_MAX 255
#define RSSI_SCALE_DEFAULT 30

#define RX_FAILSAFE_MODE_COUNT 3

typedef enum {
    RX_FAILSAFE_TYPE_FLIGHT = 0,
    RX_FAILSAFE_TYPE_AUX,
} rxFailsafeChannelType_e;

#define RX_FAILSAFE_TYPE_COUNT 2
typedef struct rxRuntimeConfig_s {
    uint8_t channelCount;                  // number of rc channels as reported by current input driver
} rxRuntimeConfig_t;

typedef uint16_t (*rcReadRawDataPtr)(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan);        // used by receiver driver to return channel data

#define RSSI_ADC_SAMPLE_COUNT 16
#define MAX_INVALID_PULS_TIME    300
#define PPM_AND_PWM_SAMPLE_COUNT 3

struct rx {
	uint16_t rssi;                  // range: [0;1023]

	bool rxDataReceived;
	bool rxSignalReceived;
	bool rxSignalReceivedNotDataDriven;
	bool rxFlightChannelsValid;
	bool rxIsInFailsafeMode;
	bool rxIsInFailsafeModeNotDataDriven;

	uint32_t rxUpdateAt;
	uint32_t needRxSignalBefore;
	uint32_t suspendRxSignalUntil;
	uint8_t  skipRxSamples;

	int16_t rcRaw[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]
	uint32_t rcInvalidPulsPeriod[MAX_SUPPORTED_RC_CHANNEL_COUNT];
	int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]

	rcReadRawDataPtr rcReadRawFunc;
	uint16_t rxRefreshRate;

	#if defined(USE_ADC)
	uint8_t adcRssiSamples[RSSI_ADC_SAMPLE_COUNT];
	uint8_t adcRssiSampleIndex;
	uint32_t rssiUpdateAt;
	#endif

	uint16_t rcSamples[MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT][PPM_AND_PWM_SAMPLE_COUNT];
	bool rxSamplesCollected;

	uint8_t rcSampleIndex;

	rxRuntimeConfig_t rxRuntimeConfig;

	uint8_t validFlightChannelMask;

	// TODO: this is a circular dependency so it must be removed
	struct failsafe *failsafe;
	const struct system_calls *system;
};

void rx_update(struct rx *self, uint32_t currentTime);
bool rx_is_receiving(struct rx *self);
bool rx_data_received(struct rx *self, uint32_t currentTime);
void rx_recalc_channels(struct rx *self, uint32_t currentTime);

bool rx_flight_chans_valid(struct rx *self);
void rx_flight_chans_reset(struct rx *self);
void rx_flight_chans_update(struct rx *self, uint8_t channel, bool valid);

//! Configures the rx using the specified channel layout and config. (channel layout example: AETR1234)
void rx_set_config(struct rx *self, const char *input, rxConfig_t *rxConfig);
uint8_t serialRxFrameStatus(void);

void rx_init(struct rx *self, const struct system_calls *system, struct failsafe *failsafe, modeActivationCondition_t *modeActivationConditions); 
void rx_update_rssi(struct rx *self, uint32_t currentTime);
void rx_reset_ranges(struct rx *self, rxChannelRangeConfiguration_t *rxChannelRangeConfiguration);

void rx_suspend_signal(struct rx *self);
void rx_resume_signal(struct rx *self);

uint16_t rx_get_refresh_rate(struct rx *self);

uint16_t rx_get_rssi(struct rx *self);

uint8_t rx_get_channel_count(struct rx *self);
int16_t rx_get_channel(struct rx *self, uint8_t chan);

//! Forces a channel to a certain value. Used by failsafe.
void rx_set_channel(struct rx *self, uint8_t chan, int16_t value);

