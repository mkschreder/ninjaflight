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

#define RX_MAX_PPM_CHANNELS 12
#define MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT 8

#define MAX_AUX_CHANNEL_COUNT (RX_MAX_SUPPORTED_RC_CHANNELS - RX_NON_AUX_CHANNEL_COUNT)

#if MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT > RX_MAX_PPM_CHANNELS
#define MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT
#else
#define MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT RX_MAX_PPM_CHANNELS
#endif
#define RX_FAILSAFE_MODE_COUNT 3

typedef enum {
    RX_FAILSAFE_TYPE_FLIGHT = 0,
    RX_FAILSAFE_TYPE_AUX,
} rxFailsafeChannelType_e;

#define RX_FAILSAFE_TYPE_COUNT 2
typedef struct rxRuntimeConfig_s {
    uint8_t channelCount;                  // number of rc channels as reported by current input driver
} rxRuntimeConfig_t;

#define RSSI_ADC_SAMPLE_COUNT 16
#define RX_CHANNEL_TIMEOUT    300
#define PPM_AND_PWM_SAMPLE_COUNT 3

typedef uint16_t (*rcReadRawDataPtr)(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan);        // used by receiver driver to return channel data

typedef enum {
	RX_PWM,
	RX_PPM,
	RX_MSP,
	RX_SERIAL,
	RX_SERIAL_SPEKTRUM1024 = RX_SERIAL,
	RX_SERIAL_SPEKTRUM2048,
	RX_SERIAL_SBUS,
	RX_SERIAL_SUMD,
	RX_SERIAL_SUMH,
	RX_SERIAL_XBUS_MODE_B,
	RX_SERIAL_XBUS_MODE_B_RJ01,
	RX_SERIAL_IBUS
} rx_type_t;

struct rx {
	rx_type_t rx_type;

	uint16_t rssi;                  //!< rssi in range: [0;1023]

	//! time for next update (we only update as often as is necessary depending on what type of receiver we have)
	sys_micros_t rxUpdateAt;

	//! suspends rx signal until specified deadline
	sys_micros_t suspendRxSignalUntil;
	uint8_t  skipRxSamples;

	//! samples collected from the receiver.
	uint16_t rcSamples[MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT][PPM_AND_PWM_SAMPLE_COUNT];
	bool rxSamplesCollected;
	uint8_t rcSampleIndex;

	//! period under which we have not had valid rx signal
	sys_micros_t rcInvalidPulsPeriod[RX_MAX_SUPPORTED_RC_CHANNELS];
	//! calculated output channels
	int16_t rcData[RX_MAX_SUPPORTED_RC_CHANNELS];     //! interval [1000;2000]
	//! holds currently active channels (until they time out)
	uint32_t active_channels;
	//! holds mask of channels that have been active since receiver was connected
	uint32_t used_channels;

	//! function used for reading raw values. Returns 0 on failure to read receiver.
	uint16_t (*rcReadRawFunc)(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan);        // used by receiver driver to return channel data

	//! currently set refresh rate in microseconds
	uint16_t rxRefreshRate;

	#if defined(USE_ADC)
	uint8_t adcRssiSamples[RSSI_ADC_SAMPLE_COUNT];
	uint8_t adcRssiSampleIndex;
	uint32_t rssiUpdateAt;
	#endif

	rxRuntimeConfig_t rxRuntimeConfig;

	const struct system_calls *system;
};

void rx_update(struct rx *self);

//! RX has signal if at least one channel is healthy
bool rx_has_signal(struct rx *self);

//! RX is healthy if all channels are healthy
bool rx_is_healthy(struct rx *self);

//! Returns true if channels 0-3 are active
bool rx_flight_channels_valid(struct rx *self);

//! Configures the rx using the specified channel layout and config. (channel layout example: AETR1234)
void rx_remap_channels(struct rx *self, const char *input);

void rx_init(struct rx *self, const struct system_calls *system);

// TODO: make it possible to switch type at runtime (involves sorting out some init things)
// currently it is only possible to set the type once
//! changes the receiver type (determines what system calls the rx system will use to get receiver data)
void rx_set_type(struct rx *self, rx_type_t type);

void rx_update_rssi(struct rx *self);
void rx_reset_ranges(struct rx *self, rxChannelRangeConfiguration_t *rxChannelRangeConfiguration);

void rx_suspend_signal(struct rx *self);
void rx_resume_signal(struct rx *self);

uint16_t rx_get_refresh_rate(struct rx *self);

uint16_t rx_get_rssi(struct rx *self);

uint8_t rx_get_channel_count(struct rx *self);
int16_t rx_get_channel(struct rx *self, uint8_t chan);

//! Forces a channel to a certain value. Used by failsafe.
//void rx_set_channel(struct rx *self, uint8_t chan, int16_t value);

