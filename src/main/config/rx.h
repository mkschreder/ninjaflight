/*
 * This file is part of Ninjaflight.
 *
 * Copyright: collective.
 * Cleanup: Martin Schröder <mkschreder.uk@gmail.com>
 * Original source from Cleanflight.
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

//! maximum number of inputs that we can remap
#define RX_MAX_MAPPABLE_RX_INPUTS 8
//! maximum supported channels when calling rx_get_channel
#define RX_MAX_SUPPORTED_RC_CHANNELS (18)
//! number of channels that are treated as "essential" for basic control
#define RX_NON_AUX_CHANNEL_COUNT 4

#define RSSI_SCALE_MIN 1
#define RSSI_SCALE_MAX 255
#define RSSI_SCALE_DEFAULT 30

#define PWM_RANGE_ZERO 0 // FIXME should all usages of this be changed to use PWM_RANGE_MIN?
#define PWM_RANGE_MIN 1000
#define PWM_RANGE_MAX 2000
#define PWM_RANGE_MIDDLE (PWM_RANGE_MIN + ((PWM_RANGE_MAX - PWM_RANGE_MIN) / 2))

#define PWM_PULSE_MIN   750       // minimum PWM pulse width which is considered valid
#define PWM_PULSE_MAX   2250      // maximum PWM pulse width which is considered valid

#define RXFAIL_STEP_TO_CHANNEL_VALUE(step) (PWM_PULSE_MIN + 25 * step)
#define CHANNEL_VALUE_TO_RXFAIL_STEP(channelValue) ((constrain(channelValue, PWM_PULSE_MIN, PWM_PULSE_MAX) - PWM_PULSE_MIN) / 25)
#define MAX_RXFAIL_RANGE_STEP ((PWM_PULSE_MAX - PWM_PULSE_MIN) / 25)

typedef enum {
    RX_FAILSAFE_MODE_AUTO = 0,
    RX_FAILSAFE_MODE_HOLD,
    RX_FAILSAFE_MODE_SET,
    RX_FAILSAFE_MODE_INVALID,
} rxFailsafeChannelMode_e;

typedef enum rc_alias {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4,
    AUX5,
    AUX6,
    AUX7,
    AUX8
} rc_alias_e;

struct rx_failsafe_chan_config {
    uint8_t mode; // See rxFailsafeChannelMode_e
    uint8_t step;
};

struct rx_channel_range_config {
    uint16_t min;
    uint16_t max;
};

struct rx_output_config {
	struct rx_channel_range_config range[RX_NON_AUX_CHANNEL_COUNT];
	struct rx_failsafe_chan_config failsafe[RX_MAX_SUPPORTED_RC_CHANNELS];
};

struct rx_config {
    uint8_t rcmap[RX_MAX_MAPPABLE_RX_INPUTS];  //!< mapping of radio channels to internal RPYTA+ order
    uint8_t serialrx_provider;              //!< type of UART-based receiver (0 = spek 10, 1 = spek 11, 2 = sbus). Must be enabled by FEATURE_RX_SERIAL first.
    uint8_t sbus_inversion;                 //!< default sbus (Futaba, FrSKY) is inverted. Support for uninverted OpenLRS (and modified FrSKY) receivers.
    uint8_t spektrum_sat_bind;              //!< number of bind pulses for Spektrum satellite receivers
    uint8_t rssi_channel;					//!< rssi channel number
    uint8_t rssi_scale;						//!< scale of the rssi signal. Range [RSSI_SCALE_MIN;RSSI_SCALE_MAX]
    uint8_t rssi_ppm_invert;				//!< if set then rssi signal is treated such that high value means low rssi
    uint8_t rcSmoothing;                    //!< Enable/Disable RC filtering
    uint16_t midrc;                         //!< Some radios have not a neutral point centered on 1500. can be changed here
    uint16_t mincheck;                      //!< minimum rc end
    uint16_t maxcheck;                      //!< maximum rc end

    uint16_t rx_min_usec;					//!< minimum value of rx pulse from receiver (below this point we consider signal being corrupt)
    uint16_t rx_max_usec;					//!< maximum value of rx pulse from receiver (above this point the signal is invalid and will trigger failsafe)
};

//! Configures the rx using the specified channel layout and config. (channel layout example: AETR1234)
void rx_config_set_mapping(struct rx_config *self, const char *input);
char rx_config_channel_letter(uint8_t ch);


