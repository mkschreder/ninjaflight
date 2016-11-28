/*
 * This file is part of Ninjaflight.
 *
 * Copyright 2013-2016 Cleanflight project
 * Copyright 2016 Martin Schröder
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

/**
 * @defgroup RX Receiver
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <string.h>

#include <platform.h>

#include "system_calls.h"
#include "build_config.h"
#include "debug.h"

#include "../common/maths.h"
#include "../common/utils.h"

#include "../io/rc_controls.h"

#include "../drivers/pwm_rx.h"
#include "../drivers/adc.h"
#include "../drivers/system.h"

#include "pwm.h"
#include "sbus.h"
#include "spektrum.h"
#include "sumd.h"
#include "sumh.h"
#include "msp.h"
#include "xbus.h"
#include "ibus.h"

#include "rx.h"

//#define DEBUG_RX_SIGNAL_LOSS
#define DELAY_50_HZ (1000000 / 50)
#define DELAY_10_HZ (1000000 / 10)
#define DELAY_5_HZ (1000000 / 5)
#define SKIP_RC_ON_SUSPEND_PERIOD 1500000		   // 1.5 second period in usec (call frequency independent)
#define SKIP_RC_SAMPLES_ON_RESUME  2				// flush 2 samples to drop wrong measurements (timing independent)
#define REQUIRED_CHANNEL_MASK 0x0F // first 4 channels

static uint16_t nullReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t channel); 

static const char rcChannelLetters[] = "AERT12345678abcdefgh";

char rx_get_channel_letter(uint8_t ch){
	if(ch >= 20) return 'x';
	return rcChannelLetters[ch];
}

static uint16_t nullReadRawRC(rxRuntimeConfig_t *rconf, uint8_t channel)
{
	UNUSED(rconf);
	UNUSED(channel);

	return PPM_RCVR_TIMEOUT;
}

void serialRxInit(rxConfig_t *rxConfig);

bool isPulseValid(uint16_t pulseDuration){
	return  pulseDuration >= rxConfig()->rx_min_usec &&
			pulseDuration <= rxConfig()->rx_max_usec;
}

static uint16_t applyRxChannelRangeConfiguraton(int sample, rxChannelRangeConfiguration_t *range){
	// Avoid corruption of channel with a value of PPM_RCVR_TIMEOUT
	if (sample == PPM_RCVR_TIMEOUT) {
		return PPM_RCVR_TIMEOUT;
	}

	sample = scaleRange(sample, range->min, range->max, PWM_RANGE_MIN, PWM_RANGE_MAX);
	sample = constrain(sample, PWM_PULSE_MIN, PWM_PULSE_MAX);

	return sample;
}

//! Returns either previously read rc value or failsafe defaults for each channel
static uint16_t _get_failsafe_channel_value(struct rx *self, uint8_t channel){
	rxFailsafeChannelConfig_t *failsafeChannelConfig = failsafeChannelConfigs(channel);
	uint8_t mode = failsafeChannelConfig->mode;

	// force auto mode to prevent fly away when failsafe stage 2 is disabled
	if ( channel < RX_NON_AUX_CHANNEL_COUNT) {
		mode = RX_FAILSAFE_MODE_AUTO;
	}

	switch(mode) {
		case RX_FAILSAFE_MODE_AUTO:
			switch (channel) {
				case THROTTLE:
					return PWM_RANGE_MIN;
				default:
					return rxConfig()->midrc;
			}
			/* no break */
		default:
		case RX_FAILSAFE_MODE_INVALID:
		case RX_FAILSAFE_MODE_HOLD:
			return self->rcData[channel];
		case RX_FAILSAFE_MODE_SET:
			return RXFAIL_STEP_TO_CHANNEL_VALUE(failsafeChannelConfig->step);
	}

	// if we get here then we return midrc because this is where we would get also when we first start up
	return rxConfig()->midrc;
}
void rx_init(struct rx *self, const struct system_calls *system){
	memset(self, 0, sizeof(struct rx));

	self->system = system;

	// at startup the receiver is in failsafe mode until some samples are received
	self->rcReadRawFunc = nullReadRawRC;

	// TODO: does this mean we always reset the mapping
	rx_remap_channels(self, "AERT1234");

	self->rcSampleIndex = 0;

	for (int i = 0; i < RX_MAX_SUPPORTED_RC_CHANNELS; i++) {
		// this is necessary since failsafe could be in HOLD mode
		if(i == THROTTLE) self->rcData[THROTTLE] = PWM_RANGE_MIN;
		else self->rcData[i] = rxConfig()->midrc; 
		// now get the failsafe value which could be the same value as we set above
		self->rcData[i] = _get_failsafe_channel_value(self, i);;
		self->rcInvalidPulsPeriod[i] = sys_millis(self->system) + RX_CHANNEL_TIMEOUT;
	}

	// Initialize ARM switch to OFF position when arming via switch is defined
	// TODO: this should be moved to rc controls logic it simply does not belong here
	/*
	for (i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
		modeActivationCondition_t *modeActivationCondition = &modeActivationConditions[i];
		if (modeActivationCondition->modeId == BOXARM && IS_RANGE_USABLE(&modeActivationCondition->range)) {
			// ARM switch is defined, determine an OFF value
			if (modeActivationCondition->range.startStep > 0) {
				value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationCondition->range.startStep - 1));
			} else {
				value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationCondition->range.endStep + 1));
			}
			// Initialize ARM AUX channel to OFF value
			self->rcData[modeActivationCondition->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = value;
		}
	}
	*/
}

/**
 * @page RX
 * @ingroup RX
 * A receiver is used to receive radio control signals from your transmitter
 * and convert them into signals that the flight controller can understand.
 *
 * There are 3 basic types of receivers:
 *
 * - Parallel PWM Receivers
 * - PPM Receivers
 * - Serial Receivers
 * 
 * As of 2016 the recommendation for new purchases is a Serial or PPM based
 * receiver. Avoid Parallel PWM recievers (1 wire per channel). This is due to
 * the amount of IO pins parallel PWM based receivers use. Some new FC's do not
 * support parallel PWM.
 */

/**
 * Sets receiver type.
 */
void rx_set_type(struct rx *self, rx_type_t type){
	bool enabled = false;
	uint8_t t = type;
	switch(t){
		case RX_PWM:
			self->rxRefreshRate = 20000;
			enabled = rxPwmInit(&self->system->pwm, &self->rxRuntimeConfig, &self->rcReadRawFunc);
			break;
		case RX_PPM:
			self->rxRefreshRate = 20000;
			enabled = rxPPMInit(&self->system->pwm, &self->rxRuntimeConfig, &self->rcReadRawFunc);
			break;
		case RX_MSP:
			self->rxRefreshRate = 20000;
			enabled = rxMspInit(&self->rxRuntimeConfig, &self->rcReadRawFunc);
			break;
#ifdef SERIAL_RX
		case RX_SERIAL_SPEKTRUM1024:
			self->rxRefreshRate = 22000;
			enabled = spektrumInit(&self->rxRuntimeConfig, &self->rcReadRawFunc);
			break;
		case RX_SERIAL_SPEKTRUM2048:
			self->rxRefreshRate = 11000;
			enabled = spektrumInit(&self->rxRuntimeConfig, &self->rcReadRawFunc);
			break;
		case RX_SERIAL_SBUS:
			self->rxRefreshRate = 11000;
			enabled = sbusInit(&self->rxRuntimeConfig, &self->rcReadRawFunc);
			break;
		case RX_SERIAL_SUMD:
			self->rxRefreshRate = 11000;
			enabled = sumdInit(&self->rxRuntimeConfig, &self->rcReadRawFunc);
			break;
		case RX_SERIAL_SUMH:
			self->rxRefreshRate = 11000;
			enabled = sumhInit(&self->rxRuntimeConfig, &self->rcReadRawFunc);
			break;
		case RX_SERIAL_XBUS_MODE_B:
		case RX_SERIAL_XBUS_MODE_B_RJ01:
			self->rxRefreshRate = 11000;
			enabled = xBusInit(&self->rxRuntimeConfig, &self->rcReadRawFunc);
			break;
		case RX_SERIAL_IBUS:
			enabled = ibusInit(&self->rxRuntimeConfig, &self->rcReadRawFunc);
			break;
#endif
		default:break;
	}

	if (!enabled) {
		self->rcReadRawFunc = nullReadRawRC;
	}
}

uint8_t serialRxFrameStatus(struct rx *self){
	/**
	 * FIXME: Each of the xxxxFrameStatus() methods MUST be able to survive being called without the
	 * corresponding xxxInit() method having been called first.
	 *
	 * This situation arises when the cli or the msp changes the value of rxConfig->serialrx_provider
	 *
	 * A solution is for the ___Init() to configure the serialRxFrameStatus function pointer which
	 * should be used instead of the switch statement below.
	 */

	 uint8_t type = self->rx_type;
	 switch (type) {
#ifdef SERIAL_RX
		case RX_SERIAL_SPEKTRUM1024:
		case RX_SERIAL_SPEKTRUM2048:
			return spektrumFrameStatus();
		case RX_SERIAL_SBUS:
			return sbusFrameStatus();
		case RX_SERIAL_SUMD:
			return sumdFrameStatus();
		case RX_SERIAL_SUMH:
			return sumhFrameStatus();
		case RX_SERIAL_XBUS_MODE_B:
		case RX_SERIAL_XBUS_MODE_B_RJ01:
			return xBusFrameStatus();
		case RX_SERIAL_IBUS:
			return ibusFrameStatus();
#endif
		default:break;
	}
	return SERIAL_RX_FRAME_PENDING;
}

static uint8_t calculateChannelRemapping(uint8_t *channelMap, uint8_t channelMapEntryCount, uint8_t channelToRemap){
	if (channelToRemap < channelMapEntryCount) {
		return channelMap[channelToRemap];
	}
	return channelToRemap;
}

bool rx_has_signal(struct rx *self){
	return self->active_channels != 0;
}

bool rx_is_healthy(struct rx *self){
	return self->used_channels != 0 && self->active_channels == self->used_channels;
}

bool rx_flight_channels_valid(struct rx *self){
	return self->active_channels & 0xf;
}

void rx_suspend_signal(struct rx *self){
	self->suspendRxSignalUntil = sys_micros(self->system) + SKIP_RC_ON_SUSPEND_PERIOD;
	self->skipRxSamples = SKIP_RC_SAMPLES_ON_RESUME;
	// TODO: reimplement this elsewhere
	//failsafe_on_rx_suspend(self->failsafe, SKIP_RC_ON_SUSPEND_PERIOD);
}

void rx_resume_signal(struct rx *self){
	self->suspendRxSignalUntil = sys_micros(self->system);
	self->skipRxSamples = SKIP_RC_SAMPLES_ON_RESUME;
	// TODO: reimplement this elsewhere
	//failsafe_on_rx_resume(self->failsafe);
}


static void _read_all_channels(struct rx *self){
	// we support at most 32 channels. This is because certain masks are 32 bit etc. So we need to ensure this.
	int chan_count = MIN(MIN(self->rxRuntimeConfig.channelCount, RX_MAX_SUPPORTED_RC_CHANNELS), 32);

	uint32_t valid = 0;
	// try to read all channels and return if we fail
	for (int channel = 0; channel < chan_count; channel++) {
		uint8_t id = calculateChannelRemapping(rxConfig()->rcmap, ARRAYLEN(rxConfig()->rcmap), channel);

		// sample should be 0 if channel could not be read
		uint16_t sample = self->rcReadRawFunc(&self->rxRuntimeConfig, id);

		sys_millis_t milli_time = sys_millis(self->system);
		if (!isPulseValid(sample)) {
			// hold the signal for a little while before resorting to signal failure
			if (milli_time >= self->rcInvalidPulsPeriod[channel]) {
				// if any channel has timed out then put receiver into failsafe mode
				self->rcData[channel] = _get_failsafe_channel_value(self, channel);   // after that apply rxfail value
				// update timeout such that we never overflow when in this state
				self->rcInvalidPulsPeriod[channel] = milli_time + RX_CHANNEL_TIMEOUT;
				// invalid channels mean we do not have signal but only if the channel has been healthy before
				if(self->active_channels & (1 << channel)) {
					self->active_channels &= ~(1 << channel);
				}
			}
		} else {
			// apply the rx calibration
			if (channel < RX_NON_AUX_CHANNEL_COUNT) {
				self->rcData[channel] = applyRxChannelRangeConfiguraton(sample, channelRanges(channel));
			} else {
				self->rcData[channel] = sample;
			}

			self->rcInvalidPulsPeriod[channel] = milli_time + RX_CHANNEL_TIMEOUT;
			
			valid |= (1 << channel);
		}
	}
	// add channels that have been valid during this frame to the valid mask
	self->used_channels |= valid;
	self->active_channels |= valid;
}

/*
static uint16_t calculateNonDataDrivenChannel(struct rx *self, uint8_t chan, uint16_t sample)
{
	uint8_t currentSampleIndex = self->rcSampleIndex % PPM_AND_PWM_SAMPLE_COUNT;

	// update the recent samples and compute the average of them
	self->rcSamples[chan][currentSampleIndex] = sample;

	// avoid returning an incorrect average which would otherwise occur before enough samples
	if (!self->rxSamplesCollected) {
		if (self->rcSampleIndex < PPM_AND_PWM_SAMPLE_COUNT) {
			return sample;
		}
		self->rxSamplesCollected = true;
	}

	uint16_t rcDataMean = 0;
	uint8_t sampleIndex;
	for (sampleIndex = 0; sampleIndex < PPM_AND_PWM_SAMPLE_COUNT; sampleIndex++)
		rcDataMean += self->rcSamples[chan][sampleIndex];

	return rcDataMean / PPM_AND_PWM_SAMPLE_COUNT;
}
*/
static void _read_channels(struct rx *self){
	sys_micros_t currentTime = sys_micros(self->system);
	self->rxUpdateAt = currentTime + DELAY_50_HZ;

	// only proceed when no more samples to skip and suspend period is over
	if (self->skipRxSamples) {
		if (currentTime > self->suspendRxSignalUntil) {
			self->skipRxSamples--;
		}
		return;
	}

	_read_all_channels(self);

	self->rcSampleIndex++;
}

void rx_update(struct rx *self){
	_read_channels(self);
}

void rx_remap_channels(struct rx *self, const char *input){
	(void)self;
	const char *c, *s;

	for (c = input; *c; c++) {
		s = strchr(rcChannelLetters, *c);
		if (s && (s < rcChannelLetters + RX_MAX_MAPPABLE_RX_INPUTS))
			rxConfig()->rcmap[s - rcChannelLetters] = c - input;
	}
}

static void updateRSSIPWM(struct rx *self){
	int16_t pwmRssi = 0;
	uint8_t chan = constrain(rxConfig()->rssi_channel - 1, 0, RX_MAX_SUPPORTED_RC_CHANNELS);

	// Read value of AUX channel as rssi
	pwmRssi = self->rcData[chan];

	// RSSI_Invert option
	if (rxConfig()->rssi_ppm_invert) {
		pwmRssi = ((PWM_RANGE_MAX - pwmRssi) + PWM_RANGE_MIN);
	}

	// Range of rawPwmRssi is [1000;2000]. rssi should be in [0;1023];
	self->rssi = (uint16_t)((constrain(pwmRssi - PWM_RANGE_MIN, 0, PWM_RANGE_MIN) / 1000.0f) * 1023.0f);
}
/*
// TODO: rssi adc
static void updateRSSIADC(struct rx *self)
{
#ifndef USE_ADC
	(void)self;
#else
	sys_micros_t currentTime = sys_micros(self->system);
	if ((int32_t)(currentTime - rssiUpdateAt) < 0) {
		return;
	}
	self->rssiUpdateAt = currentTime + DELAY_50_HZ;

	int16_t adcRssiMean = 0;
	uint16_t adcRssiSample = adcGetChannel(ADC_RSSI);
	uint8_t rssiPercentage = adcRssiSample / rxConfig()->rssi_scale;

	self->adcRssiSampleIndex = (adcRssiSampleIndex + 1) % RSSI_ADC_SAMPLE_COUNT;

	self->adcRssiSamples[adcRssiSampleIndex] = rssiPercentage;

	uint8_t sampleIndex;

	for (sampleIndex = 0; sampleIndex < RSSI_ADC_SAMPLE_COUNT; sampleIndex++) {
		adcRssiMean += adcRssiSamples[sampleIndex];
	}

	adcRssiMean = adcRssiMean / RSSI_ADC_SAMPLE_COUNT;

	self->rssi = (uint16_t)((constrain(adcRssiMean, 0, 100) / 100.0f) * 1023.0f);
#endif
}
*/

void rx_update_rssi(struct rx *self){
	if (rxConfig()->rssi_channel > 0) {
		updateRSSIPWM(self);
	} 
	// TODO: rssi adc
	/*else if (feature(FEATURE_RSSI_ADC)) {
		updateRSSIADC(self);
	}
	*/
}

uint16_t rx_get_refresh_rate(struct rx *self){
	return self->rxRefreshRate;
}

uint16_t rx_get_rssi(struct rx *self){
	return self->rssi; 
}

uint8_t rx_get_channel_count(struct rx *self){
	return self->rxRuntimeConfig.channelCount;
}

//! Get channel value. Returns interval [rx_min_usec;rx_max_usec]
int16_t rx_get_channel(struct rx *self, uint8_t chan){
	if(chan >= RX_MAX_SUPPORTED_RC_CHANNELS) return PWM_RANGE_MIN; 
	return self->rcData[chan];
}

