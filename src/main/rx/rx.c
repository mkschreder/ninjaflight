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

#include "../config/feature.h"

#include "../flight/failsafe.h"

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

const char rcChannelLetters[] = "AERT12345678abcdefgh";

static uint16_t nullReadRawRC(rxRuntimeConfig_t *rconf, uint8_t channel)
{
	UNUSED(rconf);
	UNUSED(channel);

	return PPM_RCVR_TIMEOUT;
}



void serialRxInit(rxConfig_t *rxConfig);


void rx_flight_chans_reset(struct rx *self)
{
	self->validFlightChannelMask = REQUIRED_CHANNEL_MASK;
}

bool rx_flight_chans_valid(struct rx *self)
{
	return (self->validFlightChannelMask == REQUIRED_CHANNEL_MASK);
}

bool isPulseValid(uint16_t pulseDuration){
	return  pulseDuration >= rxConfig()->rx_min_usec &&
			pulseDuration <= rxConfig()->rx_max_usec;
}

// pulse duration is in micro seconds (usec)
// TODO: make this static after refactoring unit tests
void rx_flight_chans_update(struct rx *self, uint8_t channel, bool valid)
{
	if (channel < NON_AUX_CHANNEL_COUNT && !valid) {
		// if signal is invalid - mark channel as BAD
		self->validFlightChannelMask &= ~(1 << channel);
	}
}

void rx_init(struct rx *self, const struct system_calls *system, struct failsafe *failsafe, modeActivationCondition_t *modeActivationConditions){
	uint8_t i;
	uint16_t value;

	self->rxIsInFailsafeMode = true;
	self->rxIsInFailsafeModeNotDataDriven = true;
	self->system = system;
	self->failsafe = failsafe;
	self->rcReadRawFunc = nullReadRawRC;

	rx_set_config(self, "AETR1234", rxConfig());

	self->rcSampleIndex = 0;

	for (i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
		self->rcData[i] = rxConfig()->midrc;
		self->rcInvalidPulsPeriod[i] = sys_millis(self->system) + MAX_INVALID_PULS_TIME;
	}

	self->rcData[THROTTLE] = (feature(FEATURE_3D)) ? rxConfig()->midrc : rxConfig()->rx_min_usec;

	// Initialize ARM switch to OFF position when arming via switch is defined
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

#ifdef SERIAL_RX
	if (feature(FEATURE_RX_SERIAL)) {
		serialRxInit(rxConfig());
	}
#endif

	if (feature(FEATURE_RX_MSP)) {
		self->rxRefreshRate = 20000;
		rxMspInit(&self->rxRuntimeConfig, &self->rcReadRawFunc);
	}

	if (feature(FEATURE_RX_PPM) || feature(FEATURE_RX_PARALLEL_PWM)) {
		self->rxRefreshRate = 20000;
		rxPwmInit(&self->system->pwm, &self->rxRuntimeConfig, &self->rcReadRawFunc);
	}
}

#ifdef SERIAL_RX
void serialRxInit(struct rx *self, rxConfig_t *rxConfig){
	bool enabled = false;
	switch (rxConfig->serialrx_provider) {
		case SERIALRX_SPEKTRUM1024:
			self->rxRefreshRate = 22000;
			enabled = spektrumInit(&self->rxRuntimeConfig, &rcReadRawFunc);
			break;
		case SERIALRX_SPEKTRUM2048:
			self->rxRefreshRate = 11000;
			enabled = spektrumInit(&self->rxRuntimeConfig, &rcReadRawFunc);
			break;
		case SERIALRX_SBUS:
			self->rxRefreshRate = 11000;
			enabled = sbusInit(&self->rxRuntimeConfig, &rcReadRawFunc);
			break;
		case SERIALRX_SUMD:
			self->rxRefreshRate = 11000;
			enabled = sumdInit(&self->rxRuntimeConfig, &rcReadRawFunc);
			break;
		case SERIALRX_SUMH:
			self->rxRefreshRate = 11000;
			enabled = sumhInit(&self->rxRuntimeConfig, &rcReadRawFunc);
			break;
		case SERIALRX_XBUS_MODE_B:
		case SERIALRX_XBUS_MODE_B_RJ01:
			self->rxRefreshRate = 11000;
			enabled = xBusInit(&self->rxRuntimeConfig, &rcReadRawFunc);
			break;
		case SERIALRX_IBUS:
			enabled = ibusInit(&self->rxRuntimeConfig, &rcReadRawFunc);
			break;
		default:break;
	}

	if (!enabled) {
		featureClear(FEATURE_RX_SERIAL);
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
	switch (rxConfig()->serialrx_provider) {
		case SERIALRX_SPEKTRUM1024:
		case SERIALRX_SPEKTRUM2048:
			return spektrumFrameStatus();
		case SERIALRX_SBUS:
			return sbusFrameStatus();
		case SERIALRX_SUMD:
			return sumdFrameStatus();
		case SERIALRX_SUMH:
			return sumhFrameStatus();
		case SERIALRX_XBUS_MODE_B:
		case SERIALRX_XBUS_MODE_B_RJ01:
			return xBusFrameStatus();
		case SERIALRX_IBUS:
			return ibusFrameStatus();
		default:break;
	}
	return SERIAL_RX_FRAME_PENDING;
}
#endif

static uint8_t calculateChannelRemapping(uint8_t *channelMap, uint8_t channelMapEntryCount, uint8_t channelToRemap){
	if (channelToRemap < channelMapEntryCount) {
		return channelMap[channelToRemap];
	}
	return channelToRemap;
}

bool rx_is_receiving(struct rx *self)
{
	return self->rxSignalReceived;
}

bool rx_flight_channels_valid(struct rx *self)
{
	return self->rxFlightChannelsValid;
}
static bool isRxDataDriven(void)
{
	return !(feature(FEATURE_RX_PARALLEL_PWM | FEATURE_RX_PPM));
}

static void resetRxSignalReceivedFlagIfNeeded(struct rx *self, uint32_t currentTime)
{
	if (!self->rxSignalReceived) {
		return;
	}

	if (((int32_t)(currentTime - self->needRxSignalBefore) >= 0)) {
		self->rxSignalReceived = false;
		self->rxSignalReceivedNotDataDriven = false;
	}
}

void rx_suspend_signal(struct rx *self){
	self->suspendRxSignalUntil = sys_micros(self->system) + SKIP_RC_ON_SUSPEND_PERIOD;
	self->skipRxSamples = SKIP_RC_SAMPLES_ON_RESUME;
	failsafe_on_rx_suspend(self->failsafe, SKIP_RC_ON_SUSPEND_PERIOD);
}

void rx_resume_signal(struct rx *self){
	self->suspendRxSignalUntil = sys_micros(self->system);
	self->skipRxSamples = SKIP_RC_SAMPLES_ON_RESUME;
	failsafe_on_rx_resume(self->failsafe);
}

void rx_update(struct rx *self, uint32_t currentTime){
	resetRxSignalReceivedFlagIfNeeded(self, currentTime);

	if (isRxDataDriven()) {
		self->rxDataReceived = false;
	}


#ifdef SERIAL_RX
	if (feature(FEATURE_RX_SERIAL)) {
		uint8_t frameStatus = serialRxFrameStatus();

		if (frameStatus & SERIAL_RX_FRAME_COMPLETE) {
			self->rxDataReceived = true;
			rxIsInFailsafeMode = (frameStatus & SERIAL_RX_FRAME_FAILSAFE) != 0;
			rxSignalReceived = !rxIsInFailsafeMode;
			needRxSignalBefore = currentTime + DELAY_10_HZ;
		}
	}
#endif

	if (feature(FEATURE_RX_MSP)) {
		self->rxDataReceived = rxMspFrameComplete();

		if (self->rxDataReceived) {
			self->rxSignalReceived = true;
			self->rxIsInFailsafeMode = false;
			self->needRxSignalBefore = currentTime + DELAY_5_HZ;
		}
	}

	if (feature(FEATURE_RX_PPM)) {
		if (isPPMDataBeingReceived()) {
			self->rxSignalReceivedNotDataDriven = true;
			self->rxIsInFailsafeModeNotDataDriven = false;
			self->needRxSignalBefore = currentTime + DELAY_10_HZ;
			resetPPMDataReceivedState();
		}
	}

	if (feature(FEATURE_RX_PARALLEL_PWM)) {
		if (isPWMDataBeingReceived()) {
			self->rxSignalReceivedNotDataDriven = true;
			self->rxIsInFailsafeModeNotDataDriven = false;
			self->needRxSignalBefore = currentTime + DELAY_10_HZ;
		}
	}

}

bool rx_data_received(struct rx *self, uint32_t currentTime)
{
	return self->rxDataReceived || ((int32_t)(currentTime - self->rxUpdateAt) >= 0); // data driven or 50Hz
}

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

static uint16_t getRxfailValue(struct rx *self, uint8_t channel)
{
	rxFailsafeChannelConfig_t *failsafeChannelConfig = failsafeChannelConfigs(channel);
	uint8_t mode = failsafeChannelConfig->mode;

	// force auto mode to prevent fly away when failsafe stage 2 is disabled
	if ( channel < NON_AUX_CHANNEL_COUNT && (!feature(FEATURE_FAILSAFE)) ) {
		mode = RX_FAILSAFE_MODE_AUTO;
	}

	switch(mode) {
		case RX_FAILSAFE_MODE_AUTO:
			switch (channel) {
				case ROLL:
				case PITCH:
				case YAW:
					return rxConfig()->midrc;

				case THROTTLE:
					if (feature(FEATURE_3D))
						return rxConfig()->midrc;
					else
						return rxConfig()->rx_min_usec;
				default:break;
			}
			/* no break */

		default:
		case RX_FAILSAFE_MODE_INVALID:
		case RX_FAILSAFE_MODE_HOLD:
			return self->rcData[channel];

		case RX_FAILSAFE_MODE_SET:
			return RXFAIL_STEP_TO_CHANNEL_VALUE(failsafeChannelConfig->step);
	}
	// TODO: this may not be correct
	return self->rcData[channel];
}

// TODO: make this static after refactoring unit tests
uint16_t applyRxChannelRangeConfiguraton(int sample, rxChannelRangeConfiguration_t *range);
uint16_t applyRxChannelRangeConfiguraton(int sample, rxChannelRangeConfiguration_t *range)
{
	// Avoid corruption of channel with a value of PPM_RCVR_TIMEOUT
	if (sample == PPM_RCVR_TIMEOUT) {
		return PPM_RCVR_TIMEOUT;
	}

	sample = scaleRange(sample, range->min, range->max, PWM_RANGE_MIN, PWM_RANGE_MAX);
	sample = MIN(MAX(PWM_PULSE_MIN, sample), PWM_PULSE_MAX);

	return sample;
}

static void readRxChannelsApplyRanges(struct rx *self)
{
	uint8_t channel;

	for (channel = 0; channel < self->rxRuntimeConfig.channelCount; channel++) {

		uint8_t rawChannel = calculateChannelRemapping(rxConfig()->rcmap, ARRAYLEN(rxConfig()->rcmap), channel);

		// sample the channel
		uint16_t sample = self->rcReadRawFunc(&self->rxRuntimeConfig, rawChannel);

		// apply the rx calibration
		if (channel < NON_AUX_CHANNEL_COUNT) {
			sample = applyRxChannelRangeConfiguraton(sample, channelRanges(channel));
		}

		self->rcRaw[channel] = sample;
	}
}

static void detectAndApplySignalLossBehaviour(struct rx *self)
{
	int channel;
	uint16_t sample;
	bool useValueFromRx = true;
	bool rxIsDataDriven = isRxDataDriven();
	uint32_t currentMilliTime = sys_millis(self->system);

	if (!rxIsDataDriven) {
		self->rxSignalReceived = self->rxSignalReceivedNotDataDriven;
		self->rxIsInFailsafeMode = self->rxIsInFailsafeModeNotDataDriven;
	}

	if (!self->rxSignalReceived || self->rxIsInFailsafeMode) {
		useValueFromRx = false;
	}

#ifdef DEBUG_RX_SIGNAL_LOSS
	debug[0] = rxSignalReceived;
	debug[1] = rxIsInFailsafeMode;
	debug[2] = rcReadRawFunc(&rxRuntimeConfig, 0);
#endif

	rx_flight_chans_reset(self);

	for (channel = 0; channel < self->rxRuntimeConfig.channelCount; channel++) {

		sample = (useValueFromRx) ? self->rcRaw[channel] : PPM_RCVR_TIMEOUT;

		bool validPulse = isPulseValid(sample);

		if (!validPulse) {
			if (currentMilliTime < self->rcInvalidPulsPeriod[channel]) {
				sample = self->rcData[channel];		   // hold channel for MAX_INVALID_PULS_TIME
			} else {
				sample = getRxfailValue(self, channel);   // after that apply rxfail value
				rx_flight_chans_update(self, channel, validPulse);
			}
		} else {
			self->rcInvalidPulsPeriod[channel] = currentMilliTime + MAX_INVALID_PULS_TIME;
		}

		if (rxIsDataDriven) {
			self->rcData[channel] = sample;
		} else {
			self->rcData[channel] = calculateNonDataDrivenChannel(self, channel, sample);
		}
	}

	self->rxFlightChannelsValid = rx_flight_chans_valid(self);

	if ((self->rxFlightChannelsValid) && !(rcModeIsActive(BOXFAILSAFE) && feature(FEATURE_FAILSAFE))) {
		failsafe_on_valid_data_received(self->failsafe);
	} else {
		self->rxIsInFailsafeMode = self->rxIsInFailsafeModeNotDataDriven = true;
		failsafe_on_valid_data_failed(self->failsafe);

		for (channel = 0; channel < self->rxRuntimeConfig.channelCount; channel++) {
			self->rcData[channel] = getRxfailValue(self, channel);
		}
	}

#ifdef DEBUG_RX_SIGNAL_LOSS
	debug[3] = self->rcData[THROTTLE];
#endif
}

void rx_recalc_channels(struct rx *self, uint32_t currentTime)
{
	self->rxUpdateAt = currentTime + DELAY_50_HZ;

	// only proceed when no more samples to skip and suspend period is over
	if (self->skipRxSamples) {
		if (currentTime > self->suspendRxSignalUntil) {
			self->skipRxSamples--;
		}
		return;
	}

	readRxChannelsApplyRanges(self);
	detectAndApplySignalLossBehaviour(self);

	self->rcSampleIndex++;
}

void rx_set_config(struct rx *self, const char *input, rxConfig_t *rxConfig){
	(void)self;
	const char *c, *s;

	for (c = input; *c; c++) {
		s = strchr(rcChannelLetters, *c);
		if (s && (s < rcChannelLetters + MAX_MAPPABLE_RX_INPUTS))
			rxConfig->rcmap[s - rcChannelLetters] = c - input;
	}
}

static void updateRSSIPWM(struct rx *self)
{
	int16_t pwmRssi = 0;
	// Read value of AUX channel as rssi
	pwmRssi = self->rcData[rxConfig()->rssi_channel - 1];
	
	// RSSI_Invert option	
	if (rxConfig()->rssi_ppm_invert) {
		pwmRssi = ((2000 - pwmRssi) + 1000);
	}
	
	// Range of rawPwmRssi is [1000;2000]. rssi should be in [0;1023];
	self->rssi = (uint16_t)((constrain(pwmRssi - 1000, 0, 1000) / 1000.0f) * 1023.0f);
}

static void updateRSSIADC(struct rx *self, uint32_t currentTime)
{
#ifndef USE_ADC
	(void)self;
	UNUSED(currentTime);
#else
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

void rx_update_rssi(struct rx *self, uint32_t currentTime){
	if (rxConfig()->rssi_channel > 0) {
		updateRSSIPWM(self);
	} else if (feature(FEATURE_RSSI_ADC)) {
		updateRSSIADC(self, currentTime);
	}
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

// returns interval 1000:2000
int16_t rx_get_channel(struct rx *self, uint8_t chan){
	if(chan >= MAX_SUPPORTED_RC_CHANNEL_COUNT) return 1000; 
	return self->rcData[chan]; 
}

void rx_set_channel(struct rx *self, uint8_t chan, int16_t value){
	if(chan >= MAX_SUPPORTED_RC_CHANNEL_COUNT) return;
	self->rcData[chan] = value;
}
