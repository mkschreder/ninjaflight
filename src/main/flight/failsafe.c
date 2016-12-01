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
#include <string.h>

#include <platform.h>

#include "debug.h"

#include "common/axis.h"

#include "config/parameter_group.h"

#include "drivers/system.h"

#include "rx/rx.h"

#include "io/beeper.h"

#include "config/parameter_group_ids.h"
#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_reset.h"

#include "flight/failsafe.h"
// TODO: this is not correct dependency wise
#include "../ninjaflight.h"

// TODO: failsafe
void failsafe_reset(struct failsafe *self){
	self->rxDataFailurePeriod = PERIOD_RXDATA_FAILURE + failsafeConfig()->failsafe_delay * MILLIS_PER_TENTH_SECOND;
	self->validRxDataReceivedAt = 0;
	self->validRxDataFailedAt = 0;
	self->throttleLowPeriod = 0;
	self->landingShouldBeFinishedAt = 0;
	self->receivingRxDataPeriod = 0;
	self->receivingRxDataPeriodPreset = 0;
	self->phase = FAILSAFE_IDLE;
	self->rxLinkState = FAILSAFE_RXLINK_DOWN;
}

void failsafe_init(struct failsafe *self, struct ninja *ninja){
	self->events = 0;
	self->monitoring = false;
	self->ninja = ninja;
	failsafe_reset(self);
	return;
}

failsafePhase_e failsafe_get_state(struct failsafe *self){
	return self->phase;
}

bool failsafe_is_monitoring(struct failsafe *self){
	return self->monitoring;
}

bool failsafe_is_active(struct failsafe *self){
	return self->active;
}

void failsafe_start_monitoring(struct failsafe *self){
	self->monitoring = true;
}

/*
static bool failsafeShouldHaveCausedLandingByNow(struct failsafe *self){
	return (sys_millis(self->ninja->system) > self->landingShouldBeFinishedAt);
}

static void failsafe_activate(struct failsafe *self){
	self->active = true;
	self->phase = FAILSAFE_LANDING;
	self->landingShouldBeFinishedAt = sys_millis(self->ninja->system) + failsafeConfig()->failsafe_off_delay * MILLIS_PER_TENTH_SECOND;

	self->events++;
}

static void failsafeApplyControlInput(struct failsafe *self){
	(void)self;
	// TODO: make failsafe work without interfering with the rx
	for (int i = 0; i < 3; i++) {
		//rx_set_channel(self->rx, i, rxConfig()->midrc);
	}
	//rx_set_channel(self->rx, THROTTLE, failsafeConfig()->failsafe_throttle);
}

*/
bool failsafe_is_receiving_rx(struct failsafe *self){
	return (self->rxLinkState == FAILSAFE_RXLINK_UP);
}

void failsafe_on_rx_suspend(struct failsafe *self, uint32_t usSuspendPeriod){
	self->validRxDataReceivedAt += (usSuspendPeriod / 1000);	// / 1000 to convert micros to millis
}

void failsafe_on_rx_resume(struct failsafe *self){
	self->validRxDataReceivedAt = sys_millis(self->ninja->system);					 // prevent RX link down trigger, restart rx link up
	self->rxLinkState = FAILSAFE_RXLINK_UP;					 // do so while rx link is up
}

void failsafe_on_valid_data_received(struct failsafe *self){
	self->validRxDataReceivedAt = sys_millis(self->ninja->system);
	if ((self->validRxDataReceivedAt - self->validRxDataFailedAt) > PERIOD_RXDATA_RECOVERY) {
		self->rxLinkState = FAILSAFE_RXLINK_UP;
	}
}

void failsafe_on_valid_data_failed(struct failsafe *self){
	self->validRxDataFailedAt = sys_millis(self->ninja->system);
	if ((self->validRxDataFailedAt - self->validRxDataReceivedAt) > self->rxDataFailurePeriod) {
		self->rxLinkState = FAILSAFE_RXLINK_DOWN;
	}
}

void failsafe_update(struct failsafe *self){
	(void)self;
	// TODO: failsafe
	/*
	if (!failsafe_is_monitoring(self)) {
		return;
	}

	bool receivingRxData = failsafe_is_receiving_rx(self);
	bool armed = self->ninja->is_armed;
	bool failsafeSwitchIsOn = rcModeIsActive(BOXFAILSAFE);
	beeper_command_t beeperMode = BEEPER_SILENCE;

	if (!receivingRxData) {
		beeperMode = BEEPER_RX_LOST;
	}

	bool reprocessState;

	do {
		reprocessState = false;

		switch (self->phase) {
			case FAILSAFE_IDLE:
				if (armed) {
					// Track throttle command below minimum time
					if (THROTTLE_HIGH == calculateThrottleStatus(self->rx, rxConfig(), rcControlsConfig()->deadband3d_throttle)) {
						self->throttleLowPeriod = sys_millis(self->system) + failsafeConfig()->failsafe_throttle_low_delay * MILLIS_PER_TENTH_SECOND;
					}
					// Kill switch logic (must be independent of receivingRxData to skip PERIOD_RXDATA_FAILURE delay before disarming)
					if (failsafeSwitchIsOn && failsafeConfig()->failsafe_kill_switch) {
						// KillswitchEvent: failsafe switch is configured as KILL switch and is switched ON
						failsafe_activate(self);
						self->phase = FAILSAFE_LANDED;	  // skip auto-landing procedure
						self->receivingRxDataPeriodPreset = PERIOD_OF_1_SECONDS;	// require 1 seconds of valid rxData
						reprocessState = true;
					} else if (!receivingRxData) {
						if (sys_millis(self->system) > self->throttleLowPeriod) {
							// JustDisarm: throttle was LOW for at least 'failsafe_throttle_low_delay' seconds
							failsafe_activate(self);
							self->phase = FAILSAFE_LANDED;	  // skip auto-landing procedure
							self->receivingRxDataPeriodPreset = PERIOD_OF_3_SECONDS; // require 3 seconds of valid rxData
						} else {
							self->phase = FAILSAFE_RX_LOSS_DETECTED;
						}
						reprocessState = true;
					}
				} else {
					// When NOT armed, show rxLinkState of failsafe switch in GUI (failsafe mode)
					if (failsafeSwitchIsOn) {
						ENABLE_FLIGHT_MODE(FAILSAFE_MODE);
					} else {
						DISABLE_FLIGHT_MODE(FAILSAFE_MODE);
					}
					// Throttle low period expired (= low long enough for JustDisarm)
					self->throttleLowPeriod = 0;
				}
				break;

			case FAILSAFE_RX_LOSS_DETECTED:
				if (receivingRxData) {
					self->phase = FAILSAFE_RX_LOSS_RECOVERED;
				} else {
					switch (failsafeConfig()->failsafe_procedure) {
						default:
						case FAILSAFE_PROCEDURE_AUTO_LANDING:
							// Stabilize, and set Throttle to specified level
							failsafe_activate(self);
							break;

						case FAILSAFE_PROCEDURE_DROP_IT:
							// Drop the craft
							failsafe_activate(self);
							self->phase = FAILSAFE_LANDED;	  // skip auto-landing procedure
							self->receivingRxDataPeriodPreset = PERIOD_OF_3_SECONDS; // require 3 seconds of valid rxData
							break;
					}
				}
				reprocessState = true;
				break;

			case FAILSAFE_LANDING:
				if (receivingRxData) {
					self->phase = FAILSAFE_RX_LOSS_RECOVERED;
					reprocessState = true;
				}
				if (armed) {
					failsafeApplyControlInput(self);
					beeperMode = BEEPER_RX_LOST_LANDING;
				}
				if (failsafeShouldHaveCausedLandingByNow(self) || !armed) {
					self->receivingRxDataPeriodPreset = PERIOD_OF_30_SECONDS; // require 30 seconds of valid rxData
					self->phase = FAILSAFE_LANDED;
					reprocessState = true;
				}
				break;

			case FAILSAFE_LANDED:
				ENABLE_ARMING_FLAG(PREVENT_ARMING); // To prevent accidently rearming by an intermittent rx link
				// TODO: fix this
				//mwDisarm();
				self->receivingRxDataPeriod = sys_millis(self->system) + self->receivingRxDataPeriodPreset; // set required period of valid rxData
				self->phase = FAILSAFE_RX_LOSS_MONITORING;
				reprocessState = true;
				break;

			case FAILSAFE_RX_LOSS_MONITORING:
				// Monitoring the rx link to allow rearming when it has become good for > `receivingRxDataPeriodPreset` time.
				if (receivingRxData) {
					if (sys_millis(self->system) > self->receivingRxDataPeriod) {
						// rx link is good now, when arming via ARM switch, it must be OFF first
						if (!(!isUsingSticksForArming() && rcModeIsActive(BOXARM))) {
							DISABLE_ARMING_FLAG(PREVENT_ARMING);
							self->phase = FAILSAFE_RX_LOSS_RECOVERED;
							reprocessState = true;
						}
					}
				} else {
					self->receivingRxDataPeriod = sys_millis(self->system) + self->receivingRxDataPeriodPreset;
				}
				break;

			case FAILSAFE_RX_LOSS_RECOVERED:
				// Entering IDLE with the requirement that throttle first must be at min_check for failsafe_throttle_low_delay period.
				// This is to prevent that JustDisarm is activated on the next iteration.
				// Because that would have the effect of shutting down failsafe handling on intermittent connections.
				self->throttleLowPeriod = sys_millis(self->system) + failsafeConfig()->failsafe_throttle_low_delay * MILLIS_PER_TENTH_SECOND;
				self->phase = FAILSAFE_IDLE;
				self->active = false;
				DISABLE_FLIGHT_MODE(FAILSAFE_MODE);
				reprocessState = true;
				break;

			default:
				break;
		}
	} while (reprocessState);

	if (beeperMode != BEEPER_SILENCE) {
		beeper(beeperMode);
	}
	*/
}
