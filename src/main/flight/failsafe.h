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

#include "system_calls.h"
#include "../config/failsafe.h"

#define FAILSAFE_POWER_ON_DELAY_US (1000 * 1000 * 5)
#define MILLIS_PER_TENTH_SECOND      100
#define MILLIS_PER_SECOND           1000
#define PERIOD_OF_1_SECONDS            1 * MILLIS_PER_SECOND
#define PERIOD_OF_3_SECONDS            3 * MILLIS_PER_SECOND
#define PERIOD_OF_30_SECONDS          30 * MILLIS_PER_SECOND
#define PERIOD_RXDATA_FAILURE        200    // millis
#define PERIOD_RXDATA_RECOVERY       200    // millis

typedef enum {
    FAILSAFE_IDLE = 0,
    FAILSAFE_RX_LOSS_DETECTED,
    FAILSAFE_LANDING,
    FAILSAFE_LANDED,
    FAILSAFE_RX_LOSS_MONITORING,
    FAILSAFE_RX_LOSS_RECOVERED
} failsafePhase_e;

typedef enum {
    FAILSAFE_RXLINK_DOWN = 0,
    FAILSAFE_RXLINK_UP
} failsafeRxLinkState_e;

struct failsafe {
    int16_t events;
    bool monitoring;
    bool active;
    uint32_t rxDataFailurePeriod;
    uint32_t validRxDataReceivedAt;
    uint32_t validRxDataFailedAt;
    sys_millis_t throttleLowPeriod;             // throttle stick must have been below 'min_check' for this period
    sys_millis_t landingShouldBeFinishedAt;
    sys_millis_t receivingRxDataPeriod;         // period for the required period of valid rxData
    sys_millis_t  receivingRxDataPeriodPreset;   // preset for the required period of valid rxData
    failsafePhase_e phase;
    failsafeRxLinkState_e rxLinkState;

	struct ninja *ninja;
	const struct config const *config;
};

void failsafe_init(struct failsafe *self, struct ninja *ninja, const struct config const *config);

void failsafe_start_monitoring(struct failsafe *self);
void failsafe_update(struct failsafe *self);

void failsafe_reset(struct failsafe *self);
failsafePhase_e failsafe_get_state(struct failsafe *self);
bool failsafe_is_monitoring(struct failsafe *self);
bool failsafe_is_active(struct failsafe *self);
bool failsafe_is_receiving_rx(struct failsafe *self);
void failsafe_on_rx_suspend(struct failsafe *self, uint32_t suspendPeriod);
void failsafe_on_rx_resume(struct failsafe *self);

void failsafe_on_valid_data_received(struct failsafe *self);
void failsafe_on_valid_data_failed(struct failsafe *self);
