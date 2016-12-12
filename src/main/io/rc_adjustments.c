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

#include <math.h>

#include <platform.h>

#include "system_calls.h"
#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/system.h"

#include "config/config.h"
#include "config/feature.h"

#include "io/beeper.h"

#include "ninja.h"
#include "io/rc_adjustments.h"

#define MARK_ADJUSTMENT_FUNCTION_AS_BUSY(adjustmentIndex) self->adjustmentStateMask |= (1 << adjustmentIndex)
#define MARK_ADJUSTMENT_FUNCTION_AS_READY(adjustmentIndex) self->adjustmentStateMask &= ~(1 << adjustmentIndex)

#define IS_ADJUSTMENT_FUNCTION_BUSY(adjustmentIndex) (self->adjustmentStateMask & (1 << adjustmentIndex))

static void blackboxLogInflightAdjustmentEvent(adjustmentFunction_e adjustmentFunction, int32_t newValue)
{
#ifndef BLACKBOX
    UNUSED(adjustmentFunction);
    UNUSED(newValue);
#else
    if (feature(FEATURE_BLACKBOX)) {
        flightLogEvent_inflightAdjustment_t eventData;
        eventData.adjustmentFunction = adjustmentFunction;
        eventData.newValue = newValue;
        eventData.floatFlag = false;
        blackboxLogEvent(FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT, (flightLogEventData_t*)&eventData);
    }
#endif
}

// TODO: this one is abused elsewhere. Stop the abuse. 
void blackboxLogInflightAdjustmentEventFloat(adjustmentFunction_e adjustmentFunction, float newFloatValue); 
void blackboxLogInflightAdjustmentEventFloat(adjustmentFunction_e adjustmentFunction, float newFloatValue)
{
#ifndef BLACKBOX
    UNUSED(adjustmentFunction);
    UNUSED(newFloatValue);
#else
    if (feature(FEATURE_BLACKBOX)) {
        flightLogEvent_inflightAdjustment_t eventData;
        eventData.adjustmentFunction = adjustmentFunction;
        eventData.newFloatValue = newFloatValue;
        eventData.floatFlag = true;
        blackboxLogEvent(FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT, (flightLogEventData_t*)&eventData);
    }
#endif
}

void rc_adj_add_range(struct rc_adj *self, adjustmentRange_t *adjustmentRange){
    uint8_t index = constrain(adjustmentRange->adjustmentIndex, 0, MAX_SIMULTANEOUS_ADJUSTMENT_COUNT);

    adjustmentState_t *adjustmentState = &self->adjustmentStates[index];

    if (adjustmentState->range == adjustmentRange) {
        return;
    }
    adjustmentState->range = adjustmentRange;
    adjustmentState->auxChannelIndex = adjustmentRange->auxSwitchChannelIndex;
    adjustmentState->timeoutAt = 0;

    adjustmentState->config.adjustmentFunction = adjustmentRange->adjustmentFunction;

    if (adjustmentRange->adjustmentFunction == ADJUSTMENT_RATE_PROFILE) {
        adjustmentState->config.mode = ADJUSTMENT_MODE_SELECT;
        adjustmentState->config.data.selectConfig.switchPositions = 3;
    } else {
        adjustmentState->config.mode = ADJUSTMENT_MODE_STEP;
        adjustmentState->config.data.stepConfig.step = 1;
    }

    MARK_ADJUSTMENT_FUNCTION_AS_READY(index);
}

static void setAdjustment(uint8_t* ptr, uint8_t adjustment, int delta, uint8_t min, uint8_t max)
{
    *ptr = constrain((int)(*ptr)+delta, min ,max);
    blackboxLogInflightAdjustmentEvent(adjustment, *ptr);
}

static void applyStepAdjustment(struct rc_adj *self, struct rate_profile *controlRateConfig, uint8_t adjustmentFunction, int delta){
	struct pid_config *pid = &config_get_profile_rw(self->config)->pid;
    if (delta > 0) {
        beeper_write(&self->ninja->beeper, "I");
    } else {
        beeper_write(&self->ninja->beeper, "E");
    }
    switch(adjustmentFunction) {
        case ADJUSTMENT_RC_RATE:
            setAdjustment(&controlRateConfig->rcRate8,ADJUSTMENT_RC_RATE,delta,RC_RATE_MIN,RC_RATE_MAX);
            //generatePitchRollCurve();
            break;
        case ADJUSTMENT_RC_EXPO:
            setAdjustment(&controlRateConfig->rcExpo8,ADJUSTMENT_RC_EXPO,delta,EXPO_MIN,EXPO_MAX);
            //generatePitchRollCurve();
            break;
        case ADJUSTMENT_THROTTLE_EXPO:
            setAdjustment(&controlRateConfig->thrExpo8,ADJUSTMENT_THROTTLE_EXPO,delta,EXPO_MIN,EXPO_MAX);
            //generateThrottleCurve();
            break;
        case ADJUSTMENT_PITCH_ROLL_RATE:
        case ADJUSTMENT_PITCH_RATE:
            setAdjustment(&controlRateConfig->rates[PITCH],ADJUSTMENT_PITCH_RATE,delta,0,CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX);
            if (adjustmentFunction == ADJUSTMENT_PITCH_RATE) {
                break;
            }
            // follow though for combined ADJUSTMENT_PITCH_ROLL_RATE
        case ADJUSTMENT_ROLL_RATE:
            setAdjustment(&controlRateConfig->rates[ROLL],ADJUSTMENT_ROLL_RATE,delta,0,CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX);
            break;
        case ADJUSTMENT_YAW_RATE:
            setAdjustment(&controlRateConfig->rates[YAW],ADJUSTMENT_YAW_RATE,delta,0,CONTROL_RATE_CONFIG_YAW_RATE_MAX);
            break;
        case ADJUSTMENT_PITCH_ROLL_P:
        case ADJUSTMENT_PITCH_P:
            setAdjustment(&pid->P8[PIDPITCH],ADJUSTMENT_PITCH_P,delta,PID_MIN,PID_MAX);
            if (adjustmentFunction == ADJUSTMENT_PITCH_P) {
                break;
            }
            // follow though for combined ADJUSTMENT_PITCH_ROLL_P
        case ADJUSTMENT_ROLL_P:
            setAdjustment(&pid->P8[PIDROLL],ADJUSTMENT_ROLL_P,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_PITCH_ROLL_I:
        case ADJUSTMENT_PITCH_I:
            setAdjustment(&pid->I8[PIDPITCH],ADJUSTMENT_PITCH_I,delta,PID_MIN,PID_MAX);
            if (adjustmentFunction == ADJUSTMENT_PITCH_I) {
                break;
            }
            // follow though for combined ADJUSTMENT_PITCH_ROLL_I
        case ADJUSTMENT_ROLL_I:
           setAdjustment(&pid->I8[PIDROLL],ADJUSTMENT_ROLL_I,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_PITCH_ROLL_D:
        case ADJUSTMENT_PITCH_D:
            setAdjustment(&pid->D8[PIDPITCH],ADJUSTMENT_PITCH_D,delta,PID_MIN,PID_MAX);
            if (adjustmentFunction == ADJUSTMENT_PITCH_D) {
                break;
            }
            // follow though for combined ADJUSTMENT_PITCH_ROLL_D
        case ADJUSTMENT_ROLL_D:
            setAdjustment(&pid->D8[PIDROLL],ADJUSTMENT_ROLL_D,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_YAW_P:
                setAdjustment(&pid->P8[PIDYAW],ADJUSTMENT_YAW_P,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_YAW_I:
            setAdjustment(&pid->I8[PIDYAW],ADJUSTMENT_YAW_I,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_YAW_D:
                setAdjustment(&pid->D8[PIDYAW],ADJUSTMENT_YAW_D,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_LEVEL_P:
            setAdjustment(&pid->P8[PIDLEVEL],ADJUSTMENT_LEVEL_P,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_LEVEL_I:
            setAdjustment(&pid->I8[PIDLEVEL],ADJUSTMENT_LEVEL_I,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_LEVEL_D:
            setAdjustment(&pid->D8[PIDLEVEL],ADJUSTMENT_LEVEL_D,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_ALT_P:
            setAdjustment(&pid->P8[PIDALT],ADJUSTMENT_ALT_P,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_ALT_I:
            setAdjustment(&pid->I8[PIDALT],ADJUSTMENT_ALT_I,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_ALT_D:
            setAdjustment(&pid->D8[PIDALT],ADJUSTMENT_ALT_D,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_POS_P:
            setAdjustment(&pid->P8[PIDPOS],ADJUSTMENT_POS_P,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_POS_I:
            setAdjustment(&pid->I8[PIDPOS],ADJUSTMENT_POS_I,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_POSR_P:
            setAdjustment(&pid->P8[PIDPOSR],ADJUSTMENT_POSR_P,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_POSR_I:
            setAdjustment(&pid->I8[PIDPOSR],ADJUSTMENT_POSR_I,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_POSR_D:
            setAdjustment(&pid->D8[PIDPOSR],ADJUSTMENT_POSR_D,delta,PID_MIN,PID_MAX);
            break;
       case ADJUSTMENT_NAVR_P:
            setAdjustment(&pid->P8[PIDNAVR],ADJUSTMENT_NAVR_P,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_NAVR_I:
            setAdjustment(&pid->I8[PIDNAVR],ADJUSTMENT_NAVR_I,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_NAVR_D:
            setAdjustment(&pid->D8[PIDNAVR],ADJUSTMENT_NAVR_D,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_MAG_P:
            setAdjustment(&pid->P8[PIDMAG],ADJUSTMENT_MAG_P,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_VEL_P:
            setAdjustment(&pid->P8[PIDVEL],ADJUSTMENT_VEL_P,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_VEL_I:
            setAdjustment(&pid->I8[PIDVEL],ADJUSTMENT_VEL_I,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_VEL_D:
            setAdjustment(&pid->D8[PIDVEL],ADJUSTMENT_VEL_D,delta,PID_MIN,PID_MAX);
            break;
        default:
            break;
    };
}

static void applySelectAdjustment(struct rc_adj *self, uint8_t adjustmentFunction, uint8_t position)
{
    bool applied = false;

    switch(adjustmentFunction) {
        case ADJUSTMENT_RATE_PROFILE:
            /*if (getCurrentControlRateProfile() != position) {
                changeControlRateProfile(position);
                blackboxLogInflightAdjustmentEvent(ADJUSTMENT_RATE_PROFILE, position);
                applied = true;
            }*/
            break;
		default:
			break;
    }

	char buf[16];
	snprintf(buf, sizeof(buf), "%d", position + 1);

    if (applied) {
        beeper_write(&self->ninja->beeper, buf);
    }
}

#define RESET_FREQUENCY_2HZ (1000 / 2)

void rc_adj_update(struct rc_adj *self){
    uint8_t adjustmentIndex;
    uint32_t now = sys_millis(self->ninja->system);

    bool canUseRxData = rx_has_signal(&self->ninja->rx);

    for (adjustmentIndex = 0; adjustmentIndex < MAX_SIMULTANEOUS_ADJUSTMENT_COUNT; adjustmentIndex++) {
        adjustmentState_t *adjustmentState = &self->adjustmentStates[adjustmentIndex];

        uint8_t adjustmentFunction = adjustmentState->config.adjustmentFunction;
        if (adjustmentFunction == ADJUSTMENT_NONE) {
            continue;
        }

        int32_t signedDiff = now - adjustmentState->timeoutAt;
        bool canResetReadyStates = signedDiff >= 0L;

        if (canResetReadyStates) {
            adjustmentState->timeoutAt = now + RESET_FREQUENCY_2HZ;
            MARK_ADJUSTMENT_FUNCTION_AS_READY(adjustmentIndex);
        }

        if (!canUseRxData) {
            continue;
        }

        uint8_t channelIndex = RX_NON_AUX_CHANNEL_COUNT + adjustmentState->auxChannelIndex;
		int16_t rcin = rx_get_channel(&self->ninja->rx, channelIndex); 

        if (adjustmentState->config.mode == ADJUSTMENT_MODE_STEP) {
            int delta;
            if (rcin > self->config->rx.midrc + 200) {
                delta = adjustmentState->config.data.stepConfig.step;
            } else if (rcin < self->config->rx.midrc - 200) {
                delta = 0 - adjustmentState->config.data.stepConfig.step;
            } else {
                // returning the switch to the middle immediately resets the ready state
                MARK_ADJUSTMENT_FUNCTION_AS_READY(adjustmentIndex);
                adjustmentState->timeoutAt = now + RESET_FREQUENCY_2HZ;
                continue;
            }
            if (IS_ADJUSTMENT_FUNCTION_BUSY(adjustmentIndex)) {
                continue;
            }

            applyStepAdjustment(self, config_get_rate_profile_rw(self->config), adjustmentFunction, delta);
        } else if (adjustmentState->config.mode == ADJUSTMENT_MODE_SELECT) {
            uint16_t rangeWidth = ((2100 - 900) / adjustmentState->config.data.selectConfig.switchPositions);
            uint8_t position = (constrain(rcin, 900, 2100 - 1) - 900) / rangeWidth;

            applySelectAdjustment(self, adjustmentFunction, position);
        }
        MARK_ADJUSTMENT_FUNCTION_AS_BUSY(adjustmentIndex);
    }
}

void rc_adj_update_states(struct rc_adj *self, adjustmentRange_t *adjustmentRanges){
	(void)self;
	(void)adjustmentRanges;
    uint8_t index;

    for (index = 0; index < MAX_ADJUSTMENT_RANGE_COUNT; index++) {
/*
        adjustmentRange_t *adjustmentRange = &adjustmentRanges[index];
		// TODO: RC adjustment ranges
        if (isRangeActive(&self->ninja->rx, adjustmentRange->auxChannelIndex, &adjustmentRange->range)) {

            rc_adj_add_range(self, adjustmentRange);
        }
		*/
    }
}

void rc_adj_init(struct rc_adj *self, struct ninja *ninja, struct config *config){
	memset(self, 0, sizeof(*self));
	self->ninja = ninja;
	self->config = config;
}

void rc_adj_reset(struct rc_adj *self){
    memset(self->adjustmentStates, 0, sizeof(self->adjustmentStates));
}
