#pragma once

#include "rc_controls.h"

typedef struct adjustmentRange_s {
	// when aux channel is in range...
	uint8_t auxChannelIndex;
	channelRange_t range;

	// ..then apply the adjustment function to the auxSwitchChannel ...
	uint8_t adjustmentFunction;
	uint8_t auxSwitchChannelIndex;

	// ... via slot
	uint8_t adjustmentIndex;
} adjustmentRange_t;

#define MAX_ADJUSTMENT_RANGE_COUNT 12 // enough for 2 * 6pos switches.

typedef struct adjustmentProfile_s {
    adjustmentRange_t adjustmentRanges[MAX_ADJUSTMENT_RANGE_COUNT];
} adjustmentProfile_t;

PG_DECLARE_PROFILE(adjustmentProfile_t, adjustmentProfile);

