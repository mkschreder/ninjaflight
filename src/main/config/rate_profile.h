#pragma once

#define MAX_CONTROL_RATE_PROFILE_COUNT 3

struct rate_config {
    uint8_t rcRate8;
    uint8_t rcExpo8;
    uint8_t thrMid8;
    uint8_t thrExpo8;
    uint8_t rates[3];
    uint8_t dynThrPID;
    uint8_t rcYawExpo8;
    uint16_t tpa_breakpoint;                // Breakpoint at which TPA is activated
};

struct rate_profile_selection {
    uint8_t defaultRateProfileIndex;
};

PG_DECLARE_ARR(struct rate_config, MAX_CONTROL_RATE_PROFILE_COUNT, controlRateProfiles);
PG_DECLARE_PROFILE(struct rate_profile_selection, rateProfileSelection);

