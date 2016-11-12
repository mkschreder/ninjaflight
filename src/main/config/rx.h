#pragma once

#include "parameter_group.h"

#define MAX_MAPPABLE_RX_INPUTS 8
#define MAX_SUPPORTED_RC_CHANNEL_COUNT (18)
#define NON_AUX_CHANNEL_COUNT 4

typedef enum {
    RX_FAILSAFE_MODE_AUTO = 0,
    RX_FAILSAFE_MODE_HOLD,
    RX_FAILSAFE_MODE_SET,
    RX_FAILSAFE_MODE_INVALID,
} rxFailsafeChannelMode_e;

typedef struct rxFailsafeChannelConfiguration_s {
    uint8_t mode; // See rxFailsafeChannelMode_e
    uint8_t step;
} rxFailsafeChannelConfig_t;

typedef struct rxChannelRangeConfiguration_s {
    uint16_t min;
    uint16_t max;
} rxChannelRangeConfiguration_t;

typedef struct rxConfig_s {
    uint8_t rcmap[MAX_MAPPABLE_RX_INPUTS];  // mapping of radio channels to internal RPYTA+ order
    uint8_t serialrx_provider;              // type of UART-based receiver (0 = spek 10, 1 = spek 11, 2 = sbus). Must be enabled by FEATURE_RX_SERIAL first.
    uint8_t sbus_inversion;                 // default sbus (Futaba, FrSKY) is inverted. Support for uninverted OpenLRS (and modified FrSKY) receivers.
    uint8_t spektrum_sat_bind;              // number of bind pulses for Spektrum satellite receivers
    uint8_t rssi_channel;
    uint8_t rssi_scale;
    uint8_t rssi_ppm_invert;
    uint8_t rcSmoothing;                    // Enable/Disable RC filtering
    uint16_t midrc;                         // Some radios have not a neutral point centered on 1500. can be changed here
    uint16_t mincheck;                      // minimum rc end
    uint16_t maxcheck;                      // maximum rc end

    uint16_t rx_min_usec;
    uint16_t rx_max_usec;
}  rxConfig_t;

PG_DECLARE(rxConfig_t, rxConfig);

PG_DECLARE_ARR(rxFailsafeChannelConfig_t, MAX_SUPPORTED_RC_CHANNEL_COUNT, failsafeChannelConfigs);
PG_DECLARE_ARR(rxChannelRangeConfiguration_t, NON_AUX_CHANNEL_COUNT, channelRanges);


