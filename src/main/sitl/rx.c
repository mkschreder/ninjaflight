#include <stdint.h>

#include "rx/rx.h"

#include "rx.h"

static int16_t rcData[8]; 
int16_t rcCommand[4]; 

uint16_t rc_get_rssi(void){
	return 0; 
}

// returns interval 1000:2000
int16_t rc_get_channel_value(uint8_t chan){
	if(chan >= MAX_SUPPORTED_RC_CHANNEL_COUNT) return 1000; 
	return rcData[chan]; 
}

// returns interval 1000:2000
void rc_set_channel_value(uint8_t chan, int16_t val){
	if(chan >= MAX_SUPPORTED_RC_CHANNEL_COUNT) return; 
	rcData[chan] = val;
	if(chan < 4)
	rcCommand[chan] = val - 1500;
}

