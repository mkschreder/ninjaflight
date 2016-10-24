#include <stdint.h>

#include "rx/rx.h"

#include "rx.h"

static int16_t rcData[8]; 

uint16_t rc_get_rssi(void){
	return 0; 
}

// returns interval 1000:2000
int16_t rc_get_channel_value(uint8_t chan){
	if(chan >= MAX_SUPPORTED_RC_CHANNEL_COUNT) return 1000; 
	return rcData[chan]; 
}

