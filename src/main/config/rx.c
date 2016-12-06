#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "rx.h"

static const char rcChannelLetters[] = "AERT12345678abcdefgh";

char rx_config_channel_letter(uint8_t ch){
	if(ch >= 20) return 'x';
	return rcChannelLetters[ch];
}

void rx_config_set_mapping(struct rx_config *self, const char *input){
	(void)self;
	const char *c, *s;

	for (c = input; *c; c++) {
		s = strchr(rcChannelLetters, *c);
		if (s && (s < rcChannelLetters + RX_MAX_MAPPABLE_RX_INPUTS))
			self->rcmap[s - rcChannelLetters] = c - input;
	}
}

