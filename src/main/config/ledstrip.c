#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "ledstrip.h"

bool ledstrip_config_set_color(struct ledstrip_config *self, int index, const char *colorConfig){
	const char *remainingCharacters = colorConfig;

	hsvColor_t *color = &self->colors[index];

	bool result = true;
	static const uint16_t hsv_limit[HSV_COLOR_COMPONENT_COUNT] = {
		[HSV_HUE] = HSV_HUE_MAX,
		[HSV_SATURATION] = HSV_SATURATION_MAX,
		[HSV_VALUE] = HSV_VALUE_MAX,
	};
	for (int componentIndex = 0; result && componentIndex < HSV_COLOR_COMPONENT_COUNT; componentIndex++) {
		int val = atoi(remainingCharacters);
		if(val > hsv_limit[componentIndex]) {
			result = false;
			break;
		}
		switch (componentIndex) {
			case HSV_HUE:
				color->h = val;
				break;
			case HSV_SATURATION:
				color->s = val;
				break;
			case HSV_VALUE:
				color->v = val;
				break;
			default:
				break;
		}
		remainingCharacters = strchr(remainingCharacters, ',');
		if (remainingCharacters) {
			remainingCharacters++;  // skip separator
		} else {
			if (componentIndex < HSV_COLOR_COMPONENT_COUNT - 1) {
				result = false;
			}
		}
	}

	if (!result) {
		memset(color, 0, sizeof(*color));
	}

	return result;
}


