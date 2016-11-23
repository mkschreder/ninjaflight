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

#include <platform.h>

#include "drivers/system.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"

#include "statusindicator.h"

static int32_t warningLedTimer = 0;

typedef enum {
    WARNING_LED_OFF = 0,
    WARNING_LED_ON,
    WARNING_LED_FLASH
} warningLedState_e;

static warningLedState_e warningLedState = WARNING_LED_OFF;

void warningLedEnable(void)
{
    warningLedState = WARNING_LED_ON;
}

void warningLedDisable(void)
{
    warningLedState = WARNING_LED_OFF;
}

void warningLedFlash(void)
{
    warningLedState = WARNING_LED_FLASH;
}

void warningLedRefresh(const struct system_calls_leds *leds)
{
    switch (warningLedState) {
        case WARNING_LED_OFF:
			leds->on(leds, 0, false);
            break;
        case WARNING_LED_ON:
			leds->on(leds, 0, true);
            break;
        case WARNING_LED_FLASH:
			leds->toggle(leds, 0);
            break;
    }
}

void warningLedUpdate(const struct system_calls_leds *leds, int32_t now){
    if ((int32_t)(now - warningLedTimer) < 0) {
        return;
    }

    warningLedRefresh(leds);
    warningLedTimer = now + 500000;
}


