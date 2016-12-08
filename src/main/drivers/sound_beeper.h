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

#include "gpio.h"

#if USE_BEEPER == 1
#define BEEP_TOGGLE              digitalToggle(BEEP_GPIO, BEEP_PIN)
#define BEEP_OFF                 systemBeep(false)
#define BEEP_ON                  systemBeep(true)
#else
#define BEEP_TOGGLE do{} while(0)
#define BEEP_OFF do {} while(0)
#define BEEP_ON do {} while(0)
#endif

typedef struct beeperConfig_s {
    uint32_t gpioPeripheral;
    uint16_t gpioPin;
    GPIO_TypeDef *gpioPort;
    GPIO_Mode gpioMode;
    bool isInverted;
} beeperConfig_t;

void initBeeperHardware(beeperConfig_t *config);
void beeperInit(beeperConfig_t *beeperConfig);

void systemBeep(bool onoff);

