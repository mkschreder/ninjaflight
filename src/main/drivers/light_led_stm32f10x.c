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
#include <stdlib.h>

#include <platform.h>

#include "common/utils.h"

#include "system.h"
#include "gpio.h"

#include "light_led.h"

struct {
    GPIO_TypeDef *gpio;
    uint16_t pin;
} led_config[3];

// TODO: clean up this mess
// Helpful macros
#ifdef LED0
#define LED0_TOGGLE              digitalToggle(led_config[0].gpio, led_config[0].pin)
#ifndef LED0_INVERTED
#define LED0_OFF                 digitalHi(led_config[0].gpio, led_config[0].pin)
#define LED0_ON                  digitalLo(led_config[0].gpio, led_config[0].pin)
#else
#define LED0_OFF                 digitalLo(led_config[0].gpio, led_config[0].pin)
#define LED0_ON                  digitalHi(led_config[0].gpio, led_config[0].pin)
#endif // inverted
#else
#define LED0_TOGGLE              do {} while(0)
#define LED0_OFF                 do {} while(0)
#define LED0_ON                  do {} while(0)
#endif

#ifdef LED1
#define LED1_TOGGLE              digitalToggle(led_config[1].gpio, led_config[1].pin)
#ifndef LED1_INVERTED
#define LED1_OFF                 digitalHi(led_config[1].gpio, led_config[1].pin)
#define LED1_ON                  digitalLo(led_config[1].gpio, led_config[1].pin)
#else
#define LED1_OFF                 digitalLo(led_config[1].gpio, led_config[1].pin)
#define LED1_ON                  digitalHi(led_config[1].gpio, led_config[1].pin)
#endif // inverted
#else
#define LED1_TOGGLE              do {} while(0)
#define LED1_OFF                 do {} while(0)
#define LED1_ON                  do {} while(0)
#endif


#ifdef LED2
#define LED2_TOGGLE              digitalToggle(led_config[2].gpio, led_config[2].pin)
#ifndef LED2_INVERTED
#define LED2_OFF                 digitalHi(led_config[2].gpio, led_config[2].pin)
#define LED2_ON                  digitalLo(led_config[2].gpio, led_config[2].pin)
#else
#define LED2_OFF                 digitalLo(led_config[2].gpio, led_config[2].pin)
#define LED2_ON                  digitalHi(led_config[2].gpio, led_config[2].pin)
#endif // inverted
#else
#define LED2_TOGGLE              do {} while(0)
#define LED2_OFF                 do {} while(0)
#define LED2_ON                  do {} while(0)
#endif

void led_toggle(int id){
	switch(id){
		case 0: 
			LED0_TOGGLE; 
			break; 
		case 1: 
			LED1_TOGGLE; 
			break; 
		case 2: 
			LED2_TOGGLE; 
			break; 
	}
}

void led_on(int id){
	switch(id){
		case 0: 
			LED0_ON; 
			break; 
		case 1: 
			LED1_ON; 
			break; 
		case 2: 
			LED2_ON; 
			break; 
	}
}

void led_off(int id){
	switch(id){
		case 0: 
			LED0_OFF; 
			break; 
		case 1: 
			LED1_OFF; 
			break; 
		case 2: 
			LED2_OFF; 
			break; 
	}
}

void led_init(bool alternative_led)
{
    UNUSED(alternative_led);
#if defined(LED0) || defined(LED1) || defined(LED2)
    gpio_config_t cfg;
    cfg.mode = Mode_Out_PP;
    cfg.speed = Speed_2MHz;
#ifdef LED0
    RCC_APB2PeriphClockCmd(LED0_PERIPHERAL, ENABLE);
    led_config[0].gpio = LED0_GPIO;
    led_config[0].pin = LED0_PIN;
    cfg.pin = led_config[0].pin;
    LED0_OFF;
    gpioInit(led_config[0].gpio, &cfg);
#endif
#ifdef LED1
    RCC_APB2PeriphClockCmd(LED1_PERIPHERAL, ENABLE);
    led_config[1].gpio = LED1_GPIO;
    led_config[1].pin = LED1_PIN;
    cfg.pin = led_config[1].pin;
    LED1_OFF;
    gpioInit(led_config[1].gpio, &cfg);
#endif
#ifdef LED2
    RCC_APB2PeriphClockCmd(LED2_PERIPHERAL, ENABLE);
    led_config[2].gpio = LED2_GPIO;
    led_config[2].pin = LED2_PIN;
    cfg.pin = led_config[2].pin;
    LED2_OFF;
    gpioInit(led_config[2].gpio, &cfg);
#endif
#endif
}

