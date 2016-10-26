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
#include "build_config.h"

#include "system.h"
#include "gpio.h"
#include "nvic.h"

#include "sonar_hcsr04.h"

/* HC-SR04 consists of ultrasonic transmitter, receiver, and control circuits.
 * When triggered it sends out a series of 40KHz ultrasonic pulses and receives
 * echo from an object. The distance between the unit and the object is calculated
 * by measuring the traveling time of sound and output it as the width of a TTL pulse.
 *
 * *** Warning: HC-SR04 operates at +5V ***
 *
 */

#if defined(SONAR)

// TODO: double check that sonar works (this code looks very messy and we also change things) 
// TODO: make sure we use workaround for cheap hcsr04 sonars that have locking issue

#define SONAR_GPIO GPIOB

#define HCSR04_MAX_RANGE_CM 400 // 4m, from HC-SR04 spec sheet
#define HCSR04_DETECTION_CONE_DECIDEGREES 300 // recommended cone angle30 degrees, from HC-SR04 spec sheet
#define HCSR04_DETECTION_CONE_EXTENDED_DECIDEGREES 450 // in practice 45 degrees seems to work well

// TODO: this is being abused in sensors/sonar.c 
struct sonar_hardware {
    uint16_t trigger_pin;
	GPIO_TypeDef* trigger_gpio;
    uint16_t echo_pin;
	GPIO_TypeDef* echo_gpio;
    uint32_t exti_line;
    uint8_t exti_pin_source;
    IRQn_Type exti_irqn;
};

typedef struct sonarRange_s {
    int16_t maxRangeCm;
    // these are full detection cone angles, maximum tilt is half of this
    int16_t detectionConeDeciDegrees; // detection cone angle as in HC-SR04 device spec
    int16_t detectionConeExtendedDeciDegrees; // device spec is conservative, in practice have slightly larger detection cone
} sonarRange_t;

STATIC_UNIT_TESTED volatile int32_t measurement = -1;
static uint32_t lastMeasurementAt;

static const struct sonar_hardware *sonar_hardware; 

#if !defined(UNIT_TEST)
static void ECHO_EXTI_IRQHandler(void)
{
    static uint32_t timing_start;
    uint32_t timing_stop;

    if (digitalIn(sonar_hardware->echo_gpio, sonar_hardware->echo_pin) != 0) {
        timing_start = micros();
    } else {
        timing_stop = micros();
        if (timing_stop > timing_start) {
            measurement = timing_stop - timing_start;
        }
    }

    EXTI_ClearITPendingBit(sonar_hardware->exti_line);
}

void EXTI0_IRQHandler(void); 
void EXTI0_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}

void EXTI1_IRQHandler(void); 
void EXTI1_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}

void EXTI9_5_IRQHandler(void); 
void EXTI9_5_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}
#endif

#if defined(SONAR_PWM_TRIGGER_PIN)
    static const struct sonar_hardware const sonarPWM = {
        .trigger_pin = SONAR_PWM_TRIGGER_PIN,
        .trigger_gpio = SONAR_PWM_TRIGGER_GPIO,
        .echo_pin = SONAR_PWM_ECHO_PIN,
        .echo_gpio = SONAR_PWM_ECHO_GPIO,
        .exti_line = SONAR_PWM_EXTI_LINE,
        .exti_pin_source = SONAR_PWM_EXTI_PIN_SOURCE,
        .exti_irqn = SONAR_PWM_EXTI_IRQN
    };
#endif
#if !defined(UNIT_TEST)
    static const struct sonar_hardware sonarRC = {
        .trigger_pin = SONAR_TRIGGER_PIN,
        .trigger_gpio = SONAR_TRIGGER_GPIO,
        .echo_pin = SONAR_ECHO_PIN,
        .echo_gpio = SONAR_ECHO_GPIO,
        .exti_line = SONAR_EXTI_LINE,
        .exti_pin_source = SONAR_EXTI_PIN_SOURCE,
        .exti_irqn = SONAR_EXTI_IRQN
    };
#endif

void hcsr04_init(struct sonar *self){
#if defined(SONAR_PWM_TRIGGER_PIN)
	self->hw = sonar_hardware = &sonarPWM; 
#elif !defined(UNIT_TEST)
	self->hw = sonar_hardware = &sonarRC; 
#else
	// TODO: fix sonar in unit tests
#endif
    self->max_range_cm = HCSR04_MAX_RANGE_CM;
    self->detection_cone_deci_degrees = HCSR04_DETECTION_CONE_DECIDEGREES;
    self->detection_cone_extended_deci_degrees = HCSR04_DETECTION_CONE_EXTENDED_DECIDEGREES;

#if !defined(UNIT_TEST)
    gpio_config_t gpio;
    EXTI_InitTypeDef EXTIInit;

#ifdef STM32F10X
    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif

#ifdef STM32F303xC
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    /* Enable SYSCFG clock otherwise the EXTI irq handlers are not called */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif

    // trigger pin
    gpio.pin = sonar_hardware->trigger_pin;
    gpio.mode = Mode_Out_PP;
    gpio.speed = Speed_2MHz;
    gpioInit(sonar_hardware->trigger_gpio, &gpio);

    // echo pin
    gpio.pin = sonar_hardware->echo_pin;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(sonar_hardware->echo_gpio, &gpio);

#ifdef STM32F10X
    // setup external interrupt on echo pin
    gpioExtiLineConfig(GPIO_PortSourceGPIOB, sonar_hardware->exti_pin_source);
#endif

#ifdef STM32F303xC
    gpioExtiLineConfig(EXTI_PortSourceGPIOB, sonar_hardware->exti_pin_source);
#endif

    EXTI_ClearITPendingBit(sonar_hardware->exti_line);

    EXTIInit.EXTI_Line = sonar_hardware->exti_line;
    EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTIInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTIInit);

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = sonar_hardware->exti_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_SONAR_ECHO);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_SONAR_ECHO);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    lastMeasurementAt = millis() - 60; // force 1st measurement in hcsr04_get_distance()
#else
    lastMeasurementAt = 0; // to avoid "unused" compiler warning
#endif
}

// measurement reading is done asynchronously, using interrupt
void hcsr04_start_reading(struct sonar *self)
{
#if !defined(UNIT_TEST)
    uint32_t now = millis();

    if (now < (lastMeasurementAt + 60)) {
        // the repeat interval of trig signal should be greater than 60ms
        // to avoid interference between connective measurements.
        return;
    }

    lastMeasurementAt = now;

    digitalHi(self->hw->trigger_gpio, self->hw->trigger_pin);
    //  The width of trig signal must be greater than 10us
    delayMicroseconds(11);
    digitalLo(self->hw->trigger_gpio, self->hw->trigger_pin);
#else
	UNUSED(self); 
#endif
}

/**
 * Get the distance that was measured by the last pulse, in centimeters.
 */
int32_t hcsr04_get_distance(struct sonar *self)
{
    // The speed of sound is 340 m/s or approx. 29 microseconds per centimeter.
    // The ping travels out and back, so to find the distance of the
    // object we take half of the distance traveled.
    //
    // 340 m/s = 0.034 cm/microsecond = 29.41176471 *2 = 58.82352941 rounded to 59
    self->distance = measurement / 59;

    return self->distance;
}
#endif
