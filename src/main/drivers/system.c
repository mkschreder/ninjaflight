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

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <platform.h>

#include "build_config.h"

#include "gpio.h"
#include "light_led.h"
#include "sound_beeper.h"
#include "nvic.h"
#include "serial.h"
#include "serial_uart.h"

#include "system.h"

#ifndef EXTI_CALLBACK_HANDLER_COUNT
#define EXTI_CALLBACK_HANDLER_COUNT 1
#endif

typedef struct extiCallbackHandlerConfig_s {
    IRQn_Type irqn;
    extiCallbackHandlerFunc* fn;
} extiCallbackHandlerConfig_t;

static extiCallbackHandlerConfig_t extiHandlerConfigs[EXTI_CALLBACK_HANDLER_COUNT];

void registerExtiCallbackHandler(int irqn, extiCallbackHandlerFunc *fn)
{
    for (int index = 0; index < EXTI_CALLBACK_HANDLER_COUNT; index++) {
        extiCallbackHandlerConfig_t *candidate = &extiHandlerConfigs[index];
        if (!candidate->fn) {
            candidate->fn = fn;
            candidate->irqn = irqn;
            return;
        }
    }
    failureMode(FAILURE_DEVELOPER); // EXTI_CALLBACK_HANDLER_COUNT is too low for the amount of handlers required.
}

void unregisterExtiCallbackHandler(int irqn, extiCallbackHandlerFunc *fn)
{
    for (int index = 0; index < EXTI_CALLBACK_HANDLER_COUNT; index++) {
        extiCallbackHandlerConfig_t *candidate = &extiHandlerConfigs[index];
        if (candidate->fn == fn && candidate->irqn == irqn) {
            candidate->fn = NULL;
            candidate->irqn = 0;
            return;
        }
    }
}

static void extiHandler(IRQn_Type irqn)
{
    for (int index = 0; index < EXTI_CALLBACK_HANDLER_COUNT; index++) {
        extiCallbackHandlerConfig_t *candidate = &extiHandlerConfigs[index];
        if (candidate->fn && candidate->irqn == irqn) {
            candidate->fn();
        }
    }

}

void EXTI15_10_IRQHandler(void); 
void EXTI15_10_IRQHandler(void)
{
    extiHandler(EXTI15_10_IRQn);
}

#if defined(CC3D)
 void EXTI3_IRQHandler(void);
 void EXTI3_IRQHandler(void)
{
    extiHandler(EXTI3_IRQn);
}
#endif

#if defined(COLIBRI_RACE) || defined(LUX_RACE)
void EXTI9_5_IRQHandler(void); 
void EXTI9_5_IRQHandler(void)
{
    extiHandler(EXTI9_5_IRQn);
}
#endif

// cycles per microsecond
static uint32_t usTicks = 0;
// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static volatile int32_t sysTickUptime = 0;
// cached value of RCC->CSR
uint32_t cachedRccCsrValue;

static void cycleCounterInit(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    usTicks = clocks.SYSCLK_Frequency / 1000000;
}

// SysTick
//void SysTick_Handler(void); 
//void SysTick_Handler(void)
void vApplicationTickHook(void)
{
    sysTickUptime++;
}

// Return system uptime in microseconds (rollover in 70minutes)
int32_t micros(void){
    register int32_t ms, cycle_cnt;
    do {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;

        /*
         * If the SysTick timer expired during the previous instruction, we need to give it a little time for that
         * interrupt to be delivered before we can recheck sysTickUptime:
         */
        asm volatile("\tnop\n");
    } while (ms != sysTickUptime);
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

// Return system uptime in milliseconds (rollover in 49 days)
int32_t millis(void)
{
    return sysTickUptime;
}

void systemInit(void)
{
#ifdef CC3D
    /* Accounts for OP Bootloader, set the Vector Table base address as specified in .ld file */
    extern void *isr_vector_table_base;

    NVIC_SetVectorTable((uint32_t)&isr_vector_table_base, 0x0);
#endif
    // Configure NVIC preempt/priority groups
    NVIC_PriorityGroupConfig(NVIC_PRIORITY_GROUPING);

#ifdef STM32F10X
    // Turn on clocks for stuff we use
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif

    // cache RCC->CSR value to use it in isMPUSoftreset() and others
    cachedRccCsrValue = RCC->CSR;
    RCC_ClearFlag();

    enableGPIOPowerUsageAndNoiseReductions();

    usartInitAllIOSignals();

#ifdef STM32F10X
    // Turn off JTAG port 'cause we're using the GPIO for leds
#define AFIO_MAPR_SWJ_CFG_NO_JTAG_SW            (0x2 << 24)
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_NO_JTAG_SW;
#endif

    // Init cycle counter
    cycleCounterInit();

    // Init EXTI handler configurations.
    memset(extiHandlerConfigs, 0x00, sizeof(extiHandlerConfigs));

    // SysTick
    SysTick_Config(SystemCoreClock / 1000);
}


#if 1
// TODO: make this use timestamp routines same as in linux kernel
void usleep(uint32_t us)
{
	uint32_t now = micros();
	while (micros() - now < us);
}
#else
void usleep(uint32_t us)
{
    uint32_t elapsed = 0;
    uint32_t lastCount = SysTick->VAL;

    for (;;) {
        register uint32_t current_count = SysTick->VAL;
        uint32_t elapsed_us;

        // measure the time elapsed since the last time we checked
        elapsed += current_count - lastCount;
        lastCount = current_count;

        // convert to microseconds
        elapsed_us = elapsed / usTicks;
        if (elapsed_us >= us)
            break;

        // reduce the delay by the elapsed time
        us -= elapsed_us;

        // keep fractional microseconds for the next iteration
        elapsed %= usTicks;
    }
}
#endif

#define SHORT_FLASH_DURATION 50
#define CODE_FLASH_DURATION 250

void failureMode(failureMode_e mode)
{
    int codeRepeatsRemaining = 10;
    int codeFlashesRemaining;
    int shortFlashesRemaining;

    while (codeRepeatsRemaining--) {
        led_on(1);
        led_off(0);
        shortFlashesRemaining = 5;
        codeFlashesRemaining = mode + 1;
        uint8_t flashDuration = SHORT_FLASH_DURATION;

        while (shortFlashesRemaining || codeFlashesRemaining) {
            led_toggle(1);
            led_toggle(0);
            BEEP_ON;
            usleep(flashDuration * 1000);

            led_toggle(1);
            led_toggle(0);
            BEEP_OFF;
            usleep(flashDuration * 1000);

            if (shortFlashesRemaining) {
                shortFlashesRemaining--;
                if (shortFlashesRemaining == 0) {
                    usleep(500000);
                    flashDuration = CODE_FLASH_DURATION;
                }
            } else {
                codeFlashesRemaining--;
            }
        }
        usleep(1000000);
    }

#ifdef DEBUG
    systemReset();
#else
    systemResetToBootloader();
#endif
}
