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

#include <stdbool.h>

void systemInit(void);

#ifndef __linux__
void usleep(uint32_t us);
#else
#include <unistd.h>
#include <time.h>
// since _XOPEN_SOURCE (or posix 2008) usleep is deprecated and nanosleep should be used instead.
#if _XOPEN_SOURCE > 500
int usleep(uint32_t us);
#endif
#endif

int32_t micros(void);
int32_t millis(void);

// failure
void failureMode(uint8_t mode);

// bootloader/IAP
void systemReset(void);
void systemResetToBootloader(void);
bool isMPUSoftReset(void);

void enableGPIOPowerUsageAndNoiseReductions(void);
// current crystal frequency - 8 or 12MHz
extern uint32_t hse_value;

typedef void extiCallbackHandlerFunc(void);

void registerExtiCallbackHandler(int irqn, extiCallbackHandlerFunc *fn);
void unregisterExtiCallbackHandler(int irqn, extiCallbackHandlerFunc *fn);

extern uint32_t cachedRccCsrValue;

typedef enum {
    FAILURE_DEVELOPER = 0,
    FAILURE_MISSING_ACC,
    FAILURE_ACC_INIT,
    FAILURE_ACC_INCOMPATIBLE,
    FAILURE_INVALID_EEPROM_CONTENTS,
    FAILURE_FLASH_WRITE_FAILED,
    FAILURE_GYRO_INIT_FAILED
} failureMode_e;

