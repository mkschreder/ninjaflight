/*
 * This file is part of Ninjaflight.
 *
 * Copyright: collective.
 * Cleanup: Martin Schröder <mkschreder.uk@gmail.com>
 * Original source from Cleanflight.
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

typedef enum {
    FAILSAFE_PROCEDURE_AUTO_LANDING = 0,
    FAILSAFE_PROCEDURE_DROP_IT
} failsafe_procedure_t;

//! Failsafe system configuration
typedef struct failsafeConfig_s {
    uint8_t failsafe_delay;                 //!< Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example (10)
    uint8_t failsafe_off_delay;             //!< Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example (200)
    uint16_t failsafe_throttle;             //!< Throttle level used for landing - specify value between 1000..2000 (pwm pulse width for slightly below hover). center throttle = 1500.
    uint8_t failsafe_kill_switch;           //!< failsafe switch action is 0: identical to rc link loss, 1: disarms instantly
    uint16_t failsafe_throttle_low_delay;   //!< Time throttle stick must have been below 'min_check' to "JustDisarm" instead of "full failsafe procedure".
    uint8_t failsafe_procedure;             //!< selected full failsafe procedure is 0: auto-landing, 1: Drop it
} __attribute__((packed)) failsafeConfig_t;

PG_DECLARE(failsafeConfig_t, failsafeConfig);
