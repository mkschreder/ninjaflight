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

#include "rx/rx.h"
#include "../config/frsky.h"

typedef enum {
    FRSKY_VFAS_PRECISION_LOW = 0,
    FRSKY_VFAS_PRECISION_HIGH
} frskyVFasPrecision_e;
void handleFrSkyTelemetry(uint16_t deadband3d_throttle);
void checkFrSkyTelemetryState(void);

void initFrSkyTelemetry(void);
void configureFrSkyTelemetryPort(void);
void freeFrSkyTelemetryPort(void);

