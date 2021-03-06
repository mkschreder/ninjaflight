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

#include "config/config.h"

#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "io/serial.h"

#include "rx/rx.h"

#include "telemetry/telemetry.h"
#include "telemetry/frsky.h"
#include "telemetry/hott.h"
#include "telemetry/smartport.h"
#include "telemetry/ltm.h"
#include "telemetry/mavlink.h"

// TODO: telemetry
#if 0
void telemetryInit(void)
{
    initFrSkyTelemetry();
    initHoTTTelemetry();
    initSmartPortTelemetry();
    initLtmTelemetry();
    initMAVLinkTelemetry();
    telemetryCheckState();
}

bool telemetryDetermineEnabledState(portSharing_e portSharing)
{
    bool enabled = portSharing == PORTSHARING_NOT_SHARED;

    if (portSharing == PORTSHARING_SHARED) {
        if (telemetryConfig()->telemetry_switch)
            enabled = rcModeIsActive(BOXTELEMETRY);
        else
            enabled = ARMING_FLAG(ARMED);
    }

    return enabled;
}

void telemetryCheckState(void)
{
    checkFrSkyTelemetryState();
    checkHoTTTelemetryState();
    checkSmartPortTelemetryState();
    checkLtmTelemetryState();
    checkMAVLinkTelemetryState();
}

void telemetryProcess(uint16_t deadband3d_throttle)
{
    handleFrSkyTelemetry(deadband3d_throttle);
    handleHoTTTelemetry();
    handleSmartPortTelemetry();
    handleLtmTelemetry();
    handleMAVLinkTelemetry();
}
#endif
