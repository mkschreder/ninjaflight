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
#include <string.h>

#include <platform.h>

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "config/config.h"
#include "config/config_eeprom.h"
#include "config/feature.h"
#include "config/profile.h"
#include "config/config_reset.h"
#include "config/config_system.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/system.h"
#include "drivers/serial.h"

#include "io/rc_adjustments.h"
#include "io/beeper.h"
#include "io/serial.h"

#include "sensors/compass.h"
#include "sensors/acceleration.h"

#include "telemetry/telemetry.h"

#include "flight/rate_profile.h"
#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "flight/anglerate.h"
#include "flight/navigation.h"

#include "rx/rx.h"

// FIXME remove the includes below when target specific configuration is moved out of this file
#include "sensors/battery.h"

// Default settings

void handleOneshotFeatureChangeOnRestart(void)
{
    // Shutdown PWM on all motors prior to soft restart
	// TODO: this is not supposed to be called from here and with new changes this is very apparent. So fix this after done refactoring. 
    //StopPwmAllMotors(&default_mixer);
    usleep(50000);
    // Apply additional delay when OneShot125 feature changed from on to off state
    if (feature(FEATURE_ONESHOT125) && !featureConfigured(FEATURE_ONESHOT125)) {
        usleep(ONESHOT_FEATURE_CHANGED_DELAY_ON_BOOT_MS * 1000);
    }
}
