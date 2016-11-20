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

#include <math.h>

#include <platform.h>

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "config/config.h"
#include "config/runtime_config.h"
#include "config/feature.h"
#include "config/config_reset.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"

#include "io/rc_controls.h"

#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "sensors/instruments.h"

#include "io/gps.h"
#include "io/beeper.h"
#include "io/rc_curves.h"

#include "io/display.h"

#include "flight/anglerate.h"
#include "flight/navigation.h"
#include "flight/failsafe.h"

#include "ninjaflight.h"


