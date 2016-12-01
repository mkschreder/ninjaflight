#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "platform.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"
#include "common/filter.h"

#include "config/parameter_group.h"
#include "config/tilt.h"

#include "io/rc_adjustments.h"

#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "sensors/initialisation.h"

#include "io/beeper.h"
#include "io/display.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_msp.h"
#include "io/statusindicator.h"
#include "io/transponder_ir.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "flight/rate_profile.h"
#include "flight/mixer.h"
#include "flight/anglerate.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/gtune.h"
#include "flight/navigation.h"
#include "flight/tilt.h"
#include "sensors/instruments.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/feature.h"

#include "ninjaflight.h"
#include "ninja.h"
#include "ninja_sched.h"

#define AIRMODE_DEADBAND 12

rollPitchStatus_e calculateRollPitchCenterStatus(struct rx *self, rxConfig_t *rxConfig){
	int16_t pitch = rx_get_channel(self, PITCH);
	int16_t roll = rx_get_channel(self, ROLL);
	if (((pitch < (rxConfig->midrc + AIRMODE_DEADBAND)) && (pitch > (rxConfig->midrc -AIRMODE_DEADBAND)))
			&& ((roll < (rxConfig->midrc + AIRMODE_DEADBAND)) && (roll > (rxConfig->midrc -AIRMODE_DEADBAND))))
		return CENTERED;

	return NOT_CENTERED;
}

void ninja_handle_input(struct ninja *self){
	(void)self;
}

