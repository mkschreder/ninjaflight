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

#if 0
#include <stdbool.h>
#include <string.h>

#include <platform.h>
#include "version.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/encoding.h"
#include "common/utils.h"

#include "flight/rate_profile.h"

#include "sensors/boardalignment.h"
#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"

//#include "io/rc_controls.h"
//#include "io/gps.h"

#include "flight/mixer.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"

#include "config/config.h"
#include "config/feature.h"
#include "config/profile.h"
#include "config/config_reset.h"

#include "../ninja.h"

#include "blackbox.h"
#include "blackbox_io.h"

#define BLACKBOX_I_INTERVAL 32
#define BLACKBOX_SHUTDOWN_TIMEOUT_MILLIS 200
#define SLOW_FRAME_INTERVAL 4096

#define ARRAY_LENGTH(x) (sizeof((x))/sizeof((x)[0]))

#define STATIC_ASSERT(condition, name ) \
	typedef char assert_failed_ ## name [(condition) ? 1 : -1 ]

// Some macros to make writing FLIGHT_LOG_FIELD_* constants shorter:

#define PREDICT(x) CONCAT(FLIGHT_LOG_FIELD_PREDICTOR_, x)
#define ENCODING(x) CONCAT(FLIGHT_LOG_FIELD_ENCODING_, x)
#define CONDITION(x) CONCAT(FLIGHT_LOG_FIELD_CONDITION_, x)
#define UNSIGNED FLIGHT_LOG_FIELD_UNSIGNED
#define SIGNED FLIGHT_LOG_FIELD_SIGNED

static const char blackboxHeader[] =
	"H Product:Blackbox flight data recorder by Nicholas Sherlock\n"
	"H Data version:2\n"
	"H I interval:" STR(BLACKBOX_I_INTERVAL) "\n";

static const char* const blackboxFieldHeaderNames[] = {
	"name",
	"signed",
	"predictor",
	"encoding",
	"predictor",
	"encoding"
};

/* All field definition structs should look like this (but with longer arrs): */
typedef struct blackboxFieldDefinition_s {
	const char *name;
	// If the field name has a number to be included in square brackets [1] afterwards, set it here, or -1 for no brackets:
	int8_t fieldNameIndex;

	// Each member of this array will be the value to print for this field for the given header index
	uint8_t arr[1];
} blackboxFieldDefinition_t;

#define BLACKBOX_DELTA_FIELD_HEADER_COUNT	   ARRAY_LENGTH(blackboxFieldHeaderNames)
#define BLACKBOX_SIMPLE_FIELD_HEADER_COUNT	  (BLACKBOX_DELTA_FIELD_HEADER_COUNT - 2)
#define BLACKBOX_CONDITIONAL_FIELD_HEADER_COUNT (BLACKBOX_DELTA_FIELD_HEADER_COUNT - 2)

typedef struct blackboxSimpleFieldDefinition_s {
	const char *name;
	int8_t fieldNameIndex;

	uint8_t isSigned;
	uint8_t predict;
	uint8_t encode;
} blackboxSimpleFieldDefinition_t;

typedef struct blackboxConditionalFieldDefinition_s {
	const char *name;
	int8_t fieldNameIndex;

	uint8_t isSigned;
	uint8_t predict;
	uint8_t encode;
	uint8_t condition; // Decide whether this field should appear in the log
} blackboxConditionalFieldDefinition_t;

typedef struct blackboxDeltaFieldDefinition_s {
	const char *name;
	int8_t fieldNameIndex;

	uint8_t isSigned;
	uint8_t Ipredict;
	uint8_t Iencode;
	uint8_t Ppredict;
	uint8_t Pencode;
	uint8_t condition; // Decide whether this field should appear in the log
} blackboxDeltaFieldDefinition_t;

/**
 * Description of the blackbox fields we are writing in our main intra (I) and inter (P) frames. This description is
 * written into the flight log header so the log can be properly interpreted (but these definitions don't actually cause
 * the encoding to happen, we have to encode the flight log ourselves in write{Inter|Intra}frame() in a way that matches
 * the encoding we've promised here).
 */
static const blackboxDeltaFieldDefinition_t blackboxMainFields[] = {
	/* loopIteration doesn't appear in P frames since it always increments */
	{"loopIteration",-1, UNSIGNED, .Ipredict = PREDICT(0),	 .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(INC),		   .Pencode = FLIGHT_LOG_FIELD_ENCODING_NULL, CONDITION(ALWAYS)},
	/* Time advances pretty steadily so the P-frame prediction is a straight line */
	{"time",	   -1, UNSIGNED, .Ipredict = PREDICT(0),	   .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(STRAIGHT_LINE), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
	{"axisP",	   0, SIGNED,   .Ipredict = PREDICT(0),	   .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),	  .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
	{"axisP",	   1, SIGNED,   .Ipredict = PREDICT(0),	   .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),	  .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
	{"axisP",	   2, SIGNED,   .Ipredict = PREDICT(0),	   .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),	  .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
	/* I terms get special packed encoding in P frames: */
	{"axisI",	   0, SIGNED,   .Ipredict = PREDICT(0),	   .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),	  .Pencode = ENCODING(TAG2_3S32), CONDITION(ALWAYS)},
	{"axisI",	   1, SIGNED,   .Ipredict = PREDICT(0),	   .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),	  .Pencode = ENCODING(TAG2_3S32), CONDITION(ALWAYS)},
	{"axisI",	   2, SIGNED,   .Ipredict = PREDICT(0),	   .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),	  .Pencode = ENCODING(TAG2_3S32), CONDITION(ALWAYS)},
	{"axisD",	   0, SIGNED,   .Ipredict = PREDICT(0),	   .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),	  .Pencode = ENCODING(SIGNED_VB), CONDITION(NONZERO_PID_D_0)},
	{"axisD",	   1, SIGNED,   .Ipredict = PREDICT(0),	   .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),	  .Pencode = ENCODING(SIGNED_VB), CONDITION(NONZERO_PID_D_1)},
	{"axisD",	   2, SIGNED,   .Ipredict = PREDICT(0),	   .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),	  .Pencode = ENCODING(SIGNED_VB), CONDITION(NONZERO_PID_D_2)},
	/* rcCommands are encoded together as a group in P-frames: */
	{"rcCommand",   0, SIGNED,   .Ipredict = PREDICT(0),	   .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),	  .Pencode = ENCODING(TAG8_4S16), CONDITION(ALWAYS)},
	{"rcCommand",   1, SIGNED,   .Ipredict = PREDICT(0),	   .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),	  .Pencode = ENCODING(TAG8_4S16), CONDITION(ALWAYS)},
	{"rcCommand",   2, SIGNED,   .Ipredict = PREDICT(0),	   .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),	  .Pencode = ENCODING(TAG8_4S16), CONDITION(ALWAYS)},
	/* Throttle is always in the range [minthrottle..maxthrottle]: */
	{"rcCommand",   3, UNSIGNED, .Ipredict = PREDICT(MINTHROTTLE), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),  .Pencode = ENCODING(TAG8_4S16), CONDITION(ALWAYS)},

	{"vbatLatest",	-1, UNSIGNED, .Ipredict = PREDICT(VBATREF),  .Iencode = ENCODING(NEG_14BIT),   .Ppredict = PREDICT(PREVIOUS),  .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_VBAT},
	{"amperageLatest",-1, UNSIGNED, .Ipredict = PREDICT(0),		.Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),  .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC},

#if USE_MAG == 1
	{"magADC",	  0, SIGNED,   .Ipredict = PREDICT(0),	   .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),	  .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_MAG},
	{"magADC",	  1, SIGNED,   .Ipredict = PREDICT(0),	   .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),	  .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_MAG},
	{"magADC",	  2, SIGNED,   .Ipredict = PREDICT(0),	   .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),	  .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_MAG},
#endif
#ifdef BARO
	{"BaroAlt",	-1, SIGNED,   .Ipredict = PREDICT(0),	   .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),	  .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_BARO},
#endif
#ifdef SONAR
	{"sonarRaw",   -1, SIGNED,   .Ipredict = PREDICT(0),	   .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),	  .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_SONAR},
#endif
	{"rssi",	   -1, UNSIGNED, .Ipredict = PREDICT(0),	   .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),	  .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_RSSI},

	/* Gyros and accelerometers base their P-predictions on the average of the previous 2 frames to reduce noise impact */
	{"gyroADC",   0, SIGNED,   .Ipredict = PREDICT(0),	   .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),	 .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
	{"gyroADC",   1, SIGNED,   .Ipredict = PREDICT(0),	   .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),	 .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
	{"gyroADC",   2, SIGNED,   .Ipredict = PREDICT(0),	   .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),	 .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
	{"accSmooth",  0, SIGNED,   .Ipredict = PREDICT(0),	   .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),	 .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
	{"accSmooth",  1, SIGNED,   .Ipredict = PREDICT(0),	   .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),	 .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
	{"accSmooth",  2, SIGNED,   .Ipredict = PREDICT(0),	   .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),	 .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
	/* Motors only rarely drops under minthrottle (when stick falls below mincommand), so predict minthrottle for it and use *unsigned* encoding (which is large for negative numbers but more compact for positive ones): */
	{"motor",	  0, UNSIGNED, .Ipredict = PREDICT(MINTHROTTLE), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_1)},
	/* Subsequent motors base their I-frame values on the first one, P-frame values on the average of last two frames: */
	{"motor",	  1, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),	 .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_2)},
	{"motor",	  2, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),	 .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_3)},
	{"motor",	  3, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),	 .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_4)},
	{"motor",	  4, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),	 .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_5)},
	{"motor",	  5, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),	 .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_6)},
	{"motor",	  6, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),	 .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_7)},
	{"motor",	  7, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),	 .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_8)},

	/* Tricopter tail servo */
	{"servo",	  5, UNSIGNED, .Ipredict = PREDICT(1500),	.Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),	  .Pencode = ENCODING(SIGNED_VB), CONDITION(TRICOPTER)}
};

// GPS position/vel frame
static const blackboxConditionalFieldDefinition_t blackboxGpsGFields[] = {
	{"time",			  -1, UNSIGNED, PREDICT(LAST_MAIN_FRAME_TIME), ENCODING(UNSIGNED_VB), CONDITION(NOT_LOGGING_EVERY_FRAME)},
	{"GPS_numSat",		-1, UNSIGNED, PREDICT(0),		  ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
	{"GPS_coord",		  0, SIGNED,   PREDICT(HOME_COORD), ENCODING(SIGNED_VB),   CONDITION(ALWAYS)},
	{"GPS_coord",		  1, SIGNED,   PREDICT(HOME_COORD), ENCODING(SIGNED_VB),   CONDITION(ALWAYS)},
	{"GPS_altitude",	  -1, UNSIGNED, PREDICT(0),		  ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
	{"GPS_speed",		 -1, UNSIGNED, PREDICT(0),		  ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
	{"GPS_ground_course", -1, UNSIGNED, PREDICT(0),		  ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)}
};

// GPS home frame
static const blackboxSimpleFieldDefinition_t blackboxGpsHFields[] = {
	{"GPS_home",		   0, SIGNED,   PREDICT(0),		  ENCODING(SIGNED_VB)},
	{"GPS_home",		   1, SIGNED,   PREDICT(0),		  ENCODING(SIGNED_VB)}
};

// Rarely-updated fields
static const blackboxSimpleFieldDefinition_t blackboxSlowFields[] = {
	{"flightModeFlags",	   -1, UNSIGNED, PREDICT(0),	  ENCODING(UNSIGNED_VB)},
	{"stateFlags",			-1, UNSIGNED, PREDICT(0),	  ENCODING(UNSIGNED_VB)},

	{"failsafePhase",		 -1, UNSIGNED, PREDICT(0),	  ENCODING(TAG2_3S32)},
	{"rxSignalReceived",	  -1, UNSIGNED, PREDICT(0),	  ENCODING(TAG2_3S32)},
	{"rxFlightChannelsValid", -1, UNSIGNED, PREDICT(0),	  ENCODING(TAG2_3S32)}
};

//static uint32_t blackboxLastArmingBeep = 0;

/**
 * Return true if it is safe to edit the Blackbox configuration.
 */
bool blackboxMayEditConfig(struct blackbox *self)
{
	return self->blackboxState <= BLACKBOX_STATE_STOPPED;
}

static bool blackboxIsOnlyLoggingIntraframes(struct blackbox *self){
	return self->config->blackbox.rate_num == 1 && self->config->blackbox.rate_denom == 32;
}

static bool testBlackboxConditionUncached(struct blackbox *self, FlightLogFieldCondition condition)
{
	switch (condition) {
		case FLIGHT_LOG_FIELD_CONDITION_ALWAYS:
			return true;

		case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1:
		case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_2:
		case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_3:
		case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_4:
		case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_5:
		case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_6:
		case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_7:
		case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_8:
			return mixer_get_motor_count(&self->ninja->mixer) >= condition - FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1 + 1;
		
		case FLIGHT_LOG_FIELD_CONDITION_TRICOPTER:
			return self->config->mixer.mixerMode == MIXER_TRI || self->config->mixer.mixerMode == MIXER_CUSTOM_TRI;

		case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0:
		case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_1:
		case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_2:
			return config_get_current_profile(self->config)->pid.D8[condition - FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0] != 0;

		case FLIGHT_LOG_FIELD_CONDITION_MAG:
			if(USE_MAG)
				return ninja_has_sensors(self->ninja, NINJA_SENSOR_MAG);
			return false;

		case FLIGHT_LOG_FIELD_CONDITION_BARO:
#ifdef BARO
			return ninja_has_sensors(self->ninja, NINJA_SENSOR_BARO);
#else
			return false;
#endif

		case FLIGHT_LOG_FIELD_CONDITION_VBAT:
			return feature(FEATURE_VBAT);

		case FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC:
			return feature(FEATURE_CURRENT_METER) && self->config->bat.currentMeterType == CURRENT_SENSOR_ADC;

		case FLIGHT_LOG_FIELD_CONDITION_SONAR:
#ifdef SONAR
			return feature(FEATURE_SONAR);
#else
			return false;
#endif

		case FLIGHT_LOG_FIELD_CONDITION_RSSI:
			return self->config->rx.rssi_channel > 0 || feature(FEATURE_RSSI_ADC);

		case FLIGHT_LOG_FIELD_CONDITION_NOT_LOGGING_EVERY_FRAME:
			return self->config->blackbox.rate_num < self->config->blackbox.rate_denom;

		case FLIGHT_LOG_FIELD_CONDITION_NEVER:
			return false;
		default:
			return false;
	}
}

static void blackboxBuildConditionCache(struct blackbox *self)
{
	FlightLogFieldCondition cond;

	self->blackboxConditionCache = 0;

	for (cond = FLIGHT_LOG_FIELD_CONDITION_FIRST; cond <= FLIGHT_LOG_FIELD_CONDITION_LAST; cond++) {
		if (testBlackboxConditionUncached(self, cond)) {
			self->blackboxConditionCache |= 1 << cond;
		}
	}
}

static bool testBlackboxCondition(struct blackbox *self, FlightLogFieldCondition condition)
{
	return (self->blackboxConditionCache & (1 << condition)) != 0;
}

static void blackboxSetState(struct blackbox *self, BlackboxState newState)
{
	//Perform initial setup required for the new state
	switch (newState) {
		case BLACKBOX_STATE_PREPARE_LOG_FILE:
			self->blackboxLoggedAnyFrames = false;
		break;
		case BLACKBOX_STATE_SEND_HEADER:
			self->blackboxHeaderBudget = 0;
			self->xmitState.headerIndex = 0;
			self->xmitState.u.startTime = sys_millis(self->ninja->system);
		break;
		case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER:
		case BLACKBOX_STATE_SEND_GPS_G_HEADER:
		case BLACKBOX_STATE_SEND_GPS_H_HEADER:
		case BLACKBOX_STATE_SEND_SLOW_HEADER:
			self->xmitState.headerIndex = 0;
			self->xmitState.u.fieldIndex = -1;
		break;
		case BLACKBOX_STATE_SEND_SYSINFO:
			self->xmitState.headerIndex = 0;
		break;
		case BLACKBOX_STATE_RUNNING:
			self->blackboxSlowFrameIterationTimer = SLOW_FRAME_INTERVAL; //Force a slow frame to be written on the first iteration
		break;
		case BLACKBOX_STATE_SHUTTING_DOWN:
			self->xmitState.u.startTime = sys_millis(self->ninja->system);
		break;
		case BLACKBOX_STATE_DISABLED:
		case BLACKBOX_STATE_STOPPED:
		case BLACKBOX_STATE_PAUSED:
		default:
			;
	}
	self->blackboxState = newState;
}

static void writeIntraframe(struct blackbox *self)
{
	blackboxMainState_t *blackboxCurrent = blackboxHistory[0];
	int x;

	blackboxWrite(self, 'I');

	blackboxWriteUnsignedVB(blackboxIteration);
	blackboxWriteUnsignedVB(blackboxCurrent->time);

	blackboxWriteSignedVBArray(blackboxCurrent->axisPID_P, XYZ_AXIS_COUNT);
	blackboxWriteSignedVBArray(blackboxCurrent->axisPID_I, XYZ_AXIS_COUNT);

	// Don't bother writing the current D term if the corresponding PID setting is zero
	for (x = 0; x < XYZ_AXIS_COUNT; x++) {
		if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0 + x)) {
			blackboxWriteSignedVB(blackboxCurrent->axisPID_D[x]);
		}
	}

	// Write roll, pitch and yaw first:
	blackboxWriteSigned16VBArray(blackboxCurrent->rcCommand, 3);

	/*
	 * Write the throttle separately from the rest of the RC data so we can apply a predictor to it.
	 * Throttle lies in range [minthrottle..maxthrottle]:
	 */
	blackboxWriteUnsignedVB(blackboxCurrent->rcCommand[THROTTLE] - self->config->pwm_out.minthrottle);

	if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_VBAT)) {
		/*
		 * Our voltage is expected to decrease over the course of the flight, so store our difference from
		 * the reference:
		 *
		 * Write 14 bits even if the number is negative (which would otherwise result in 32 bits)
		 */
		blackboxWriteUnsignedVB((vbatReference - blackboxCurrent->vbatLatest) & 0x3FFF);
	}

	if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC)) {
		// 12bit value directly from ADC
		blackboxWriteUnsignedVB(blackboxCurrent->amperageLatest);
	}

	if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_MAG)) {
		blackboxWriteSigned16VBArray(blackboxCurrent->magADC, XYZ_AXIS_COUNT);
	}

#ifdef BARO
		if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_BARO)) {
			blackboxWriteSignedVB(blackboxCurrent->BaroAlt);
		}
#endif

#ifdef SONAR
		if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_SONAR)) {
			blackboxWriteSignedVB(blackboxCurrent->sonarRaw);
		}
#endif

	if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_RSSI)) {
		blackboxWriteUnsignedVB(blackboxCurrent->rssi);
	}

	blackboxWriteSigned16VBArray(blackboxCurrent->gyroADC, XYZ_AXIS_COUNT);
	blackboxWriteSigned16VBArray(blackboxCurrent->accSmooth, XYZ_AXIS_COUNT);

	//Motors can be below minthrottle when disarmed, but that doesn't happen much
	blackboxWriteUnsignedVB(blackboxCurrent->motor[0] - self->config->pwm_out.minthrottle);

	//Motors tend to be similar to each other so use the first motor's value as a predictor of the others
	for (x = 1; x < mixer_get_motor_count(&self->ninja->mixer); x++) {
		blackboxWriteSignedVB(blackboxCurrent->motor[x] - blackboxCurrent->motor[0]);
	}

	if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_TRICOPTER)) {
		//Assume the tail spends most of its time around the center
		blackboxWriteSignedVB(blackboxCurrent->servo[5] - 1500);
	}

	//Rotate our history buffers:

	//The current state becomes the new "before" state
	blackboxHistory[1] = blackboxHistory[0];
	//And since we have no other history, we also use it for the "before, before" state
	blackboxHistory[2] = blackboxHistory[0];
	//And advance the current state over to a blank space ready to be filled
	blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;

	blackboxLoggedAnyFrames = true;
}

static void blackboxWriteMainStateArrayUsingAveragePredictor(int arrOffsetInHistory, int count)
{
	int16_t *curr  = (int16_t*) ((char*) (blackboxHistory[0]) + arrOffsetInHistory);
	int16_t *prev1 = (int16_t*) ((char*) (blackboxHistory[1]) + arrOffsetInHistory);
	int16_t *prev2 = (int16_t*) ((char*) (blackboxHistory[2]) + arrOffsetInHistory);

	for (int i = 0; i < count; i++) {
		// Predictor is the average of the previous two history states
		int32_t predictor = (prev1[i] + prev2[i]) / 2;

		blackboxWriteSignedVB(curr[i] - predictor);
	}
}

static void writeInterframe(struct blackbox *self)
{
	int x;
	int32_t deltas[8];

	blackboxMainState_t *blackboxCurrent = blackboxHistory[0];
	blackboxMainState_t *blackboxLast = blackboxHistory[1];

	blackboxWrite(self, 'P');

	//No need to store iteration count since its delta is always 1

	/*
	 * Since the difference between the difference between successive times will be nearly zero (due to consistent
	 * looptime spacing), use second-order differences.
	 */
	blackboxWriteSignedVB((int32_t) (blackboxHistory[0]->time - 2 * blackboxHistory[1]->time + blackboxHistory[2]->time));

	arraySubInt32(deltas, blackboxCurrent->axisPID_P, blackboxLast->axisPID_P, XYZ_AXIS_COUNT);
	blackboxWriteSignedVBArray(deltas, XYZ_AXIS_COUNT);

	/* 
	 * The PID I field changes very slowly, most of the time +-2, so use an encoding
	 * that can pack all three fields into one byte in that situation.
	 */
	arraySubInt32(deltas, blackboxCurrent->axisPID_I, blackboxLast->axisPID_I, XYZ_AXIS_COUNT);
	blackboxWriteTag2_3S32(deltas);
	
	/*
	 * The PID D term is frequently set to zero for yaw, which makes the result from the calculation
	 * always zero. So don't bother recording D results when PID D terms are zero.
	 */
	for (x = 0; x < XYZ_AXIS_COUNT; x++) {
		if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0 + x)) {
			blackboxWriteSignedVB(blackboxCurrent->axisPID_D[x] - blackboxLast->axisPID_D[x]);
		}
	}

	/*
	 * RC tends to stay the same or fairly small for many frames at a time, so use an encoding that
	 * can pack multiple values per byte:
	 */
	for (x = 0; x < 4; x++) {
		deltas[x] = blackboxCurrent->rcCommand[x] - blackboxLast->rcCommand[x];
	}

	blackboxWriteTag8_4S16(deltas);

	//Check for sensors that are updated periodically (so deltas are normally zero)
	int optionalFieldCount = 0;

	if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_VBAT)) {
		deltas[optionalFieldCount++] = (int32_t) blackboxCurrent->vbatLatest - blackboxLast->vbatLatest;
	}

	if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC)) {
		deltas[optionalFieldCount++] = (int32_t) blackboxCurrent->amperageLatest - blackboxLast->amperageLatest;
	}

	if (USE_MAG && testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_MAG)) {
		for (x = 0; x < XYZ_AXIS_COUNT; x++) {
			deltas[optionalFieldCount++] = blackboxCurrent->magADC[x] - blackboxLast->magADC[x];
		}
	}

#ifdef BARO
	if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_BARO)) {
		deltas[optionalFieldCount++] = blackboxCurrent->BaroAlt - blackboxLast->BaroAlt;
	}
#endif

#ifdef SONAR
	if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_SONAR)) {
		deltas[optionalFieldCount++] = blackboxCurrent->sonarRaw - blackboxLast->sonarRaw;
	}
#endif

	if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_RSSI)) {
		deltas[optionalFieldCount++] = (int32_t) blackboxCurrent->rssi - blackboxLast->rssi;
	}

	blackboxWriteTag8_8SVB(deltas, optionalFieldCount);

	//Since gyros, accs and motors are noisy, base their predictions on the average of the history:
	blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, gyroADC),   XYZ_AXIS_COUNT);
	blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, accSmooth), XYZ_AXIS_COUNT);
	blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, motor),	 mixer_get_motor_count(&self->ninja->mixer));

	if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_TRICOPTER)) {
		blackboxWriteSignedVB(blackboxCurrent->servo[5] - blackboxLast->servo[5]);
	}

	//Rotate our history buffers
	blackboxHistory[2] = blackboxHistory[1];
	blackboxHistory[1] = blackboxHistory[0];
	blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;

	blackboxLoggedAnyFrames = true;
}

/* Write the contents of the global "slowHistory" to the log as an "S" frame. Because this data is logged so
 * infrequently, delta updates are not reasonable, so we log independent frames. */
static void writeSlowFrame(struct blackbox *self)
{
	int32_t values[3];

	blackboxWrite(self, 'S');

	blackboxWriteUnsignedVB(slowHistory.flightModeFlags);
	blackboxWriteUnsignedVB(slowHistory.stateFlags);

	/*
	 * Most of the time these three values will be able to pack into one byte for us:
	 */
	values[0] = slowHistory.failsafePhase;
	values[1] = slowHistory.rxSignalReceived ? 1 : 0;
	values[2] = slowHistory.rxFlightChannelsValid ? 1 : 0;
	blackboxWriteTag2_3S32(values);

	blackboxSlowFrameIterationTimer = 0;
}

/**
 * Load rarely-changing values from the FC into the given structure
 */
static void loadSlowState(struct blackbox *self, blackboxSlowState_t *slow)
{
	// TODO: blackbox flight mode flags
	//slow->flightModeFlags = flightModeFlags;
	//slow->stateFlags = stateFlags;
	slow->failsafePhase = failsafe_get_state(&self->ninja->failsafe);
	slow->rxSignalReceived = rx_has_signal(&self->ninja->rx);
	slow->rxFlightChannelsValid = rx_flight_channels_valid(&self->ninja->rx);
}

/**
 * If the data in the slow frame has changed, log a slow frame.
 *
 * If allowPeriodicWrite is true, the frame is also logged if it has been more than SLOW_FRAME_INTERVAL logging iterations
 * since the field was last logged.
 */
static void writeSlowFrameIfNeeded(struct blackbox *self, bool allowPeriodicWrite)
{
	// Write the slow frame peridocially so it can be recovered if we ever lose sync
	bool shouldWrite = allowPeriodicWrite && blackboxSlowFrameIterationTimer >= SLOW_FRAME_INTERVAL;

	if (shouldWrite) {
		loadSlowState(self, &slowHistory);
	} else {
		blackboxSlowState_t newSlowState;

		loadSlowState(self, &newSlowState);

		// Only write a slow frame if it was different from the previous state
		if (memcmp(&newSlowState, &slowHistory, sizeof(slowHistory)) != 0) {
			// Use the new state as our new history
			memcpy(&slowHistory, &newSlowState, sizeof(slowHistory));
			shouldWrite = true;
		}
	}

	if (shouldWrite) {
		writeSlowFrame(self);
	}
}

/**
 * Start Blackbox logging if it is not already running. Intended to be called upon arming.
 */
void blackbox_start(struct blackbox *self)
{
	if (self->blackboxState == BLACKBOX_STATE_STOPPED) {
		if (!blackboxDeviceOpen()) {
			blackboxSetState(self, BLACKBOX_STATE_DISABLED);
			return;
		}

		memset(&gpsHistory, 0, sizeof(gpsHistory));

		blackboxHistory[0] = &blackboxHistoryRing[0];
		blackboxHistory[1] = &blackboxHistoryRing[1];
		blackboxHistory[2] = &blackboxHistoryRing[2];

		vbatReference = battery_get_voltage(&self->ninja->bat);

		//No need to clear the content of blackboxHistoryRing since our first frame will be an intra which overwrites it

		/*
		 * We use conditional tests to decide whether or not certain fields should be logged. Since our headers
		 * must always agree with the logged data, the results of these tests must not change during logging. So
		 * cache those now.
		 */
		blackboxBuildConditionCache(self);
		
		blackboxModeActivationConditionPresent = true; //rcModeIsActivationConditionPresent(modeActivationProfile()->modeActivationConditions, BOXBLACKBOX);

		blackboxIteration = 0;
		blackboxPFrameIndex = 0;
		blackboxIFrameIndex = 0;

		/*
		 * Record the beeper's current idea of the last arming beep time, so that we can detect it changing when
		 * it finally plays the beep for this arming event.
		 */
		//blackboxLastArmingBeep = getArmingBeepTimeMicros();

		blackboxSetState(self, BLACKBOX_STATE_PREPARE_LOG_FILE);
	}
}

/**
 * Begin Blackbox shutdown.
 */
void blackbox_stop(struct blackbox *self)
{
	switch (self->blackboxState) {
		case BLACKBOX_STATE_DISABLED:
		case BLACKBOX_STATE_STOPPED:
		case BLACKBOX_STATE_SHUTTING_DOWN:
			// We're already stopped/shutting down
		break;

		case BLACKBOX_STATE_RUNNING:
		case BLACKBOX_STATE_PAUSED:
			blackboxLogEvent(FLIGHT_LOG_EVENT_LOG_END, NULL);

			// Fall through
		default:
			blackboxSetState(self, BLACKBOX_STATE_SHUTTING_DOWN);
		case BLACKBOX_STATE_PREPARE_LOG_FILE:
		case BLACKBOX_STATE_SEND_HEADER:
		case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER:
		case BLACKBOX_STATE_SEND_GPS_H_HEADER:
		case BLACKBOX_STATE_SEND_GPS_G_HEADER:
		case BLACKBOX_STATE_SEND_SLOW_HEADER:
		case BLACKBOX_STATE_SEND_SYSINFO:
			break;
	}
}

#ifdef GPS
static void writeGPSHomeFrame(struct blackbox *self)
{
	(void)self;
	blackboxWrite(self, 'H');

	// TODO: navigaiton data in blackbox
	//blackboxWriteSignedVB(self->ninja->gps.GPS_home[0]);
	//blackboxWriteSignedVB(self->ninja->gps.GPS_home[1]);
	//TODO it'd be great if we could grab the GPS current time and write that too

	//gpsHistory.GPS_home[0] = self->ninja->gps.GPS_home[0];
	//gpsHistory.GPS_home[1] = self->ninja->gps.GPS_home[1];
}

static void writeGPSFrame(struct blackbox *self)
{
	struct gps *gps = &self->ninja->gps;
	blackboxWrite(self, 'G');

	/*
	 * If we're logging every frame, then a GPS frame always appears just after a frame with the
	 * currentTime timestamp in the log, so the reader can just use that timestamp for the GPS frame.
	 *
	 * If we're not logging every frame, we need to store the time of this GPS frame.
	 */
	if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_NOT_LOGGING_EVERY_FRAME)) {
		// Predict the time of the last frame in the main log
		blackboxWriteUnsignedVB(sys_micros(self->ninja->system) - blackboxHistory[1]->time);
	}

	blackboxWriteUnsignedVB(gps->GPS_numSat);
	blackboxWriteSignedVB(gps->GPS_coord[0] - gpsHistory.GPS_home[0]);
	blackboxWriteSignedVB(gps->GPS_coord[1] - gpsHistory.GPS_home[1]);
	blackboxWriteUnsignedVB(gps->GPS_altitude);
	blackboxWriteUnsignedVB(gps->GPS_speed);
	blackboxWriteUnsignedVB(gps->GPS_ground_course);

	gpsHistory.GPS_numSat = gps->GPS_numSat;
	gpsHistory.GPS_coord[0] = gps->GPS_coord[0];
	gpsHistory.GPS_coord[1] = gps->GPS_coord[1];
}
#endif // GPS

/**
 * Fill the current state of the blackbox using values read from the flight controller
 */
static void loadMainState(struct blackbox *self)
{
	blackboxMainState_t *blackboxCurrent = blackboxHistory[0];
	int i;

	blackboxCurrent->time = sys_micros(self->ninja->system);

	/* TODO: this kind of thing should be solved differently. If debugging is needed then debug other values, not intermediate calculations.
	const struct pid_controller_output *out = anglerate_get_output_ptr(&self->ninja->ctrl); 
	for (i = 0; i < XYZ_AXIS_COUNT; i++) {
		blackboxCurrent->axisPID_P[i] = out->axis_P[i];
	}
	for (i = 0; i < XYZ_AXIS_COUNT; i++) {
		blackboxCurrent->axisPID_I[i] = out->axis_I[i];
	}
	for (i = 0; i < XYZ_AXIS_COUNT; i++) {
		blackboxCurrent->axisPID_D[i] = out->axis_D[i];
	}
	*/

	for (i = 0; i < 4; i++) {
		blackboxCurrent->rcCommand[i] = rc_get_command(&self->ninja->rc, i);
	}

	blackboxCurrent->gyroADC[X] = ins_get_gyro_x(&self->ninja->ins);
	blackboxCurrent->gyroADC[Y] = ins_get_gyro_y(&self->ninja->ins);
	blackboxCurrent->gyroADC[Z] = ins_get_gyro_z(&self->ninja->ins);

	blackboxCurrent->accSmooth[X] = ins_get_acc_x(&self->ninja->ins);
	blackboxCurrent->accSmooth[Y] = ins_get_acc_y(&self->ninja->ins);
	blackboxCurrent->accSmooth[Z] = ins_get_acc_z(&self->ninja->ins);

	for (i = 0; i < mixer_get_motor_count(&self->ninja->mixer); i++) {
		// TODO: pwm direct input
		//blackboxCurrent->motor[i] = mixer_get_motor_value(&self->ninja->mixer, i);
	}

	blackboxCurrent->vbatLatest = battery_get_voltage(&self->ninja->bat);
	blackboxCurrent->amperageLatest = battery_get_current(&self->ninja->bat);

	if(USE_MAG){
		blackboxCurrent->magADC[X] = ins_get_mag_x(&self->ninja->ins);
		blackboxCurrent->magADC[Y] = ins_get_mag_y(&self->ninja->ins);
		blackboxCurrent->magADC[Z] = ins_get_mag_z(&self->ninja->ins);
	}

#ifdef BARO
	blackboxCurrent->BaroAlt = BaroAlt;
#endif

#ifdef SONAR
	// Store the raw sonar value without applying tilt correction
	blackboxCurrent->sonarRaw = sonar_read(&default_sonar);
#endif

	blackboxCurrent->rssi = rx_get_rssi(&self->ninja->rx);

#ifdef USE_SERVOS
	//Tail servo for tricopters
	// TODO: direct servo values
	//blackboxCurrent->servo[5] = mixer_get_servo_value(&self->ninja->mixer, 5);
#endif
}

/**
 * Transmit the header information for the given field definitions. Transmitted header lines look like:
 *
 * H Field I name:a,b,c
 * H Field I predictor:0,1,2
 *
 * For all header types, provide a "mainFrameChar" which is the name for the field and will be used to refer to it in the
 * header (e.g. P, I etc). For blackboxDeltaField_t fields, also provide deltaFrameChar, otherwise set this to zero.
 *
 * Provide an array 'conditions' of FlightLogFieldCondition enums if you want these conditions to decide whether a field
 * should be included or not. Otherwise provide NULL for this parameter and NULL for secondCondition.
 *
 * Set xmitState.headerIndex to 0 and xmitState.u.fieldIndex to -1 before calling for the first time.
 *
 * secondFieldDefinition and secondCondition element pointers need to be provided in order to compute the stride of the
 * fieldDefinition and secondCondition arrays.
 *
 * Returns true if there is still header left to transmit (so call again to continue transmission).
 */
static bool sendFieldDefinition(struct blackbox *self, char mainFrameChar, char deltaFrameChar, const void *fieldDefinitions,
		const void *secondFieldDefinition, int fieldCount, const uint8_t *conditions, const uint8_t *secondCondition)
{
	const blackboxFieldDefinition_t *def;
	unsigned int headerCount;
	static bool needComma = false;
	size_t definitionStride = (char*) secondFieldDefinition - (char*) fieldDefinitions;
	size_t conditionsStride = (char*) secondCondition - (char*) conditions;

	if (deltaFrameChar) {
		headerCount = BLACKBOX_DELTA_FIELD_HEADER_COUNT;
	} else {
		headerCount = BLACKBOX_SIMPLE_FIELD_HEADER_COUNT;
	}

	/*
	 * We're chunking up the header data so we don't exceed our datarate. So we'll be called multiple times to transmit
	 * the whole header.
	 */

	// On our first call we need to print the name of the header and a colon
	if (xmitState.u.fieldIndex == -1) {
		if (xmitState.headerIndex >= headerCount) {
			return false; //Someone probably called us again after we had already completed transmission
		}

		uint32_t charsToBeWritten = strlen("H Field x :") + strlen(blackboxFieldHeaderNames[xmitState.headerIndex]);

		if (blackboxDeviceReserveBufferSpace(charsToBeWritten) != BLACKBOX_RESERVE_SUCCESS) {
			return true; // Try again later
		}

		self->blackboxHeaderBudget -= blackboxPrintf("H Field %c %s:", xmitState.headerIndex >= BLACKBOX_SIMPLE_FIELD_HEADER_COUNT ? deltaFrameChar : mainFrameChar, blackboxFieldHeaderNames[xmitState.headerIndex]);

		xmitState.u.fieldIndex++;
		needComma = false;
	}

	// The longest we expect an integer to be as a string:
	const uint32_t LONGEST_INTEGER_STRLEN = 2;

	for (; xmitState.u.fieldIndex < fieldCount; xmitState.u.fieldIndex++) {
		def = (const blackboxFieldDefinition_t*) ((const char*)fieldDefinitions + definitionStride * xmitState.u.fieldIndex);

		if (!conditions || testBlackboxCondition(conditions[conditionsStride * xmitState.u.fieldIndex])) {
			// First (over)estimate the length of the string we want to print

			int32_t bytesToWrite = 1; // Leading comma

			// The first header is a field name
			if (xmitState.headerIndex == 0) {
				bytesToWrite += strlen(def->name) + strlen("[]") + LONGEST_INTEGER_STRLEN;
			} else {
				//The other headers are integers
				bytesToWrite += LONGEST_INTEGER_STRLEN;
			}

			// Now perform the write if the buffer is large enough
			if (blackboxDeviceReserveBufferSpace(bytesToWrite) != BLACKBOX_RESERVE_SUCCESS) {
				// Ran out of space!
				return true;
			}

			self->blackboxHeaderBudget -= bytesToWrite;

			if (needComma) {
				blackboxWrite(self, ',');
			} else {
				needComma = true;
			}

			// The first header is a field name
			if (xmitState.headerIndex == 0) {
				blackboxPrint(def->name);

				// Do we need to print an index in brackets after the name?
				if (def->fieldNameIndex != -1) {
					blackboxPrintf("[%d]", def->fieldNameIndex);
				}
			} else {
				//The other headers are integers
				blackboxPrintf("%d", def->arr[xmitState.headerIndex - 1]);
			}
		}
	}

	// Did we complete this line?
	if (xmitState.u.fieldIndex == fieldCount && blackboxDeviceReserveBufferSpace(1) == BLACKBOX_RESERVE_SUCCESS) {
		self-<blackboxHeaderBudget--;
		blackboxWrite(self, '\n');
		xmitState.headerIndex++;
		xmitState.u.fieldIndex = -1;
	}

	return xmitState.headerIndex < headerCount;
}

/**
 * Transmit a portion of the system information headers. Call the first time with xmitState.headerIndex == 0. Returns
 * true iff transmission is complete, otherwise call again later to continue transmission.
 */
static bool blackboxWriteSysinfo(struct blackbox *self){
	// Make sure we have enough room in the buffer for our longest line (as of this writing, the "Firmware date" line)
	if (blackboxDeviceReserveBufferSpace(64) != BLACKBOX_RESERVE_SUCCESS) {
		return false;
	}

	switch (xmitState.headerIndex) {
		case 0:
			blackboxPrintfHeaderLine("Firmware type:Ninjaflight");
		break;
		case 1:
			blackboxPrintfHeaderLine("Firmware revision:%s", shortGitRevision);
		break;
		case 2:
			blackboxPrintfHeaderLine("Firmware date:%s %s", buildDate, buildTime);
		break;
		case 3:
			blackboxPrintfHeaderLine("P interval:%d/%d", self->config->blackbox.rate_num, self->config->blackbox.rate_denom);
		break;
		case 4:
			blackboxPrintfHeaderLine("Device UID:0x%x%x%x", U_ID_0, U_ID_1, U_ID_2);
		break;
		case 5:
			blackboxPrintfHeaderLine("rcRate:%d", config_get_rate_profile(self->config)->rcRate8);
		break;
		case 6:
			blackboxPrintfHeaderLine("minthrottle:%d", self->config->pwm_out.minthrottle);
		break;
		case 7:
			blackboxPrintfHeaderLine("maxthrottle:%d", self->config->pwm_out.maxthrottle);
		break;
		case 8:
			// TODO: blackbox gyro scale and acc1g
			//blackboxPrintfHeaderLine("gyro.scale:0x%x", castFloatBytesToInt(gyro.scale));
		break;
		case 9:
			//blackboxPrintfHeaderLine("acc_1G:%u", acc.acc_1G);
		break;
		case 10:
			if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_VBAT)) {
				blackboxPrintfHeaderLine("vbatscale:%u", self->config->bat.vbatscale);
			} else {
				xmitState.headerIndex += 2; // Skip the next two vbat fields too
			}
		break;
		case 11:
			blackboxPrintfHeaderLine("vbatcellvoltage:%u,%u,%u", self->config->bat.vbatmincellvoltage,
				self->config->bat.vbatwarningcellvoltage, self->config->bat.vbatmaxcellvoltage);
		break;
		case 12:
			blackboxPrintfHeaderLine("vbatref:%u", vbatReference);
		break;
		case 13:
			//Note: Log even if this is a virtual current meter, since the virtual meter uses these parameters too:
			if (feature(FEATURE_CURRENT_METER)) {
				blackboxPrintfHeaderLine("currentMeter:%d,%d", self->config->bat.currentMeterOffset, self->config->bat.currentMeterScale);
			}
		break;
		default:
			return true;
	}

	xmitState.headerIndex++;
	return false;
}

/**
 * Write the given event to the log immediately
 */
void blackboxLogEvent(struct blackbox *self, FlightLogEvent event, flightLogEventData_t *data)
{
	// Only allow events to be logged after headers have been written
	if (!(self->blackboxState == BLACKBOX_STATE_RUNNING || blackboxState == BLACKBOX_STATE_PAUSED)) {
		return;
	}

	//Shared header for event frames
	blackboxWrite(self, 'E');
	blackboxWrite(self, event);

	//Now serialize the data for this specific frame type
	switch (event) {
		case FLIGHT_LOG_EVENT_SYNC_BEEP:
			blackboxWriteUnsignedVB(data->syncBeep.time);
		break;
		case FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT:
			if (data->inflightAdjustment.floatFlag) {
				blackboxWrite(data->inflightAdjustment.adjustmentFunction + FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT_FUNCTION_FLOAT_VALUE_FLAG);
				blackboxWriteFloat(data->inflightAdjustment.newFloatValue);
			} else {
				blackboxWrite(data->inflightAdjustment.adjustmentFunction);
				blackboxWriteSignedVB(data->inflightAdjustment.newValue);
			}
		case FLIGHT_LOG_EVENT_GTUNE_RESULT:
			blackboxWrite(data->gtuneCycleResult.gtuneAxis);
			blackboxWriteSignedVB(data->gtuneCycleResult.gtuneGyroAVG);
			blackboxWriteS16(data->gtuneCycleResult.gtuneNewP);
		break;
		case FLIGHT_LOG_EVENT_LOGGING_RESUME:
			blackboxWriteUnsignedVB(data->loggingResume.logIteration);
			blackboxWriteUnsignedVB(data->loggingResume.currentTime);
		break;
		case FLIGHT_LOG_EVENT_LOG_END:
			blackboxPrint("End of log");
			blackboxWrite(0);
		break;
		default:
			break;
	}
}

/* If an arming beep has played since it was last logged, write the time of the arming beep to the log as a synchronization point */
static void blackboxCheckAndLogArmingBeep(void)
{
/*
 *  // TODO: blackbox arming beep crap
	flightLogEvent_syncBeep_t eventData;

	// Use != so that we can still detect a change if the counter wraps
	if (getArmingBeepTimeMicros() != blackboxLastArmingBeep) {
		blackboxLastArmingBeep = getArmingBeepTimeMicros();

		eventData.time = blackboxLastArmingBeep;

		blackboxLogEvent(FLIGHT_LOG_EVENT_SYNC_BEEP, (flightLogEventData_t *) &eventData);
	}
	*/
}

/* 
 * Use the user's num/denom settings to decide if the P-frame of the given index should be logged, allowing the user to control
 * the portion of logged loop iterations.
 */
static bool blackboxShouldLogPFrame(struct blackbox *self, uint32_t pFrameIndex)
{
	/* Adding a magic shift of "self->config->rate_num - 1" in here creates a better spread of
	 * recorded / skipped frames when the I frame's position is considered:
	 */
	return (pFrameIndex + self->config->blackbox.rate_num - 1) % self->config->blackbox.rate_denom < self->config->blackbox.rate_num;
}

static bool blackboxShouldLogIFrame(void)
{
	return blackboxPFrameIndex == 0;
}

// Called once every FC loop in order to keep track of how many FC loop iterations have passed
static void blackboxAdvanceIterationTimers(void)
{
	blackboxSlowFrameIterationTimer++;
	blackboxIteration++;
	blackboxPFrameIndex++;

	if (blackboxPFrameIndex == BLACKBOX_I_INTERVAL) {
		blackboxPFrameIndex = 0;
		blackboxIFrameIndex++;
	}
}

// Called once every FC loop in order to log the current state
static void blackboxLogIteration(struct blackbox *self)
{
	// Write a keyframe every BLACKBOX_I_INTERVAL frames so we can resynchronise upon missing frames
	if (blackboxShouldLogIFrame()) {
		/*
		 * Don't log a slow frame if the slow data didn't change ("I" frames are already large enough without adding
		 * an additional item to write at the same time). Unless we're *only* logging "I" frames, then we have no choice.
		 */
		writeSlowFrameIfNeeded(self, blackboxIsOnlyLoggingIntraframes(self));

		loadMainState(self);
		writeIntraframe(self);
	} else {
		blackboxCheckAndLogArmingBeep();
		
		if (blackboxShouldLogPFrame(self, blackboxPFrameIndex)) {
			/*
			 * We assume that slow frames are only interesting in that they aid the interpretation of the main data stream.
			 * So only log slow frames during loop iterations where we log a main frame.
			 */
			writeSlowFrameIfNeeded(self, true);

			loadMainState(self);
			writeInterframe(self);
		}
#ifdef GPS
		if (feature(FEATURE_GPS)) {
			/*
			 * If the GPS home point has been updated, or every 128 intraframes (~10 seconds), write the
			 * GPS home position.
			 *
			 * We write it periodically so that if one Home Frame goes missing, the GPS coordinates can
			 * still be interpreted correctly.
			 */
			if (GPS_home[0] != gpsHistory.GPS_home[0] || GPS_home[1] != gpsHistory.GPS_home[1]
				|| (blackboxPFrameIndex == BLACKBOX_I_INTERVAL / 2 && blackboxIFrameIndex % 128 == 0)) {

				writeGPSHomeFrame(self);
				writeGPSFrame(self);
				// TODO: accessing gps data directly is not good
			} else if (self->ninja->gps.GPS_numSat != gpsHistory.GPS_numSat || self->ninja->gps.GPS_coord[0] != gpsHistory.GPS_coord[0]
					|| self->ninja->gps.GPS_coord[1] != gpsHistory.GPS_coord[1]) {
				//We could check for velocity changes as well but I doubt it changes independent of position
				writeGPSFrame(self);
			}
		}
#endif
	}

	//Flush every iteration so that our runtime variance is minimized
	blackboxDeviceFlush();
}

/**
 * Call each flight loop iteration to perform blackbox logging.
 */
void blackbox_update(struct blackbox *self)
{
	int i;

	if (self->blackboxState >= BLACKBOX_FIRST_HEADER_SENDING_STATE && self->blackboxState <= BLACKBOX_LAST_HEADER_SENDING_STATE) {
		blackboxReplenishHeaderBudget();
	}

	switch (self->blackboxState) {
		case BLACKBOX_STATE_PREPARE_LOG_FILE:
			if (blackboxDeviceBeginLog()) {
				blackboxSetState(self, BLACKBOX_STATE_SEND_HEADER);
			}
		break;
		case BLACKBOX_STATE_SEND_HEADER:
			//On entry of this state, xmitState.headerIndex is 0 and startTime is intialised

			/*
			 * Once the UART has had time to init, transmit the header in chunks so we don't overflow its transmit
			 * buffer, overflow the OpenLog's buffer, or keep the main loop busy for too long.
			 */
			if (sys_millis(self->ninja->system) > xmitState.u.startTime + 100) {
				if (blackboxDeviceReserveBufferSpace(BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION) == BLACKBOX_RESERVE_SUCCESS) {
					for (i = 0; i < BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION && blackboxHeader[xmitState.headerIndex] != '\0'; i++, xmitState.headerIndex++) {
						blackboxWrite(blackboxHeader[xmitState.headerIndex]);
						self->blackboxHeaderBudget--;
					}

					if (blackboxHeader[xmitState.headerIndex] == '\0') {
						blackboxSetState(self, BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER);
					}
				}
			}
		break;
		case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER:
			//On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
			if (!sendFieldDefinition('I', 'P', blackboxMainFields, blackboxMainFields + 1, ARRAY_LENGTH(blackboxMainFields),
					&blackboxMainFields[0].condition, &blackboxMainFields[1].condition)) {
				if (feature(FEATURE_GPS)) {
					blackboxSetState(self, BLACKBOX_STATE_SEND_GPS_H_HEADER);
				} else
					blackboxSetState(self, BLACKBOX_STATE_SEND_SLOW_HEADER);
			}
		break;
		case BLACKBOX_STATE_SEND_GPS_H_HEADER:
			//On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
			if (!sendFieldDefinition('H', 0, blackboxGpsHFields, blackboxGpsHFields + 1, ARRAY_LENGTH(blackboxGpsHFields),
					NULL, NULL)) {
				blackboxSetState(self, BLACKBOX_STATE_SEND_GPS_G_HEADER);
			}
		break;
		case BLACKBOX_STATE_SEND_GPS_G_HEADER:
			//On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
			if (!sendFieldDefinition('G', 0, blackboxGpsGFields, blackboxGpsGFields + 1, ARRAY_LENGTH(blackboxGpsGFields),
					&blackboxGpsGFields[0].condition, &blackboxGpsGFields[1].condition)) {
				blackboxSetState(self, BLACKBOX_STATE_SEND_SLOW_HEADER);
			}
		break;
		case BLACKBOX_STATE_SEND_SLOW_HEADER:
			//On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
			if (!sendFieldDefinition('S', 0, blackboxSlowFields, blackboxSlowFields + 1, ARRAY_LENGTH(blackboxSlowFields),
					NULL, NULL)) {
				blackboxSetState(self, BLACKBOX_STATE_SEND_SYSINFO);
			}
		break;
		case BLACKBOX_STATE_SEND_SYSINFO:
			//On entry of this state, xmitState.headerIndex is 0

			//Keep writing chunks of the system info headers until it returns true to signal completion
			if (blackboxWriteSysinfo(self)) {

				/*
				 * Wait for header buffers to drain completely before data logging begins to ensure reliable header delivery
				 * (overflowing circular buffers causes all data to be discarded, so the first few logged iterations
				 * could wipe out the end of the header if we weren't careful)
				 */
				if (blackboxDeviceFlushForce()) {
					blackboxSetState(self, BLACKBOX_STATE_RUNNING);
				}
			}
		break;
		case BLACKBOX_STATE_PAUSED:
			// Only allow resume to occur during an I-frame iteration, so that we have an "I" base to work from
			if (blackboxShouldLogIFrame()) {
				// Write a log entry so the decoder is aware that our large time/iteration skip is intended
				flightLogEvent_loggingResume_t resume;

				resume.logIteration = blackboxIteration;
				resume.currentTime = sys_micros(self->ninja->system);

				blackboxLogEvent(FLIGHT_LOG_EVENT_LOGGING_RESUME, (flightLogEventData_t *) &resume);
				blackboxSetState(self, BLACKBOX_STATE_RUNNING);
				
				blackboxLogIteration(self);
			}

			// Keep the logging timers ticking so our log iteration continues to advance
			blackboxAdvanceIterationTimers();
		break;
		case BLACKBOX_STATE_RUNNING:
			// On entry to this state, blackboxIteration, blackboxPFrameIndex and blackboxIFrameIndex are reset to 0
		  	blackboxLogIteration(self);

			blackboxAdvanceIterationTimers();
		break;
		case BLACKBOX_STATE_SHUTTING_DOWN:
			//On entry of this state, startTime is set

			/*
			 * Wait for the log we've transmitted to make its way to the logger before we release the serial port,
			 * since releasing the port clears the Tx buffer.
			 *
			 * Don't wait longer than it could possibly take if something funky happens.
			 */
			if (blackboxDeviceEndLog(blackboxLoggedAnyFrames) && (sys_millis(self->ninja->system) > xmitState.u.startTime + BLACKBOX_SHUTDOWN_TIMEOUT_MILLIS || blackboxDeviceFlushForce())) {
				blackboxDeviceClose();
				blackboxSetState(self, BLACKBOX_STATE_STOPPED);
			}
		break;
		default:
		case BLACKBOX_STATE_DISABLED:
		case BLACKBOX_STATE_STOPPED:
		break;
	}

	// Did we run out of room on the device? Stop!
	if (isBlackboxDeviceFull()) {
		blackboxSetState(self, BLACKBOX_STATE_STOPPED);
	}
}
/**
 * Call during system startup to initialize the blackbox.
 */
void blackbox_init(struct blackbox *self, struct ninja *ninja, const struct config * const config){
	memset(self, 0, sizeof(*self));

	self->ninja = ninja;
	self->config = config;

	self->blackboxState = BLACKBOX_STATE_DISABLED;

	blackboxSetState(self, BLACKBOX_STATE_STOPPED);
}
#endif
