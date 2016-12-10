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
#include <string.h>
#include <math.h>

#include "build_config.h"
#include "debug.h"
#include <platform.h>

#include "common/axis.h"
#include "common/utils.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/streambuf.h"

#include "config/config.h"
#include "config/feature.h"
#include "config/profile.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/serial.h"
#include "drivers/bus_i2c.h"
#include "drivers/sdcard.h"
#include "drivers/flashfs.h"
#include "common/buf_writer.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "flight/rate_profile.h"
#include "io/rc_adjustments.h"
#include "io/serial.h"
#include "io/ledstrip.h"
#include "io/transponder_ir.h"
#include "io/msp_protocol.h"
#include "io/serial_msp.h"
#include "io/serial_4way.h"

//#include "telemetry/telemetry.h"

#include "sensors/boardalignment.h"
#include "sensors/battery.h"
#include "sensors/gps.h"
#include "sensors/sonar.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/instruments.h"

#include "flight/mixer.h"
#include "flight/anglerate.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"
#include "flight/altitudehold.h"

#include "blackbox/blackbox.h"

#include "ninja_config.h"
#include "ninjaflight.h"

#include "version.h"
#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#include "msp.h"

extern void resetPidProfile(struct pid_config *pidProfile);

static const char * const flightControllerIdentifier = CLEANFLIGHT_IDENTIFIER; // 4 UPPER CASE alpha numeric characters that identify the flight controller.
static const char * const boardIdentifier = TARGET_BOARD_IDENTIFIER;

typedef struct box_e {
    const char *boxName;            // GUI-readable box name
    const uint8_t boxId;            // see boxId_e (it is equal to table index, may be optimized)
    const uint8_t permanentId;      // ID for MSP protocol
} box_t;

static const box_t boxes[CHECKBOX_ITEM_COUNT] = {
    { "ARM",       BOXARM,        0 },
    { "ANGLE",     BOXANGLE,      1 },
    { "HORIZON",   BOXHORIZON,    2 },
    { "BARO",      BOXBARO,       3 },
  //{ "VARIO",     BOXVARIO,      4 },
    { "MAG",       BOXMAG,        5 },
    { "HEADFREE",  BOXHEADFREE,   6 },
    { "HEADADJ",   BOXHEADADJ,    7 },
    { "CAMSTAB",   BOXCAMSTAB,    8 },
    { "CAMTRIG",   BOXCAMTRIG,    9 },
    { "GPS HOME",  BOXGPSHOME,   10 },
    { "GPS HOLD",  BOXGPSHOLD,   11 },
    { "PASSTHRU",  BOXPASSTHRU,  12 },
    { "BEEPER",    BOXBEEPERON,  13 },
    { "LEDMAX",    BOXLEDMAX,    14 },
    { "LEDLOW",    BOXLEDLOW,    15 },
    { "LLIGHTS",   BOXLLIGHTS,   16 },
    { "CALIB",     BOXCALIB,     17 },
    { "GOVERNOR",  BOXGOV,       18 },
    { "OSD SW",    BOXOSD,       19 },
    { "TELEMETRY", BOXTELEMETRY, 20 },
    { "GTUNE",     BOXGTUNE,     21 },
    { "SONAR",     BOXSONAR,     22 },
    { "SERVO1",    BOXSERVO1,    23 },
    { "SERVO2",    BOXSERVO2,    24 },
    { "SERVO3",    BOXSERVO3,    25 },
    { "BLACKBOX",  BOXBLACKBOX,  26 },
    { "FAILSAFE",  BOXFAILSAFE,  27 },
    { "AIR MODE",  BOXAIRMODE,   28 },
};

static const char pidnames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "ALT;"
    "Pos;"
    "PosR;"
    "NavR;"
    "LEVEL;"
    "MAG;"
    "VEL;";

typedef enum {
    MSP_SDCARD_STATE_NOT_PRESENT = 0,
    MSP_SDCARD_STATE_FATAL       = 1,
    MSP_SDCARD_STATE_CARD_INIT   = 2,
    MSP_SDCARD_STATE_FS_INIT     = 3,
    MSP_SDCARD_STATE_READY       = 4,
} mspSDCardState_e;

typedef enum {
    MSP_SDCARD_FLAG_SUPPORTTED   = 1,
} mspSDCardFlags_e;

typedef enum {
    MSP_FLASHFS_BIT_READY        = 1,
    MSP_FLASHFS_BIT_SUPPORTED    = 2,
} mspFlashfsFlags_e;

static const box_t *findBoxByBoxId(uint8_t boxId)
{
    for (unsigned i = 0; i < ARRAYLEN(boxes); i++) {
        const box_t *candidate = &boxes[i];
        if (candidate->boxId == boxId) {
            return candidate;
        }
    }
    return NULL;
}

static const box_t *findBoxByPermenantId(uint8_t permanentId)
{
    for (unsigned i = 0; i < ARRAYLEN(boxes); i++) {
        const box_t *candidate = &boxes[i];
        if (candidate->permanentId == permanentId) {
            return candidate;
        }
    }
    return NULL;
}

static void serializeBoxNamesReply(struct msp *self, mspPacket_t *reply)
{
    sbuf_t *dst = &reply->buf;
    for (int i = 0; i < CHECKBOX_ITEM_COUNT; i++) {
        if(!(self->activeBoxIds & (1 << i)))
            continue;                          // box is not enabled
        const box_t *box = findBoxByBoxId(i);
        sbufWriteString(dst, box->boxName);
        sbufWriteU8(dst, ';');                 // TODO - sbufWriteChar?
    }
}

static void serializeBoxIdsReply(struct msp *self, mspPacket_t *reply)
{
    sbuf_t *dst = &reply->buf;
    for (int i = 0; i < CHECKBOX_ITEM_COUNT; i++) {
        if(!(self->activeBoxIds & (1 << i)))
            continue;
        const box_t *box = findBoxByBoxId(i);
        sbufWriteU8(dst, box->permanentId);
    }
}

static void initActiveBoxIds(struct msp *self)
{
    uint32_t ena = 0;

    ena |= 1 << BOXARM;
	ena |= 1 << BOXANGLE;
    ena |= 1 << BOXHORIZON;

	// TODO: BOXANGLE and BOXHORIZON
	/*
    if (ninja_has_sensor(ninja, SENSOR_ACC)) {
        ena |= 1 << BOXANGLE;
        ena |= 1 << BOXHORIZON;
    }

#ifdef BARO
    if (ninja_has_sensor(ninja, SENSOR_BARO)) {
        ena |= 1 << BOXBARO;
    }
#endif
*/
    ena |= 1 << BOXAIRMODE;
/*
    if (ninja_has_sensor(ninja, SENSOR_ACC) || ninja_has_sensor(ninja, SENSOR_MAG)) {
        ena |= 1 << BOXMAG;
        ena |= 1 << BOXHEADFREE;
        ena |= 1 << BOXHEADADJ;
    }
*/
    if (feature(self->config, FEATURE_SERVO_TILT))
        ena |= 1 << BOXCAMSTAB;

#ifdef GPS
    if (feature(self->config, FEATURE_GPS)) {
        ena |= 1 << BOXGPSHOME;
        ena |= 1 << BOXGPSHOLD;
    }
#endif

    if (self->config->mixer.mixerMode == MIXER_FLYING_WING
        || self->config->mixer.mixerMode == MIXER_AIRPLANE
        || self->config->mixer.mixerMode == MIXER_CUSTOM_AIRPLANE)
        ena |= 1 << BOXPASSTHRU;

    ena |= 1 << BOXBEEPERON;

#ifdef LED_STRIP
    if (feature(self->config, FEATURE_LED_STRIP)) {
        ena |= 1 << BOXLEDLOW;
    }
#endif

    if (feature(self->config, FEATURE_INFLIGHT_ACC_CAL))
        ena |= 1 << BOXCALIB;

    ena |= 1 << BOXOSD;

#ifdef TELEMETRY
    if (feature(self->config, FEATURE_TELEMETRY) && self->config->telemetry.telemetry_switch)
        ena |= 1 << BOXTELEMETRY;
#endif

    if (feature(self->config, FEATURE_SONAR)){
        ena |= 1 << BOXSONAR;
    }

#ifdef USE_SERVOS
    if (self->config->mixer.mixerMode == MIXER_CUSTOM_AIRPLANE) {
        ena |= 1 << BOXSERVO1;
        ena |= 1 << BOXSERVO2;
        ena |= 1 << BOXSERVO3;
    }
#endif

    if (USE_BLACKBOX && feature(self->config, FEATURE_BLACKBOX)){
        ena |= 1 << BOXBLACKBOX;
    }

    if (feature(self->config, FEATURE_FAILSAFE)){
        ena |= 1 << BOXFAILSAFE;
    }

#ifdef GTUNE
    ena |= 1 << BOXGTUNE;
#endif

    // check that all enabled IDs are in boxes array (check is skipped when using findBoxBy<id>() functions
    for(boxId_e boxId = 0;  boxId < CHECKBOX_ITEM_COUNT; boxId++)
        if((ena & (1 << boxId))
           && findBoxByBoxId(boxId) == NULL)
            ena &= ~ (1 << boxId);                // this should not happen, but handle it gracefully
    self->activeBoxIds = ena;
}

#define IS_ENABLED(mask) (mask == 0 ? 0 : 1)

static uint32_t packFlightModeFlags(struct msp *self)
{
    // Serialize the flags in the order we delivered them, ignoring BOXNAMES and BOXINDEXES
    // Requires new Multiwii protocol version to fix
    // It would be preferable to setting the enabled bits based on BOXINDEX.

	// TODO: pack flight mode flags MSP
    uint32_t boxEnabledMask = 0;      // enabled BOXes, bits indexed by boxId_e

    // enable BOXes dependent on FLIGHT_MODE, use mapping table
	/*
    static const int8_t flightMode_boxId_map[] = FLIGHT_MODE_BOXID_MAP_INITIALIZER;
    flightModeFlags_e flightModeCopyMask = ~(GTUNE_MODE);  // BOXGTUNE is based on rcMode, not flight mode
    for(unsigned i = 0; i < ARRAYLEN(flightMode_boxId_map); i++) {
        if(flightMode_boxId_map[i] == -1)
            continue;                 // boxId_e does not exist for this FLIGHT_MODE
        if((flightModeCopyMask & (1 << i)) == 0)
            continue;                 // this flightmode is not copied
        if(FLIGHT_MODE(1 << i))
            boxEnabledMask |= 1 << flightMode_boxId_map[i];
    }
	*/
	if(rc_key_state(&self->ninja->rc, RC_KEY_FUNC_LEVEL) == RC_KEY_PRESSED) boxEnabledMask |= (1 << BOXANGLE);
	if(rc_key_state(&self->ninja->rc, RC_KEY_FUNC_BLEND) == RC_KEY_PRESSED) boxEnabledMask |= (1 << BOXHORIZON);
/*
    // enable BOXes dependent on rcMode bits, indexes are the same.
    // only subset of BOXes depend on rcMode, use mask to mark them
#define BM(x) (1 << (x))
    const uint32_t rcModeCopyMask = BM(BOXHEADADJ) | BM(BOXCAMSTAB) | BM(BOXCAMTRIG) | BM(BOXBEEPERON)
        | BM(BOXLEDMAX) | BM(BOXLEDLOW) | BM(BOXLLIGHTS) | BM(BOXCALIB) | BM(BOXGOV) | BM(BOXOSD)
        | BM(BOXTELEMETRY) | BM(BOXGTUNE) | BM(BOXBLACKBOX)  | BM(BOXAIRMODE) ;
    for(unsigned i = 0; i < sizeof(rcModeCopyMask) * 8; i++) {
        if((rcModeCopyMask & BM(i)) == 0)
            continue;
        if(rc_key_(i))
            boxEnabledMask |= 1 << i;
    }
#undef BM
    // copy ARM state
    if(ARMING_FLAG(ARMED))
        boxEnabledMask |= 1 << BOXARM;
*/
    // map boxId_e enabled bits to MSP status indexes
    // only active boxIds are sent in status over MSP, other bits are not counted
    uint32_t mspBoxEnabledMask = 0;
    unsigned mspBoxIdx = 0;           // index of active boxId (matches sent permanentId and boxNames)
    for (boxId_e boxId = 0; boxId < CHECKBOX_ITEM_COUNT; boxId++) {
        if((self->activeBoxIds & (1 << boxId)) == 0)
            continue;                 // this box is not active
        if (boxEnabledMask & (1 << boxId))
            mspBoxEnabledMask |= 1 << mspBoxIdx;      // box is enabled
        mspBoxIdx++;                  // next output bit ID
    }
    return mspBoxEnabledMask;
}

static void serializeSDCardSummaryReply(mspPacket_t *reply)
{
    sbuf_t *dst = &reply->buf;
#ifdef USE_SDCARD
    uint8_t flags = MSP_SDCARD_FLAG_SUPPORTTED;
    uint8_t state;

    sbufWriteU8(dst, flags);

    // Merge the card and filesystem states together
    if (!sdcard_isInserted()) {
        state = MSP_SDCARD_STATE_NOT_PRESENT;
    } else if (!sdcard_isFunctional()) {
        state = MSP_SDCARD_STATE_FATAL;
    } else {
        switch (afatfs_getFilesystemState()) {
            case AFATFS_FILESYSTEM_STATE_READY:
                state = MSP_SDCARD_STATE_READY;
                break;
            case AFATFS_FILESYSTEM_STATE_INITIALIZATION:
                if (sdcard_isInitialized()) {
                    state = MSP_SDCARD_STATE_FS_INIT;
                } else {
                    state = MSP_SDCARD_STATE_CARD_INIT;
                }
                break;
            case AFATFS_FILESYSTEM_STATE_FATAL:
            case AFATFS_FILESYSTEM_STATE_UNKNOWN:
            default:
                state = MSP_SDCARD_STATE_FATAL;
                break;
        }
    }

    sbufWriteU8(dst, state);
    sbufWriteU8(dst, afatfs_getLastError());
    // Write free space and total space in kilobytes
    sbufWriteU32(dst, afatfs_getContiguousFreeSpace() / 1024);
    sbufWriteU32(dst, sdcard_getMetadata()->numBlocks / 2); // Block size is half a kilobyte
#else
    sbufWriteU8(dst, 0);
    sbufWriteU8(dst, 0);
    sbufWriteU8(dst, 0);
    sbufWriteU32(dst, 0);
    sbufWriteU32(dst, 0);
#endif
}

static void serializeDataflashSummaryReply(mspPacket_t *reply)
{
    sbuf_t *dst = &reply->buf;
#ifdef USE_FLASHFS
    const flashGeometry_t *geometry = flashfsGetGeometry();
    uint8_t flags = (flashfsIsReady() ? MSP_FLASHFS_BIT_READY : 0) | MSP_FLASHFS_BIT_SUPPORTED;

    sbufWriteU8(dst, flags);
    sbufWriteU32(dst, geometry->sectors);
    sbufWriteU32(dst, geometry->totalSize);
    sbufWriteU32(dst, flashfsGetOffset()); // Effectively the current number of bytes stored on the volume
#else
    sbufWriteU8(dst, 0); // FlashFS is neither ready nor supported
    sbufWriteU32(dst, 0);
    sbufWriteU32(dst, 0);
    sbufWriteU32(dst, 0);
#endif
}

#ifdef USE_FLASHFS
static void serializeDataflashReadReply(mspPacket_t *reply, uint32_t address, int size)
{
    sbuf_t *dst = &reply->buf;
    sbufWriteU32(dst, address);
    size = MIN(size, sbufBytesRemaining(dst));    // limit reply to available buffer space
    // bytesRead will be lower than that requested if we reach end of volume
    int bytesRead = flashfsReadAbs(address, sbufPtr(dst), size);
    sbufAdvance(dst, bytesRead);
}
#endif

// process commands that match registered parameter_group
/*
static int processPgCommand(mspPacket_t *command, mspPacket_t *reply)
{
    sbuf_t *src = &command->buf;
    sbuf_t *dst = &reply->buf;
    int cmdLength = sbufBytesRemaining(src);

    // MSP IN - read config structure
    {
        const pgRegistry_t *reg = pgMatcher(pgMatcherForMSP, (void*)(intptr_t)command->cmd);
        if (reg) {
            // this works for system and profile settings as
            //  the profile index will be ignored by pgLoad if the reg is a system registration.
            int stored = pgStore(reg, sbufPtr(dst), sbufBytesRemaining(dst), getCurrentProfile());
            if(stored < 0)
                return stored;
            sbufAdvance(dst, stored);       // commit saved data
            return 1;
        }
    }
    // MSP OUT - write config structure
    {
        const pgRegistry_t *reg = pgMatcher(pgMatcherForMSPSet, (void*)(intptr_t)command->cmd);
        if (reg != NULL) {
            // this works for system and profile settings as
            //  the profile index will be ignored by pgLoad if the reg is a system registration.
            pgLoad(reg, sbufPtr(src), cmdLength, getCurrentProfile());
            sbufAdvance(src, cmdLength);    // consume data explicitly
            return 1;
        }
    }
    return 0;
}
*/
static int processOutCommand(struct msp *self, mspPacket_t *cmd, mspPacket_t *reply)
{
    sbuf_t *dst = &reply->buf;
    sbuf_t *src = &cmd->buf;
    UNUSED(src);

    switch (cmd->cmd) {
        case MSP_API_VERSION:
            sbufWriteU8(dst, MSP_PROTOCOL_VERSION);

            sbufWriteU8(dst, API_VERSION_MAJOR);
            sbufWriteU8(dst, API_VERSION_MINOR);
            break;

        case MSP_FC_VARIANT:
            sbufWriteData(dst, flightControllerIdentifier, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
            break;

        case MSP_FC_VERSION:
            sbufWriteU8(dst, FC_VERSION_MAJOR);
            sbufWriteU8(dst, FC_VERSION_MINOR);
            sbufWriteU8(dst, FC_VERSION_PATCH_LEVEL);
            break;

        case MSP_BOARD_INFO:
            sbufWriteData(dst, boardIdentifier, BOARD_IDENTIFIER_LENGTH);
#ifdef USE_HARDWARE_REVISION_DETECTION
            sbufWriteU16(dst, hardwareRevision);
#else
            sbufWriteU16(dst, 0); // No hardware revision available.
#endif
            break;

        case MSP_BUILD_INFO:
            sbufWriteData(dst, buildDate, BUILD_DATE_LENGTH);
            sbufWriteData(dst, buildTime, BUILD_TIME_LENGTH);
            sbufWriteData(dst, shortGitRevision, GIT_SHORT_REVISION_LENGTH);
            break;

            // DEPRECATED - Use MSP_API_VERSION
        case MSP_IDENT:
            sbufWriteU8(dst, MW_VERSION);
            sbufWriteU8(dst, self->config->mixer.mixerMode);
            sbufWriteU8(dst, MSP_PROTOCOL_VERSION);
            sbufWriteU32(dst, CAP_DYNBALANCE); // "capability"
            break;

        case MSP_STATUS_EX:
        case MSP_STATUS:
            sbufWriteU16(dst, self->ninja->cycleTime);
#ifdef USE_I2C
            sbufWriteU16(dst, i2cGetErrorCounter());
#else
            sbufWriteU16(dst, 0);
#endif
            sbufWriteU16(dst,
				ninja_has_sensors(self->ninja, NINJA_SENSOR_ACC) |
				ninja_has_sensors(self->ninja, NINJA_SENSOR_BARO) << 1 |
				ninja_has_sensors(self->ninja, NINJA_SENSOR_MAG) << 2 |
				ninja_has_sensors(self->ninja, NINJA_SENSOR_GPS) << 3 |
				ninja_has_sensors(self->ninja, NINJA_SENSOR_SONAR) << 4);
            sbufWriteU32(dst, packFlightModeFlags(self));
            sbufWriteU8(dst, self->config->profile.profile_id);
            if(cmd->cmd == MSP_STATUS_EX) {
                sbufWriteU16(dst, self->ninja->sched.averageSystemLoadPercent);
            }
            break;

        case MSP_RAW_IMU: {
            // Hack scale due to choice of units for sensor data in multiwii
            unsigned scale_shift = (SYSTEM_ACC_1G > 1024) ? 3 : 0;
			sbufWriteU16(dst, ins_get_acc_x(&self->ninja->ins) >> scale_shift);
			sbufWriteU16(dst, ins_get_acc_y(&self->ninja->ins) >> scale_shift);
			sbufWriteU16(dst, ins_get_acc_z(&self->ninja->ins) >> scale_shift);
			sbufWriteU16(dst, ins_get_gyro_x(&self->ninja->ins));
			sbufWriteU16(dst, ins_get_gyro_y(&self->ninja->ins));
			sbufWriteU16(dst, ins_get_gyro_z(&self->ninja->ins));
            sbufWriteU16(dst, ins_get_mag_x(&self->ninja->ins));
            sbufWriteU16(dst, ins_get_mag_y(&self->ninja->ins));
            sbufWriteU16(dst, ins_get_mag_z(&self->ninja->ins));
            break;
        }

#ifdef USE_SERVOS
        case MSP_SERVO:
			for(int c = 0; c < MIXER_MAX_SERVOS; c++){
				sbufWriteU16(dst, self->ninja->direct_outputs[MIXER_OUTPUT_SERVOS + c]);
			}
            break;

        case MSP_SERVO_CONFIGURATIONS:
            for (unsigned i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
				const struct servo_config *servo = &config_get_profile(self->config)->servos.servoConf[i];
                sbufWriteU16(dst, servo->min);
                sbufWriteU16(dst, servo->max);
                sbufWriteU16(dst, servo->middle);
                sbufWriteU8(dst, servo->rate);
                sbufWriteU8(dst, servo->angleAtMin);
                sbufWriteU8(dst, servo->angleAtMax);
                sbufWriteU8(dst, servo->forwardFromChannel);
                sbufWriteU32(dst, servo->reversedSources);
            }
            break;
		// TODO: servo mix rules
/*
        case MSP_SERVO_MIX_RULES:
            for (unsigned i = 0; i < MAX_SERVO_RULES; i++) {
                sbufWriteU8(dst, customServoMixer(i)->targetChannel);
                sbufWriteU8(dst, customServoMixer(i)->inputSource);
                sbufWriteU8(dst, customServoMixer(i)->rate);
                sbufWriteU8(dst, customServoMixer(i)->speed);
                sbufWriteU8(dst, customServoMixer(i)->min);
                sbufWriteU8(dst, customServoMixer(i)->max);
                sbufWriteU8(dst, customServoMixer(i)->box);
            }
            break;
*/
#endif

        case MSP_MOTOR:
            for (unsigned i = 0; i < 8; i++) {
				// TODO: fix this
                //sbufWriteU16(dst, i < MAX_SUPPORTED_MOTORS ? mixer_get_motor_value(&ninja->mixer, i) : 0);
            }
            break;

        case MSP_RC:
            for (int i = 0; i < rx_get_channel_count(&self->ninja->rx); i++)
                sbufWriteU16(dst, rx_get_channel(&self->ninja->rx, i));
            break;

        case MSP_ATTITUDE:
            sbufWriteU16(dst, ins_get_roll_dd(&self->ninja->ins));
            sbufWriteU16(dst, ins_get_pitch_dd(&self->ninja->ins));
            sbufWriteU16(dst, DECIDEGREES_TO_DEGREES(ins_get_yaw_dd(&self->ninja->ins)));
            break;

        case MSP_ALTITUDE:
            sbufWriteU32(dst, ins_get_altitude_cm(&self->ninja->ins));
            sbufWriteU16(dst, ins_get_vertical_speed_cms(&self->ninja->ins)); // vario
            break;

        case MSP_SONAR_ALTITUDE:
			// TODO: msp sonar altitude
            //sbufWriteU32(dst, sonar_get_altitude(&default_sonar));
            sbufWriteU32(dst, 0);
            break;

        case MSP_ANALOG:
            sbufWriteU8(dst, (uint8_t)constrain(battery_get_voltage(&self->ninja->bat), 0, 255));
            sbufWriteU16(dst, (uint16_t)constrain(battery_get_spent_capacity(&self->ninja->bat), 0, 0xFFFF)); // milliamp hours drawn from battery
            sbufWriteU16(dst, rx_get_rssi(&self->ninja->rx));
            if(self->config->bat.multiwiiCurrentMeterOutput) {
                sbufWriteU16(dst, (uint16_t)constrain(battery_get_current(&self->ninja->bat) * 10, 0, 0xFFFF)); // send amperage in 0.001 A steps. Negative range is truncated to zero
            } else
                sbufWriteU16(dst, (int16_t)constrain(battery_get_current(&self->ninja->bat), -0x8000, 0x7FFF)); // send amperage in 0.01 A steps, range is -320A to 320A
            break;

        case MSP_ARMING_CONFIG:
            sbufWriteU8(dst, self->config->arm.auto_disarm_delay);
            sbufWriteU8(dst, self->config->arm.disarm_kill_switch);
            break;

        case MSP_LOOP_TIME:
            sbufWriteU16(dst, self->config->imu.looptime);
            break;

        case MSP_RC_TUNING: {
			const struct rate_profile *rate = config_get_rate_profile(self->config);
            sbufWriteU8(dst, rate->rcRate8);
            sbufWriteU8(dst, rate->rcExpo8);
            for (unsigned i = 0 ; i < 3; i++) {
                sbufWriteU8(dst, rate->rates[i]); // R,P,Y see flight_dynamics_index_t
            }
            sbufWriteU8(dst, rate->dynThrPID);
            sbufWriteU8(dst, rate->thrMid8);
            sbufWriteU8(dst, rate->thrExpo8);
            sbufWriteU16(dst, rate->tpa_breakpoint);
            sbufWriteU8(dst, rate->rcYawExpo8);
        } break;

        case MSP_PID: {
			const struct pid_config *pid = &config_get_profile(self->config)->pid;
            for (int i = 0; i < PID_ITEM_COUNT; i++) {
                sbufWriteU8(dst, pid->P8[i]);
                sbufWriteU8(dst, pid->I8[i]);
                sbufWriteU8(dst, pid->D8[i]);
            }
        } break;

        case MSP_PIDNAMES:
            sbufWriteString(dst, pidnames);
            break;

        case MSP_PID_CONTROLLER:
            sbufWriteU8(dst, config_get_profile(self->config)->pid.pidController);
            break;

        case MSP_MODE_RANGES:
            for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
                const struct rc_func_range *mac = &config_get_profile(self->config)->rc_funcs.ranges[i];
                const box_t *box = findBoxByBoxId(mac->modeId);
                sbufWriteU8(dst, box->permanentId);
                sbufWriteU8(dst, mac->auxChannelIndex);
                sbufWriteU8(dst, mac->range.startStep);
                sbufWriteU8(dst, mac->range.endStep);
            }
            break;

        case MSP_ADJUSTMENT_RANGES:
            for (int i = 0; i < MAX_ADJUSTMENT_RANGE_COUNT; i++) {
                const adjustmentRange_t *adjRange = &config_get_profile(self->config)->rc_adj.adjustmentRanges[i];
                sbufWriteU8(dst, adjRange->adjustmentIndex);
                sbufWriteU8(dst, adjRange->auxChannelIndex);
                sbufWriteU8(dst, adjRange->range.startStep);
                sbufWriteU8(dst, adjRange->range.endStep);
                sbufWriteU8(dst, adjRange->adjustmentFunction);
                sbufWriteU8(dst, adjRange->auxSwitchChannelIndex);
            }
            break;

        case MSP_BOXNAMES:
            serializeBoxNamesReply(self, reply);
            break;

        case MSP_BOXIDS:
            serializeBoxIdsReply(self, reply);
            break;

        case MSP_MISC:
            sbufWriteU16(dst, self->config->rx.midrc);

            sbufWriteU16(dst, self->config->pwm_out.minthrottle);
            sbufWriteU16(dst, self->config->pwm_out.maxthrottle);
            sbufWriteU16(dst, self->config->pwm_out.mincommand);

            sbufWriteU16(dst, self->config->failsafe.failsafe_throttle);

#ifdef GPS
            sbufWriteU8(dst, self->config->gps.provider); // gps_type
            sbufWriteU8(dst, 0); // TODO gps_baudrate (an index, ninjaflight uses a uint32_t
            sbufWriteU8(dst, self->config->gps.sbasMode); // gps_ubx_sbas
#else
            sbufWriteU8(dst, 0); // gps_type
            sbufWriteU8(dst, 0); // TODO gps_baudrate (an index, ninjaflight uses a uint32_t
            sbufWriteU8(dst, 0); // gps_ubx_sbas
#endif
            sbufWriteU8(dst, self->config->bat.multiwiiCurrentMeterOutput);
            sbufWriteU8(dst, self->config->rx.rssi_channel);
            sbufWriteU8(dst, 0);

            sbufWriteU16(dst, config_get_profile(self->config)->mag.mag_declination / 10);

            sbufWriteU8(dst, self->config->bat.vbatscale);
            sbufWriteU8(dst, self->config->bat.vbatmincellvoltage);
            sbufWriteU8(dst, self->config->bat.vbatmaxcellvoltage);
            sbufWriteU8(dst, self->config->bat.vbatwarningcellvoltage);
            break;

        case MSP_MOTOR_PINS:
            // FIXME This is hardcoded and should not be.
            for (int i = 0; i < 8; i++)
                sbufWriteU8(dst, i + 1);
            break;

#ifdef GPS
        case MSP_RAW_GPS:
            //sbufWriteU8(dst, STATE(GPS_FIX));
            sbufWriteU8(dst, false);
            sbufWriteU8(dst, self->ninja->gps.GPS_numSat);
            sbufWriteU32(dst, self->ninja->gps.GPS_coord[LAT]);
            sbufWriteU32(dst, self->ninja->gps.GPS_coord[LON]);
            sbufWriteU16(dst, self->ninja->gps.GPS_altitude);
            sbufWriteU16(dst, self->ninja->gps.GPS_speed);
            sbufWriteU16(dst, self->ninja->gps.GPS_ground_course);
            break;

        case MSP_COMP_GPS:
			// TODO: msp comp gps
            //sbufWriteU16(dst, ninja->gps.GPS_distanceToHome);
            //sbufWriteU16(dst, ninja->gps.GPS_directionToHome);
            sbufWriteU8(dst, self->ninja->gps.GPS_update & 1);
            break;

        case MSP_WP: {
            uint8_t wp_no = sbufReadU8(src);    // get the wp number
            int32_t lat = 0, lon = 0;
            if (wp_no == 0) {
                lat = GPS_home[LAT];
                lon = GPS_home[LON];
            } else if (wp_no == 16) {
                lat = GPS_hold[LAT];
                lon = GPS_hold[LON];
            }
            sbufWriteU8(dst, wp_no);
            sbufWriteU32(dst, lat);
            sbufWriteU32(dst, lon);
            sbufWriteU32(dst, ins_get_altitude_cm(&self->ninja->ins));           // altitude (cm) will come here -- temporary implementation to test feature with apps
            sbufWriteU16(dst, 0);                 // heading  will come here (deg)
            sbufWriteU16(dst, 0);                 // time to stay (ms) will come here
            sbufWriteU8(dst, 0);                  // nav flag will come here
            break;
        }

        case MSP_GPSSVINFO:
            sbufWriteU8(dst, self->ninja->gps.GPS_numCh);
            for (int i = 0; i < self->ninja->gps.GPS_numCh; i++){
                sbufWriteU8(dst, self->ninja->gps.GPS_svinfo_chn[i]);
                sbufWriteU8(dst, self->ninja->gps.GPS_svinfo_svid[i]);
                sbufWriteU8(dst, self->ninja->gps.GPS_svinfo_quality[i]);
                sbufWriteU8(dst, self->ninja->gps.GPS_svinfo_cno[i]);
            }
            break;
#endif

        case MSP_DEBUG: 
            // output some useful QA statistics
            // debug[x] = ((hse_value / 1000000) * 1000) + (SystemCoreClock / 1000000);         // XX0YY [crystal clock : core clock]

            for (int i = 0; i < DEBUG16_VALUE_COUNT; i++)
                sbufWriteU16(dst, debug[i]);      // 4 variables are here for general monitoring purpose
            break;

            // Additional commands that are not compatible with MultiWii
        case MSP_ACC_TRIM:
            sbufWriteU16(dst, config_get_profile(self->config)->acc.trims.values.pitch);
            sbufWriteU16(dst, config_get_profile(self->config)->acc.trims.values.roll);
            break;

        case MSP_UID:
            sbufWriteU32(dst, U_ID_0);
            sbufWriteU32(dst, U_ID_1);
            sbufWriteU32(dst, U_ID_2);
            break;

        case MSP_FEATURE:
            sbufWriteU32(dst, featureMask(self->config));
            break;

        case MSP_VOLTAGE_METER_CONFIG:
            sbufWriteU8(dst, self->config->bat.vbatscale);
            sbufWriteU8(dst, self->config->bat.vbatmincellvoltage);
            sbufWriteU8(dst, self->config->bat.vbatmaxcellvoltage);
            sbufWriteU8(dst, self->config->bat.vbatwarningcellvoltage);
            break;

        case MSP_CURRENT_METER_CONFIG:
            sbufWriteU16(dst, self->config->bat.currentMeterScale);
            sbufWriteU16(dst, self->config->bat.currentMeterOffset);
            sbufWriteU8(dst, self->config->bat.currentMeterType);
            sbufWriteU16(dst, self->config->bat.batteryCapacity);
            break;

        case MSP_MIXER:
            sbufWriteU8(dst, self->config->mixer.mixerMode);
            break;

        case MSP_RX_CONFIG:
            sbufWriteU8(dst, self->config->rx.serialrx_provider);
            sbufWriteU16(dst, self->config->rx.maxcheck);
            sbufWriteU16(dst, self->config->rx.midrc);
            sbufWriteU16(dst, self->config->rx.mincheck);
            sbufWriteU8(dst, self->config->rx.spektrum_sat_bind);
            sbufWriteU16(dst, self->config->rx.rx_min_usec);
            sbufWriteU16(dst, self->config->rx.rx_max_usec);
            break;

        case MSP_RXFAIL_CONFIG:
            for (int i = 0; i < rx_get_channel_count(&self->ninja->rx); i++) {
                sbufWriteU8(dst, self->config->rx_output.failsafe[i].mode);
                sbufWriteU16(dst, RXFAIL_STEP_TO_CHANNEL_VALUE(self->config->rx_output.failsafe[i].step));
            }
            break;

        case MSP_RSSI_CONFIG:
            sbufWriteU8(dst, self->config->rx.rssi_channel);
            break;

        case MSP_RX_MAP:
            for (int i = 0; i < RX_MAX_MAPPABLE_RX_INPUTS; i++)
                sbufWriteU8(dst, self->config->rx.rcmap[i]);
            break;

        case MSP_BF_CONFIG:
            sbufWriteU8(dst, self->config->mixer.mixerMode);

            sbufWriteU32(dst, featureMask(self->config));

            sbufWriteU8(dst, self->config->rx.serialrx_provider);

            sbufWriteU16(dst, self->config->alignment.rollDegrees);
            sbufWriteU16(dst, self->config->alignment.pitchDegrees);
            sbufWriteU16(dst, self->config->alignment.yawDegrees);

            sbufWriteU16(dst, self->config->bat.currentMeterScale);
            sbufWriteU16(dst, self->config->bat.currentMeterOffset);
            break;

        case MSP_CF_SERIAL_CONFIG:
            for (int i = 0; i < SERIAL_PORT_COUNT; i++) {
                if (!serialIsPortAvailable(self->config->serial.portConfigs[i].identifier)) {
                    continue;
                };
                sbufWriteU8(dst, self->config->serial.portConfigs[i].identifier);
                sbufWriteU16(dst, self->config->serial.portConfigs[i].functionMask);
                sbufWriteU8(dst, self->config->serial.portConfigs[i].msp_baudrateIndex);
                sbufWriteU8(dst, self->config->serial.portConfigs[i].gps_baudrateIndex);
                sbufWriteU8(dst, self->config->serial.portConfigs[i].telemetry_baudrateIndex);
                sbufWriteU8(dst, self->config->serial.portConfigs[i].blackbox_baudrateIndex);
            }
            break;

#ifdef LED_STRIP
        case MSP_LED_COLORS:
            for (int i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++) {
                hsvColor_t *color = &self->config->ledstrip.colors[i];
                sbufWriteU16(dst, color->h);
                sbufWriteU8(dst, color->s);
                sbufWriteU8(dst, color->v);
            }
            break;

        case MSP_LED_STRIP_CONFIG:
            for (int i = 0; i < LED_MAX_STRIP_LENGTH; i++) {
                struct led_config *ledConfig = &self->config->ledstrip.leds[i];
                sbufWriteU16(dst, (ledConfig->flags & LED_FLAG_DIRECTION_MASK) >> LED_DIRECTION_BIT_OFFSET);
                sbufWriteU16(dst, (ledConfig->flags & LED_FLAG_FUNCTION_MASK) >> LED_FUNCTION_BIT_OFFSET);
                sbufWriteU8(dst, ledGetX(ledConfig));
                sbufWriteU8(dst, ledGetY(ledConfig));
                sbufWriteU8(dst, ledConfig->color);
            }
            break;

        case MSP_LED_STRIP_MODECOLOR:
            for (int i = 0; i < LED_MODE_COUNT; i++) {
                for (int j = 0; j < LED_DIRECTION_COUNT; j++) {
                    sbufWriteU8(dst, i);
                    sbufWriteU8(dst, j);
                    sbufWriteU8(dst, self->config->ledstrip.modeColors[i].color[j]);
                }
            }
            for (int j = 0; j < LED_SPECIAL_COLOR_COUNT; j++) {
                sbufWriteU8(dst, LED_MODE_COUNT);
                sbufWriteU8(dst, j);
                sbufWriteU8(dst, self->config->ledstrip.spcColors[0].color[j]);
            }
            break;
#endif

        case MSP_DATAFLASH_SUMMARY:
            serializeDataflashSummaryReply(reply);
            break;

#ifdef USE_FLASHFS
        case MSP_DATAFLASH_READ: {
            uint32_t readAddress = sbufReadU32(src);

            serializeDataflashReadReply(reply, readAddress, 128);
            break;
        }
#endif

        case MSP_BLACKBOX_CONFIG:
			if(USE_BLACKBOX){
				sbufWriteU8(dst, 1); //Blackbox supported
				sbufWriteU8(dst, self->config->blackbox.device);
				sbufWriteU8(dst, self->config->blackbox.rate_num);
				sbufWriteU8(dst, self->config->blackbox.rate_denom);
			} else {
				sbufWriteU8(dst, 0); // Blackbox not supported
				sbufWriteU8(dst, 0);
				sbufWriteU8(dst, 0);
				sbufWriteU8(dst, 0);
			}
            break;

        case MSP_SDCARD_SUMMARY:
            serializeSDCardSummaryReply(reply);
            break;

        case MSP_TRANSPONDER_CONFIG:
#ifdef TRANSPONDER
            sbufWriteU8(dst, 1); //Transponder supported
            sbufWriteData(dst, transponderConfig()->data, sizeof(transponderConfig()->data));
#else
            sbufWriteU8(dst, 0); // Transponder not supported
#endif
            break;

        case MSP_BF_BUILD_INFO:
            sbufWriteData(dst, buildDate, 11); // MMM DD YYYY as ascii, MMM = Jan/Feb... etc
            sbufWriteU32(dst, 0); // future exp
            sbufWriteU32(dst, 0); // future exp
            break;

        case MSP_3D:
            sbufWriteU16(dst, self->config->motor_3d.deadband3d_low);
            sbufWriteU16(dst, self->config->motor_3d.deadband3d_high);
            sbufWriteU16(dst, self->config->motor_3d.neutral3d);
            break;

        case MSP_RC_DEADBAND:
            sbufWriteU8(dst, config_get_profile(self->config)->rc.deadband);
            sbufWriteU8(dst, config_get_profile(self->config)->rc.yaw_deadband);
            sbufWriteU8(dst, config_get_profile(self->config)->rc.alt_hold_deadband);
            sbufWriteU16(dst, config_get_profile(self->config)->rc.deadband3d_throttle);
            break;

        case MSP_SENSOR_ALIGNMENT:
            sbufWriteU8(dst, self->config->sensors.alignment.gyro_align);
            sbufWriteU8(dst, self->config->sensors.alignment.acc_align);
            sbufWriteU8(dst, self->config->sensors.alignment.mag_align);
            break;

#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
        case MSP_SET_4WAY_IF:
            // initialize 4way ESC interface, return number of ESCs available
            sbufWriteU8(dst, esc4wayInit());
            self->mspEnterEsc4way = true;     // request protocol switch
            break;
#endif

        default:
            return 0;   // unknown command
    }
    return 1;           // command processed
}

// return positive for ACK, negative on error, zero for no reply
static int processInCommand(struct msp *self, mspPacket_t *cmd){
    sbuf_t * src = &cmd->buf;
    int len = sbufBytesRemaining(src);

    switch (cmd->cmd) {
        case MSP_SELECT_SETTING:
            if (!ninja_is_armed(self->ninja)) {
                //int profile = sbufReadU8(src);
                //ninja_config_change_profile(self->ninja, profile);
            }
            break;

        case MSP_SET_HEAD:
            self->ninja->magHold = sbufReadU16(src);
            break;

        case MSP_SET_RAW_RC: {
            uint8_t channelCount = len / sizeof(uint16_t);
            if (channelCount > RX_MAX_SUPPORTED_RC_CHANNELS)
                return -1;
            uint16_t frame[RX_MAX_SUPPORTED_RC_CHANNELS];

            for (unsigned i = 0; i < channelCount; i++) {
                frame[i] = sbufReadU16(src);
            }

            rxMspFrameReceive(frame, channelCount);
            break;
        }

        case MSP_SET_ACC_TRIM:
            config_get_profile_rw(self->config)->acc.trims.values.pitch = sbufReadU16(src);
            config_get_profile_rw(self->config)->acc.trims.values.roll  = sbufReadU16(src);
            break;

        case MSP_SET_ARMING_CONFIG:
            self->config->arm.auto_disarm_delay = sbufReadU8(src);
            self->config->arm.disarm_kill_switch = sbufReadU8(src);
            break;

        case MSP_SET_LOOP_TIME:
            self->config->imu.looptime = sbufReadU16(src);
            break;

        case MSP_SET_PID_CONTROLLER:
            config_get_profile_rw(self->config)->pid.pidController = sbufReadU8(src);
            break;

        case MSP_SET_PID: {
			struct pid_config *pid = &config_get_profile_rw(self->config)->pid;
            for (int i = 0; i < PID_ITEM_COUNT; i++) {
                    pid->P8[i] = sbufReadU8(src);
                    pid->I8[i] = sbufReadU8(src);
                    pid->D8[i] = sbufReadU8(src);
                }
        } break;

        case MSP_SET_MODE_RANGE: {
            int i = sbufReadU8(src);
            if (i >= MAX_MODE_ACTIVATION_CONDITION_COUNT)
                return -1;
            struct rc_func_range *mac = &config_get_profile_rw(self->config)->rc_funcs.ranges[i];
            int permId = sbufReadU8(src);
            const box_t *box = findBoxByPermenantId(permId);
            if (box == NULL)
                return -1;
            mac->modeId = box->boxId;
            mac->auxChannelIndex = sbufReadU8(src);
            mac->range.startStep = sbufReadU8(src);
            mac->range.endStep = sbufReadU8(src);

            break;
        }

        case MSP_SET_ADJUSTMENT_RANGE: {
            int aRange = sbufReadU8(src);
            if (aRange >= MAX_ADJUSTMENT_RANGE_COUNT)
                return -1;
            adjustmentRange_t *adjRange = &config_get_profile_rw(self->config)->rc_adj.adjustmentRanges[aRange];
            int aIndex = sbufReadU8(src);
            if (aIndex > MAX_SIMULTANEOUS_ADJUSTMENT_COUNT)
                return -1;
            adjRange->adjustmentIndex = aIndex;
            adjRange->auxChannelIndex = sbufReadU8(src);
            adjRange->range.startStep = sbufReadU8(src);
            adjRange->range.endStep = sbufReadU8(src);
            adjRange->adjustmentFunction = sbufReadU8(src);
            adjRange->auxSwitchChannelIndex = sbufReadU8(src);
            break;
        }

        case MSP_SET_RC_TUNING: {
            if (len < 10)
                return -1;
			struct rate_profile *currentControlRateProfile = config_get_rate_profile_rw(self->config);
            currentControlRateProfile->rcRate8 = sbufReadU8(src);
            currentControlRateProfile->rcExpo8 = sbufReadU8(src);
            for (int i = 0; i < 3; i++) {
                unsigned rate = sbufReadU8(src);
                currentControlRateProfile->rates[i] = MIN(rate, i == YAW ? CONTROL_RATE_CONFIG_YAW_RATE_MAX : CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX);
            }
            unsigned rate = sbufReadU8(src);
            currentControlRateProfile->dynThrPID = MIN(rate, CONTROL_RATE_CONFIG_TPA_MAX);
            currentControlRateProfile->thrMid8 = sbufReadU8(src);
            currentControlRateProfile->thrExpo8 = sbufReadU8(src);
            currentControlRateProfile->tpa_breakpoint = sbufReadU16(src);
            if (len < 11)
                break;
            currentControlRateProfile->rcYawExpo8 = sbufReadU8(src);
        } break;

        case MSP_SET_MISC: {
            unsigned midrc = sbufReadU16(src);
            if (midrc > 1400 && midrc < 1600)
                self->config->rx.midrc = midrc;

            self->config->pwm_out.minthrottle = sbufReadU16(src);
            self->config->pwm_out.maxthrottle = sbufReadU16(src);
            self->config->pwm_out.mincommand = sbufReadU16(src);

            self->config->failsafe.failsafe_throttle = sbufReadU16(src);

#ifdef GPS
            self->config->gps.provider = sbufReadU8(src); // gps_type
            sbufReadU8(src); // gps_baudrate
            self->config->gps.sbasMode = sbufReadU8(src); // gps_ubx_sbas
#else
            sbufReadU8(src); // gps_type
            sbufReadU8(src); // gps_baudrate
            sbufReadU8(src); // gps_ubx_sbas
#endif
            self->config->bat.multiwiiCurrentMeterOutput = sbufReadU8(src);
            self->config->rx.rssi_channel = sbufReadU8(src);
            sbufReadU8(src);

            config_get_profile_rw(self->config)->mag.mag_declination = sbufReadU16(src) * 10;

            self->config->bat.vbatscale = sbufReadU8(src);           // actual vbatscale as intended
            self->config->bat.vbatmincellvoltage = sbufReadU8(src);  // vbatlevel_warn1 in MWC2.3 GUI
            self->config->bat.vbatmaxcellvoltage = sbufReadU8(src);  // vbatlevel_warn2 in MWC2.3 GUI
            self->config->bat.vbatwarningcellvoltage = sbufReadU8(src);  // vbatlevel when buzzer starts to alert
            break;
        }

        case MSP_SET_MOTOR:
            for (int i = 0; i < MIXER_MAX_MOTORS; i++) {
                const int16_t value = sbufReadU16(src);
				mixer_input_command(&self->ninja->mixer, MIXER_INPUT_GROUP_MOTOR_PASSTHROUGH + i, value - 1500);
            }
            break;

        case MSP_SET_SERVO_CONFIGURATION: {
#ifdef USE_SERVOS
            if (len != 1 + sizeof(struct servo_config))
                return -1;
            unsigned i = sbufReadU8(src);
            if (i >= MAX_SUPPORTED_SERVOS)
                return -1;
			struct servo_config *servo = &config_get_profile_rw(self->config)->servos.servoConf[i];
            servo->min = sbufReadU16(src);
            servo->max = sbufReadU16(src);
            servo->middle = sbufReadU16(src);
            servo->rate = sbufReadU8(src);
            servo->angleAtMin = sbufReadU8(src);
            servo->angleAtMax = sbufReadU8(src);
            servo->forwardFromChannel = sbufReadU8(src);
            servo->reversedSources = sbufReadU32(src);
#endif
            break;
        }
/*
        case MSP_SET_SERVO_MIX_RULE: {
#ifdef USE_SERVOS
            int i = sbufReadU8(src);
            if (i >= MAX_SERVO_RULES)
                return -1;

            customServoMixer(i)->targetChannel = sbufReadU8(src);
            customServoMixer(i)->inputSource = sbufReadU8(src);
            customServoMixer(i)->rate = sbufReadU8(src);
            customServoMixer(i)->speed = sbufReadU8(src);
            customServoMixer(i)->min = sbufReadU8(src);
            customServoMixer(i)->max = sbufReadU8(src);
            customServoMixer(i)->box = sbufReadU8(src);
			//TODO: make this use standard mixer rules
            //loadCustomServoMixer();
#endif
            break;
        }
*/
        case MSP_SET_3D:
            self->config->motor_3d.deadband3d_low = sbufReadU16(src);
            self->config->motor_3d.deadband3d_high = sbufReadU16(src);
            self->config->motor_3d.neutral3d = sbufReadU16(src);
            break;

        case MSP_SET_RC_DEADBAND:
            config_get_profile_rw(self->config)->rc.deadband = sbufReadU8(src);
            config_get_profile_rw(self->config)->rc.yaw_deadband = sbufReadU8(src);
            config_get_profile_rw(self->config)->rc.alt_hold_deadband = sbufReadU8(src);
            config_get_profile_rw(self->config)->rc.deadband3d_throttle = sbufReadU16(src);
            break;
/*
		// TODO: msp reset pid 
        case MSP_SET_RESET_CURR_PID:
            PG_RESET_CURRENT(pidProfile);
            break;
*/
        case MSP_SET_SENSOR_ALIGNMENT:
            self->config->sensors.alignment.gyro_align = sbufReadU8(src);
            self->config->sensors.alignment.acc_align = sbufReadU8(src);
            self->config->sensors.alignment.mag_align = sbufReadU8(src);
            break;

        case MSP_RESET_CONF:
			if(!ninja_is_armed(self->ninja)){
				ninja_config_reset(self->ninja);
				ninja_config_save(self->ninja);
			}
            break;

        case MSP_ACC_CALIBRATION:
            if (!ninja_is_armed(self->ninja))
				ninja_calibrate_acc(self->ninja);
            break;

        case MSP_MAG_CALIBRATION:
            if (!ninja_is_armed(self->ninja))
				ninja_calibrate_mag(self->ninja);
            break;

        case MSP_EEPROM_WRITE:
			ninja_config_save(self->ninja);
            break;

        case MSP_SET_BLACKBOX_CONFIG:
			if(USE_BLACKBOX){
				if (!blackbox_is_running(&self->ninja->blackbox))
					return -1;
				self->config->blackbox.device = sbufReadU8(src);
				self->config->blackbox.rate_num = sbufReadU8(src);
				self->config->blackbox.rate_denom = sbufReadU8(src);
			} else {
				return -1;
			}
            break;

#ifdef TRANSPONDER
        case MSP_SET_TRANSPONDER_CONFIG:
            if (len != sizeof(self->config->transponder.data))
                return -1;
            sbufReadData(src, self->config->transponder.data, sizeof(self->config->transponder.data));
            transponderUpdateData(self->config->transponder.data);
            break;
#endif

#ifdef USE_FLASHFS
        case MSP_DATAFLASH_ERASE:
            flashfsEraseCompletely();
            break;
#endif

#ifdef GPS
        case MSP_SET_RAW_GPS:
			// TODO: msp gps fix
            if (sbufReadU8(src)) {
                //ENABLE_STATE(GPS_FIX);
            } else {
                //DISABLE_STATE(GPS_FIX);
            }
            self->ninja->gps.GPS_numSat = sbufReadU8(src);
            self->ninja->gps.GPS_coord[LAT] = sbufReadU32(src);
            self->ninja->gps.GPS_coord[LON] = sbufReadU32(src);
            self->ninja->gps.GPS_altitude = sbufReadU16(src);
            self->ninja->gps.GPS_speed = sbufReadU16(src);
            self->ninja->gps.GPS_update |= 2;        // New data signalisation to GPS functions // FIXME Magic Numbers
            break;

        case MSP_SET_WP: {
            uint8_t wp_no = sbufReadU8(src);             // get the wp number
            int32_t lat = sbufReadU32(src);
            int32_t lon = sbufReadU32(src);
            int32_t __attribute__((unused)) alt = sbufReadU32(src);              // to set altitude (cm)
            sbufReadU16(src);                            // future: to set heading (deg)
            sbufReadU16(src);                            // future: to set time to stay (ms)
            sbufReadU8(src);                             // future: to set nav flag
            if (wp_no == 0) {
                GPS_home[LAT] = lat;
                GPS_home[LON] = lon;
				// TODO: gps
                //DISABLE_FLIGHT_MODE(GPS_HOME_MODE);     // with this flag, GPS_set_next_wp will be called in the next loop -- OK with SERIAL GPS / OK with I2C GPS
                //ENABLE_STATE(GPS_FIX_HOME);
                //if (alt != 0)
                 //   AltHold = alt;                      // temporary implementation to test feature with apps
            } else if (wp_no == 16) {                   // OK with SERIAL GPS  --  NOK for I2C GPS / needs more code dev in order to inject GPS coord inside I2C GPS
                GPS_hold[LAT] = lat;
                GPS_hold[LON] = lon;
                //if (alt != 0)
                 //   AltHold = alt;                      // temporary implementation to test feature with apps
                navi_mode = NAV_MODE_WP;
                GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
            }
            break;
        }
#endif

        case MSP_SET_FEATURE:
            featureClearAll(self->config);
            featureSet(self->config, sbufReadU32(src)); // features bitmap
            break;

        case MSP_SET_VOLTAGE_METER_CONFIG:
            self->config->bat.vbatscale = sbufReadU8(src);               // actual vbatscale as intended
            self->config->bat.vbatmincellvoltage = sbufReadU8(src);      // vbatlevel_warn1 in MWC2.3 GUI
            self->config->bat.vbatmaxcellvoltage = sbufReadU8(src);      // vbatlevel_warn2 in MWC2.3 GUI
            self->config->bat.vbatwarningcellvoltage = sbufReadU8(src);  // vbatlevel when buzzer starts to alert
            break;

        case MSP_SET_CURRENT_METER_CONFIG:
            self->config->bat.currentMeterScale = sbufReadU16(src);
            self->config->bat.currentMeterOffset = sbufReadU16(src);
            self->config->bat.currentMeterType = sbufReadU8(src);
            self->config->bat.batteryCapacity = sbufReadU16(src);
            break;

        case MSP_SET_MIXER:
			self->config->mixer.mixerMode = sbufReadU8(src);
            break;

        case MSP_SET_RX_CONFIG:
            self->config->rx.serialrx_provider = sbufReadU8(src);
            self->config->rx.maxcheck = sbufReadU16(src);
            self->config->rx.midrc = sbufReadU16(src);
            self->config->rx.mincheck = sbufReadU16(src);
            self->config->rx.spektrum_sat_bind = sbufReadU8(src);
            if (sbufBytesRemaining(src) < 2)
                break;
            self->config->rx.rx_min_usec = sbufReadU16(src);
            self->config->rx.rx_max_usec = sbufReadU16(src);
            break;

        case MSP_SET_RXFAIL_CONFIG: {
            int channel =  sbufReadU8(src);
            if (channel >= RX_MAX_SUPPORTED_RC_CHANNELS)
                return -1;
            self->config->rx_output.failsafe[channel].mode = sbufReadU8(src);
            self->config->rx_output.failsafe[channel].step = CHANNEL_VALUE_TO_RXFAIL_STEP(sbufReadU16(src));
            break;
        }

        case MSP_SET_RSSI_CONFIG:
            self->config->rx.rssi_channel = sbufReadU8(src);
            break;

        case MSP_SET_RX_MAP:
            for (int i = 0; i < RX_MAX_MAPPABLE_RX_INPUTS; i++) {
                self->config->rx.rcmap[i] = sbufReadU8(src);
            }
            break;

        case MSP_SET_BF_CONFIG:
            self->config->mixer.mixerMode = sbufReadU8(src);        // mixerMode

            featureClearAll(self->config);
            featureSet(self->config, sbufReadU32(src));                      // features bitmap

            self->config->rx.serialrx_provider = sbufReadU8(src);   // serialrx_type

            self->config->alignment.rollDegrees = sbufReadU16(src);  // board_align_roll
            self->config->alignment.pitchDegrees = sbufReadU16(src); // board_align_pitch
            self->config->alignment.yawDegrees = sbufReadU16(src);   // board_align_yaw

            self->config->bat.currentMeterScale = sbufReadU16(src);
            self->config->bat.currentMeterOffset = sbufReadU16(src);
            break;

        case MSP_SET_CF_SERIAL_CONFIG: {
            int portConfigSize = sizeof(uint8_t) + sizeof(uint16_t) + (sizeof(uint8_t) * 4);

            if (len % portConfigSize != 0)
                return -1;

            while (sbufBytesRemaining(src) >= portConfigSize) {
				// TODO: serial config
/*
                uint8_t identifier = sbufReadU8(src);
                struct serial_port_config *portConfig = serialFindPortConfiguration(&self->config->serial, identifier);
                if (!portConfig)
                    return -1;

                portConfig->identifier = identifier;
                portConfig->functionMask = sbufReadU16(src);
                portConfig->msp_baudrateIndex = sbufReadU8(src);
                portConfig->gps_baudrateIndex = sbufReadU8(src);
                portConfig->telemetry_baudrateIndex = sbufReadU8(src);
                portConfig->blackbox_baudrateIndex = sbufReadU8(src);
				*/
            }
            break;
        }

#ifdef LED_STRIP
        case MSP_SET_LED_COLORS:

            for (int i = 0; i < LED_CONFIGURABLE_COLOR_COUNT && sbufBytesRemaining(src) >= 4; i++) {
                hsvColor_t *color = &self->config->ledstrip.colors[i];

                int h = sbufReadU16(src);
                int s = sbufReadU8(src);
                int v = sbufReadU8(src);

                if (h > HSV_HUE_MAX || s > HSV_SATURATION_MAX || v > HSV_VALUE_MAX) {
                    memset(color, 0, sizeof(*color));
                    return -1;
                }

                color->h = h;
                color->s = s;
                color->v = v;
            }
            break;

        case MSP_SET_LED_STRIP_CONFIG: {
            int i = sbufReadU8(src);
            if (len != (1 + 7) || i >= LED_MAX_STRIP_LENGTH)
                return -1;

            struct led_config *ledConfig = &self->config->ledstrip.leds[i];
            uint16_t mask;
            uint16_t flags;
            // currently we're storing directions and functions in a uint16 (flags)
            // the msp uses 2 x uint16_t to cater for future expansion
            mask = sbufReadU16(src);
            flags = (mask << LED_DIRECTION_BIT_OFFSET) & LED_FLAG_DIRECTION_MASK;
            mask = sbufReadU16(src);
            flags |= (mask << LED_FUNCTION_BIT_OFFSET) & LED_FLAG_FUNCTION_MASK;
            ledConfig->flags = flags;

            int x = sbufReadU8(src);
            int y = sbufReadU8(src);
            ledSetXY(ledConfig, x, y);

            ledConfig->color = sbufReadU8(src);

            //ledstrip_reload_config(&ninja->ledstrip);
        }
        break;

        case MSP_SET_LED_STRIP_MODECOLOR:
		/*
            while (sbufBytesRemaining(src) >= 3) {
                ledModeIndex_e modeIdx = sbufReadU8(src);
                int funIdx = sbufReadU8(src);
                int color = sbufReadU8(src);

                if (!ledstrip_set_mode_color(&self->ninja->ledstrip, modeIdx, funIdx, color))
                    return -1;
            }*/
            break;
#endif

        case MSP_REBOOT:
            self->isRebootScheduled = true;
            break;

        default:
            // we do not know how to handle the (valid) message, try another message handler
            return 0;
    }
    return 1;     // message was handled succesfully
}

void msp_init(struct msp *self, struct ninja *nin, struct config *config){
	self->ninja = nin;
	self->config = config;
    initActiveBoxIds(self);
}


// handle received command, possibly generate reply.
// return nonzero when reply was generated (including reported error)
int msp_process(struct msp *self, mspPacket_t *command, mspPacket_t *reply)
{
    // initialize reply by default
    reply->cmd = command->cmd;
    int status;
    do {
        if((status = processInCommand(self, command)) != 0)
            break;
        if((status = processOutCommand(self, command, reply)) != 0)
            break;
        //if((status = processPgCommand(self, command, reply)) != 0)
         //   break;
        // command was not handled, return error
        status = -1;
    } while(0);
	if(status == -1){
		//printf("unhandled command: %02x\n", command->cmd);
	}
    reply->result = status;
    return status;
}
