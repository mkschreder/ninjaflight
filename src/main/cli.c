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

/**
 * @addtogroup ninja
 * @{
 */

/**
 * @defgroup cli Command line interface
 * @{
 *
 * The cli module implements a command line interface for setting various
 * ninjaflight config variables. It operates on a struct ninja object and on
 * corresponding struct config object. It also provides various commands for
 * diagnostics and control. It is similar to other configuration interfaces
 * such as msp but in contrast to msp does not use a binary protocol, instead
 * using a human readable stream of data.
 */

/**
 * @file cli.c
 * @author Cleanflight
 * @author Martin Schröder 2016
 */


#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <strings.h>
#include <math.h>
#include <ctype.h>

#include <platform.h>
#include "version.h"

#include "build_config.h"

#include "common/utils.h"
#include "common/axis.h"
#include "common/maths.h"
#include "common/color.h"
#include "common/typeconversion.h"

#include "config/tilt.h"
#include "config/gimbal.h"

#include "drivers/system.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"

#include "drivers/serial.h"
#include "drivers/bus_i2c.h"
#include "drivers/pwm_rx.h"
#include "drivers/sdcard.h"
#include "drivers/pwm_output.h"
#include "common/buf_writer.h"

#include "flight/rate_profile.h"
#include "io/rc_adjustments.h"
#include "io/serial.h"
#include "io/ledstrip.h"
#include "io/beeper.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/barometer.h"

#include "blackbox/blackbox.h"

#include "flight/anglerate.h"
#include "flight/gtune.h"
#include "flight/mixer.h"
#include "flight/navigation.h"
#include "flight/failsafe.h"
#include "flight/altitudehold.h"
#include "flight/tilt.h"

//#include "telemetry/telemetry.h"
//#include "telemetry/frsky.h"
//#include "telemetry/hott.h"

#include "ninja.h"

#include "config/config.h"
#include "config/feature.h"
#include "config/profile.h"

#include "common/printf.h"

#include "cli.h"

static void cliAux(struct cli *self, char *cmdline);
static void cliRxFail(struct cli *self, char *cmdline);
static void cliAdjustmentRange(struct cli *self, char *cmdline);
static void cliMotorMix(struct cli *self, char *cmdline);
static void cliDefaults(struct cli *self, char *cmdline);
static void cliDump(struct cli *self, char *cmdLine);
static void cliExit(struct cli *self, char *cmdline);
static void cliFeature(struct cli *self, char *cmdline);
static void cliMotor(struct cli *self, char *cmdline);
static void cliPlaySound(struct cli *self, char *cmdline);
static void cliProfile(struct cli *self, char *cmdline);
static void cliRateProfile(struct cli *self, char *cmdline);
static void cliReboot(struct cli *self);
static void cliSave(struct cli *self, char *cmdline);
static void cliSerial(struct cli *self, char *cmdline);

#ifdef USE_SERVOS
static void cliServo(struct cli *self, char *cmdline);
static void cliServoMix(struct cli *self, char *cmdline);
#endif

static void cliSet(struct cli *self, char *cmdline);
static void cliGet(struct cli *self, char *cmdline);
static void cliStatus(struct cli *self, char *cmdline);
#ifndef SKIP_TASK_STATISTICS
static void cliTasks(struct cli *self, char *cmdline);
#endif
static void cliVersion(struct cli *self, char *cmdline);
static void cliRxRange(struct cli *self, char *cmdline);

#ifdef GPS
static void cliGpsPassthrough(struct cli *self, char *cmdline);
#endif

static void cliHelp(struct cli *self, char *cmdline);
static void cliMap(struct cli *self, char *cmdline);

#ifdef LED_STRIP
static void cliLed(struct cli *self, char *cmdline);
static void cliColor(struct cli *self, char *cmdline);
static void cliModeColor(struct cli *self, char *cmdline);
#endif

static void cliMixer(struct cli *self, char *cmdline);

#ifdef USE_FLASHFS
static void cliFlashInfo(struct cli *self, char *cmdline);
static void cliFlashErase(struct cli *self, char *cmdline);
#ifdef USE_FLASH_TOOLS
static void cliFlashWrite(struct cli *self, char *cmdline);
static void cliFlashRead(struct cli *self, char *cmdline);
#endif
#endif

#ifdef USE_SDCARD
static void cliSdInfo(struct cli *self, char *cmdline);
#endif

static void cliTilt(struct cli *self, char *cmdline); 

//  this with mixerMode_e
static const char * const mixerNames[] = {
    "TRI", "QUADP", "QUADX", "BI",
    "GIMBAL", "Y6", "HEX6",
    "FLYING_WING", "Y4", "HEX6X", "OCTOX8", "OCTOFLATP", "OCTOFLATX",
    "AIRPLANE", "HELI_120_CCPM", "HELI_90_DEG", "VTAIL4",
    "HEX6H", "PPM_TO_SERVO", "DUALCOPTER", "SINGLECOPTER",
    "ATAIL4", "TILT1", "TILT2", "CUSTOM", "CUSTOMAIRPLANE", "CUSTOMTRI", NULL
};

// sync this with features_e
static const char * const featureNames[] = {
    "RX_PPM", "VBAT", "INFLIGHT_ACC_CAL", "RX_SERIAL", "MOTOR_STOP",
    "SERVO_TILT", "SOFTSERIAL", "GPS", "FAILSAFE",
    "SONAR", "TELEMETRY", "CURRENT_METER", "3D", "RX_PARALLEL_PWM",
    "RX_MSP", "RSSI_ADC", "LED_STRIP", "DISPLAY", "ONESHOT125",
    "BLACKBOX", "CHANNEL_FORWARDING", "TRANSPONDER", NULL
};

// sync this with rxFailsafeChannelMode_e
static const char rxFailsafeModeCharacters[] = "ahs";

static const rxFailsafeChannelMode_e rxFailsafeModesTable[RX_FAILSAFE_TYPE_COUNT][RX_FAILSAFE_MODE_COUNT] = {
    { RX_FAILSAFE_MODE_AUTO, RX_FAILSAFE_MODE_HOLD, RX_FAILSAFE_MODE_INVALID },
    { RX_FAILSAFE_MODE_INVALID, RX_FAILSAFE_MODE_HOLD, RX_FAILSAFE_MODE_SET }
};

#ifndef CJMCU
// sync this with sensors_e
static const char * const sensorTypeNames[] = {
    "GYRO", "ACC", "BARO", "MAG", "SONAR", "GPS", "GPS+MAG", NULL
};

#define SENSOR_NAMES_MASK (SENSOR_GYRO | SENSOR_ACC | SENSOR_BARO | SENSOR_MAG)

static const char * const sensorHardwareNames[4][11] = {
    { "", "None", "MPU6050", "L3G4200D", "MPU3050", "L3GD20", "MPU6000", "MPU6500", "FAKE", NULL },
    { "", "None", "ADXL345", "MPU6050", "MMA845x", "BMA280", "LSM303DLHC", "MPU6000", "MPU6500", "FAKE", NULL },
    { "", "None", "BMP085", "MS5611", "BMP280", NULL },
    { "", "None", "HMC5883", "AK8975", "AK8963", NULL }
};
#endif

typedef struct {
    const char *name;
#ifndef SKIP_CLI_COMMAND_HELP
    const char *description;
    const char *args;
#endif
    void (*func)(struct cli *self, char *cmdline);
} clicmd_t;

#ifndef SKIP_CLI_COMMAND_HELP
#define CLI_COMMAND_DEF(name, description, args, method) \
{ \
    name , \
    description , \
    args , \
    method \
}
#else
#define CLI_COMMAND_DEF(name, description, args, method) \
{ \
    name, \
    method \
}
#endif

// should be sorted a..z for bsearch()
const clicmd_t cmdTable[] = {
    CLI_COMMAND_DEF("adjrange", "configure adjustment ranges", NULL, cliAdjustmentRange),
    CLI_COMMAND_DEF("aux", "configure modes", NULL, cliAux),
#ifdef LED_STRIP
    CLI_COMMAND_DEF("color", "configure colors", NULL, cliColor),
    CLI_COMMAND_DEF("mode_color", "configure mode and special colors", NULL, cliModeColor),
#endif
    CLI_COMMAND_DEF("defaults", "reset to defaults and reboot", NULL, cliDefaults),
    CLI_COMMAND_DEF("dump", "dump configuration",
        "[master|profile|rates]", cliDump),
    CLI_COMMAND_DEF("exit", NULL, NULL, cliExit),
    CLI_COMMAND_DEF("feature", "configure features",
        "list\r\n"
        "\t<+|->[name]", cliFeature),
#ifdef USE_FLASHFS
    CLI_COMMAND_DEF("flash_erase", "erase flash chip", NULL, cliFlashErase),
    CLI_COMMAND_DEF("flash_info", "show flash chip info", NULL, cliFlashInfo),
#ifdef USE_FLASH_TOOLS
    CLI_COMMAND_DEF("flash_read", NULL, "<length> <address>", cliFlashRead),
    CLI_COMMAND_DEF("flash_write", NULL, "<address> <message>", cliFlashWrite),
#endif
#endif
    CLI_COMMAND_DEF("get", "get variable value",
            "[name]", cliGet),
#ifdef GPS
    CLI_COMMAND_DEF("gpspassthrough", "passthrough gps to serial", NULL, cliGpsPassthrough),
#endif
    CLI_COMMAND_DEF("help", NULL, NULL, cliHelp),
#ifdef LED_STRIP
    CLI_COMMAND_DEF("led", "configure leds", NULL, cliLed),
#endif
    CLI_COMMAND_DEF("map", "configure rc channel order",
        "[<map>]", cliMap),
#if USE_QUAD_MIXER_ONLY == 0
    CLI_COMMAND_DEF("mixer", "configure mixer",
        "list\r\n"
        "\t<name>", cliMixer),
#endif
    CLI_COMMAND_DEF("mmix", "custom motor mixer", NULL, cliMotorMix),
	CLI_COMMAND_DEF("tilt", "Dynamic tilting arm configuration", 
		"mode <dynamic|static>\n"
		"in <AUX1|AUX2>\n"
		"out1 <servo channel>\n"
		"out2 <servo channel>\n", cliTilt),
    CLI_COMMAND_DEF("motor",  "get/set motor",
       "<index> [<value>]", cliMotor),
    CLI_COMMAND_DEF("play_sound", NULL,
        "[<index>]\r\n", cliPlaySound),
    CLI_COMMAND_DEF("profile", "change profile",
        "[<index>]", cliProfile),
    CLI_COMMAND_DEF("rateprofile", "change rate profile",
        "[<index>]", cliRateProfile),
    CLI_COMMAND_DEF("rxrange", "configure rx channel ranges", NULL, cliRxRange),
    CLI_COMMAND_DEF("rxfail", "show/set rx failsafe settings", NULL, cliRxFail),
    CLI_COMMAND_DEF("save", "save and reboot", NULL, cliSave),
    CLI_COMMAND_DEF("serial", "configure serial ports", NULL, cliSerial),
#ifdef USE_SERVOS
    CLI_COMMAND_DEF("servo", "configure servos", NULL, cliServo),
#endif
    CLI_COMMAND_DEF("set", "change setting",
        "[<name>=<value>]", cliSet),
#ifdef USE_SERVOS
    CLI_COMMAND_DEF("smix", "servo mixer",
        "<rule> <servo> <source> <rate> <speed> <min> <max> <box>\r\n"
        "\treset\r\n"
        "\tload <mixer>\r\n"
        "\treverse <servo> <source> r|n", cliServoMix),
#endif
#ifdef USE_SDCARD
    CLI_COMMAND_DEF("sd_info", "sdcard info", NULL, cliSdInfo),
#endif
    CLI_COMMAND_DEF("status", "show status", NULL, cliStatus),
#ifndef SKIP_TASK_STATISTICS
    CLI_COMMAND_DEF("tasks", "show task stats", NULL, cliTasks),
#endif
    CLI_COMMAND_DEF("version", "show version", NULL, cliVersion),
};
#define CMD_COUNT (sizeof(cmdTable) / sizeof(clicmd_t))

static const char * const lookupTableOffOn[] = {
    "OFF", "ON"
};

static const char * const lookupTableUnit[] = {
    "IMPERIAL", "METRIC"
};

static const char * const lookupTableAlignment[] = {
    "DEFAULT",
    "CW0",
    "CW90",
    "CW180",
    "CW270",
    "CW0FLIP",
    "CW90FLIP",
    "CW180FLIP",
    "CW270FLIP"
};

static const char * const lookupTableGPSProvider[] = {
    "NMEA", "UBLOX"
};

static const char * const lookupTableGPSSBASMode[] = {
    "AUTO", "EGNOS", "WAAS", "MSAS", "GAGAN"
};

static const char * const lookupTableCurrentSensor[] = {
    "NONE", "ADC", "VIRTUAL"
};

static const char * const lookupTableGimbalMode[] = {
    "NORMAL", "MIXTILT"
};

static const char * const lookupTablePidController[] = {
    "MW23", "MWREWRITE", "LUX"
};

static const char * const lookupTableBlackboxDevice[] = {
    "SERIAL", "SPIFLASH", "SDCARD"
};

static const char * const lookupTableSerialRX[] = {
    "SPEK1024",
    "SPEK2048",
    "SBUS",
    "SUMD",
    "SUMH",
    "XB-B",
    "XB-B-RJ01",
    "IBUS"
};

static const char * const lookupTableGyroFilter[] = {
    "OFF", "LOW", "MEDIUM", "HIGH"
};

static const char * const lookupTableGyroLpf[] = {
    "OFF",
    "188HZ",
    "98HZ",
    "42HZ",
    "20HZ",
    "10HZ"
};

typedef struct lookupTableEntry_s {
    const char * const *values;
    const uint8_t valueCount;
} lookupTableEntry_t;

typedef enum {
    TABLE_OFF_ON = 0,
    TABLE_UNIT,
    TABLE_ALIGNMENT,
    TABLE_GPS_PROVIDER,
    TABLE_GPS_SBAS_MODE,
    TABLE_BLACKBOX_DEVICE,
    TABLE_CURRENT_SENSOR,
    TABLE_GIMBAL_MODE,
    TABLE_PID_CONTROLLER,
    TABLE_SERIAL_RX,
    TABLE_GYRO_FILTER,
    TABLE_GYRO_LPF,
} lookupTableIndex_e;

static const lookupTableEntry_t lookupTables[] = {
    { lookupTableOffOn, sizeof(lookupTableOffOn) / sizeof(char *) },
    { lookupTableUnit, sizeof(lookupTableUnit) / sizeof(char *) },
    { lookupTableAlignment, sizeof(lookupTableAlignment) / sizeof(char *) },
    { lookupTableGPSProvider, sizeof(lookupTableGPSProvider) / sizeof(char *) },
    { lookupTableGPSSBASMode, sizeof(lookupTableGPSSBASMode) / sizeof(char *) },
    { lookupTableBlackboxDevice, sizeof(lookupTableBlackboxDevice) / sizeof(char *) },
    { lookupTableCurrentSensor, sizeof(lookupTableCurrentSensor) / sizeof(char *) },
    { lookupTableGimbalMode, sizeof(lookupTableGimbalMode) / sizeof(char *) },
    { lookupTablePidController, sizeof(lookupTablePidController) / sizeof(char *) },
    { lookupTableSerialRX, sizeof(lookupTableSerialRX) / sizeof(char *) },
    { lookupTableGyroFilter, sizeof(lookupTableGyroFilter) / sizeof(char *) },
    { lookupTableGyroLpf, sizeof(lookupTableGyroLpf) / sizeof(char *) },
};

#define VALUE_TYPE_OFFSET 0
#define VALUE_SECTION_OFFSET 4
#define VALUE_MODE_OFFSET 6

typedef enum {
    // value type
    VAR_UINT8 = (0 << VALUE_TYPE_OFFSET),
    VAR_INT8 = (1 << VALUE_TYPE_OFFSET),
    VAR_UINT16 = (2 << VALUE_TYPE_OFFSET),
    VAR_INT16 = (3 << VALUE_TYPE_OFFSET),
    VAR_UINT32 = (4 << VALUE_TYPE_OFFSET),
    VAR_FLOAT = (5 << VALUE_TYPE_OFFSET),

    // value section
    MASTER_VALUE = (0 << VALUE_SECTION_OFFSET),
    PROFILE_VALUE = (1 << VALUE_SECTION_OFFSET),
    CONTROL_RATE_VALUE = (2 << VALUE_SECTION_OFFSET),

    // value mode
    MODE_DIRECT = (0 << VALUE_MODE_OFFSET),
    MODE_LOOKUP = (1 << VALUE_MODE_OFFSET)
} cliValueFlag_e;

#define VALUE_TYPE_MASK (0x0F)
#define VALUE_SECTION_MASK (0x30)
#define VALUE_MODE_MASK (0xC0)

typedef struct cliMinMaxConfig_s {
    const int32_t min;
    const int32_t max;
} cliMinMaxConfig_t;

typedef struct cliLookupTableConfig_s {
    const lookupTableIndex_e tableIndex;
} cliLookupTableConfig_t;

typedef union {
    cliLookupTableConfig_t lookup;
    cliMinMaxConfig_t minmax;

} cliValueConfig_t;

typedef struct {
    const char *name;
    const uint8_t type; // see cliValueFlag_e
    const cliValueConfig_t config;

    uint16_t offset;
} __attribute__((packed)) clivalue_t;

#define CPATH(path) offsetof(struct config, path)
#define PPATH(path) offsetof(struct config_profile, path)
#define RPATH(path) offsetof(struct rate_profile, path)

const clivalue_t valueTable[] = {
    { "emf_avoidance",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON } ,		CPATH(system.emf_avoidance)},
    { "i2c_highspeed",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON } ,		CPATH(system.i2c_highspeed)},

    { "looptime",                   VAR_UINT16 | MASTER_VALUE, .config.minmax = {0, 9000},								CPATH(imu.looptime)},
    { "gyro_sync",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON } ,		CPATH(imu.gyroSync)},
    { "gyro_sync_denom",            VAR_UINT8  | MASTER_VALUE, .config.minmax = { 1,  32 } ,							CPATH(imu.gyroSyncDenominator)},
	{ "small_angle",                VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  180 } ,							CPATH(imu.small_angle)},
    { "max_angle_inclination",      VAR_UINT16 | MASTER_VALUE, .config.minmax = { 100,  900 } ,							CPATH(imu.max_angle_inclination) },
	{ "imu_dcm_kp",                 VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0,  20000 } ,							CPATH(imu.dcm_kp)},
    { "imu_dcm_ki",                 VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0,  20000 } ,							CPATH(imu.dcm_ki)},

    { "mid_rc",                     VAR_UINT16 | MASTER_VALUE, .config.minmax = { 1200,  1700 } ,						CPATH(rx.midrc)},
    { "min_check",                  VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } ,	CPATH(rx.mincheck)},
    { "max_check",                  VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } ,	CPATH(rx.maxcheck)},
    { "rssi_channel",               VAR_INT8   | MASTER_VALUE, .config.minmax = { 0,  RX_MAX_SUPPORTED_RC_CHANNELS },	CPATH(rx.rssi_channel)},
    { "rssi_scale",                 VAR_UINT8  | MASTER_VALUE, .config.minmax = { RSSI_SCALE_MIN,  RSSI_SCALE_MAX },	CPATH(rx.rssi_scale)},
    { "rssi_ppm_invert",            VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON } ,		CPATH(rx.rssi_ppm_invert)},
    { "rc_smoothing",               VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON } ,		CPATH(rx.rcSmoothing)},
    { "rx_min_usec",                VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_PULSE_MIN,  PWM_PULSE_MAX } ,		CPATH(rx.rx_min_usec)},
    { "rx_max_usec",                VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_PULSE_MIN,  PWM_PULSE_MAX } ,		CPATH(rx.rx_max_usec)},
    { "serialrx_provider",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_SERIAL_RX } ,		CPATH(rx.serialrx_provider)},
    { "sbus_inversion",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON },			CPATH(rx.sbus_inversion)},
    { "spektrum_sat_bind",          VAR_UINT8  | MASTER_VALUE, .config.minmax = { SPEKTRUM_SAT_BIND_DISABLED,  SPEKTRUM_SAT_BIND_MAX}, CPATH(rx.spektrum_sat_bind)},

    { "input_filtering_mode",       VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON } ,		CPATH(pwm_in.inputFilteringMode)},

    { "min_throttle",               VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } ,	CPATH(pwm_out.minthrottle)},
    { "max_throttle",               VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } ,	CPATH(pwm_out.maxthrottle)},
    { "min_command",                VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } ,	CPATH(pwm_out.mincommand)},
    { "servo_center_pulse",         VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } ,	CPATH(pwm_out.servoCenterPulse)},
    { "motor_pwm_rate",             VAR_UINT16 | MASTER_VALUE, .config.minmax = { 50,  32000 } ,						CPATH(pwm_out.motor_pwm_rate)},
    { "servo_pwm_rate",             VAR_UINT16 | MASTER_VALUE, .config.minmax = { 50,  498 } ,							CPATH(pwm_out.servo_pwm_rate)},


    { "3d_deadband_low",            VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } ,	CPATH(motor_3d.deadband3d_low)}, // FIXME upper limit should match code in the mixer, 1500 currently
    { "3d_deadband_high",           VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } ,	CPATH(motor_3d.deadband3d_high)}, // FIXME lower limit should match code in the mixer, 1500 currently,
    { "3d_neutral",                 VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } ,	CPATH(motor_3d.neutral3d)},

    { "retarded_arm",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON } ,		CPATH(arm.retarded_arm)},
    { "disarm_kill_switch",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON } ,		CPATH(arm.disarm_kill_switch)},
    { "auto_disarm_delay",          VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  60 } ,							CPATH(arm.auto_disarm_delay)},
    { "max_arm_angle",              VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  180 } ,							CPATH(arm.max_arm_angle)},

    { "fixedwing_althold_dir",      VAR_INT8   | MASTER_VALUE, .config.minmax = { -1,  1 },								CPATH(airplane_althold.fixedwing_althold_dir) },

    { "reboot_character",           VAR_UINT8  | MASTER_VALUE, .config.minmax = { 48,  126 } ,							CPATH(serial.reboot_character)},

    { "gps_provider",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GPS_PROVIDER },	CPATH(gps.provider)},
    { "gps_sbas_mode",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GPS_SBAS_MODE },	CPATH(gps.sbasMode)},
    { "gps_auto_config",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON },			CPATH(gps.autoConfig)},
    { "gps_auto_baud",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON },			CPATH(gps.autoBaud)},

    { "gps_pos_p",                  VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 },							PPATH(pid.P8[PIDPOS]) },
    { "gps_pos_i",                  VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 },							PPATH(pid.I8[PIDPOS]) },
    { "gps_pos_d",                  VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 },							PPATH(pid.D8[PIDPOS]) },

    { "gps_posr_p",                 VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 },							PPATH(pid.P8[PIDPOSR]) },
    { "gps_posr_i",                 VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 },							PPATH(pid.I8[PIDPOSR]) },
    { "gps_posr_d",                 VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 },							PPATH(pid.D8[PIDPOSR]) },

    { "gps_nav_p",                 VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 },							PPATH(pid.P8[PIDNAVR]) },
    { "gps_nav_i",                 VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 },							PPATH(pid.I8[PIDNAVR]) },
    { "gps_nav_d",                 VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 },							PPATH(pid.D8[PIDNAVR]) },

    { "gps_wp_radius",              VAR_UINT16 | PROFILE_VALUE, .config.minmax = { 0,  2000 },							PPATH(gps.gps_wp_radius) },

    { "nav_controls_heading",       VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON },		PPATH(gps.nav_controls_heading) },
    { "nav_speed_min",              VAR_UINT16 | PROFILE_VALUE, .config.minmax = { 10,  2000 },							PPATH(gps.nav_speed_min) },
    { "nav_speed_max",              VAR_UINT16 | PROFILE_VALUE, .config.minmax = { 10,  2000 },							PPATH(gps.nav_speed_max) },
    { "nav_slew_rate",              VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  100 },							PPATH(gps.nav_slew_rate) },

    { "telemetry_switch",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON } ,		CPATH(telemetry.telemetry_switch)},
    { "telemetry_inversion",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON } ,		CPATH(telemetry.telemetry_inversion)},

    { "frsky_default_lattitude",    VAR_FLOAT  | MASTER_VALUE, .config.minmax = { -90.0,  90.0 } ,						CPATH(frsky.gpsNoFixLatitude)},
    { "frsky_default_longitude",    VAR_FLOAT  | MASTER_VALUE, .config.minmax = { -180.0,  180.0 } ,					CPATH(frsky.gpsNoFixLongitude)},
    { "frsky_coordinates_format",   VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  FRSKY_FORMAT_NMEA } ,				CPATH(frsky.frsky_coordinate_format)},
    { "frsky_unit",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_UNIT } ,			CPATH(frsky.frsky_unit)},
    { "frsky_vfas_precision",       VAR_UINT8  | MASTER_VALUE, .config.minmax = { FRSKY_VFAS_PRECISION_LOW,  FRSKY_VFAS_PRECISION_HIGH }, CPATH(frsky.frsky_vfas_precision)},

    { "hott_alarm_sound_interval",  VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  120 } ,							CPATH(hott.hottAlarmSoundInterval)},

    { "battery_capacity",           VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0,  20000 } ,							CPATH(bat.batteryCapacity)},
    { "vbat_scale",                 VAR_UINT8  | MASTER_VALUE, .config.minmax = { VBAT_SCALE_MIN,  VBAT_SCALE_MAX } ,	CPATH(bat.vbatscale)},
    { "vbat_max_cell_voltage",      VAR_UINT8  | MASTER_VALUE, .config.minmax = { 10,  50 } ,							CPATH(bat.vbatmaxcellvoltage)},
    { "vbat_min_cell_voltage",      VAR_UINT8  | MASTER_VALUE, .config.minmax = { 10,  50 } ,							CPATH(bat.vbatmincellvoltage)},
    { "vbat_warning_cell_voltage",  VAR_UINT8  | MASTER_VALUE, .config.minmax = { 10,  50 } ,							CPATH(bat.vbatwarningcellvoltage)},
    { "current_meter_scale",        VAR_INT16  | MASTER_VALUE, .config.minmax = { -10000,  10000 } ,					CPATH(bat.currentMeterScale)},
    { "current_meter_offset",       VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0,  3300 } ,							CPATH(bat.currentMeterOffset)},
    { "multiwii_current_meter_output", VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON } ,		CPATH(bat.multiwiiCurrentMeterOutput)},
    { "current_meter_type",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_CURRENT_SENSOR },	CPATH(bat.currentMeterType)},

    { "align_gyro",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ALIGNMENT } ,		CPATH(sensors.alignment.gyro_align)},
    { "align_acc",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ALIGNMENT } ,		CPATH(sensors.alignment.acc_align)},
    { "align_mag",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ALIGNMENT } ,		CPATH(sensors.alignment.mag_align)},

    { "align_board_roll",           VAR_INT16  | MASTER_VALUE, .config.minmax = { -180,  360 } ,						CPATH(alignment.rollDegrees)},
    { "align_board_pitch",          VAR_INT16  | MASTER_VALUE, .config.minmax = { -180,  360 } ,						CPATH(alignment.pitchDegrees)},
    { "align_board_yaw",            VAR_INT16  | MASTER_VALUE, .config.minmax = { -180,  360 } ,						CPATH(alignment.yawDegrees)},

    { "gyro_lpf",                   VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GYRO_LPF } ,		CPATH(gyro.gyro_lpf)},
    { "gyro_soft_lpf",              VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0,  500 } ,							CPATH(gyro.soft_gyro_lpf_hz)},
    { "move_threshold",             VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  128 } ,							CPATH(gyro.move_threshold)},

    { "alt_hold_deadband",          VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 1,  250 } ,							PPATH(rc.alt_hold_deadband)},
    { "alt_hold_fast_change",       VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON } ,		PPATH(rc.alt_hold_fast_change)},
    { "deadband",                   VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  32 } ,							PPATH(rc.deadband)},
    { "yaw_deadband",               VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  100 } ,							PPATH(rc.yaw_deadband)},
    { "yaw_control_direction",      VAR_INT8   | PROFILE_VALUE, .config.minmax = { -1,  1 } ,							PPATH(rc.yaw_control_direction) },
    { "3d_deadband_throttle",       VAR_UINT16 | PROFILE_VALUE, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX },	PPATH(rc.deadband3d_throttle) },

    { "throttle_correction_value",  VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  150 } ,							PPATH(throttle.throttle_correction_value)},
    { "throttle_correction_angle",  VAR_UINT16 | PROFILE_VALUE, .config.minmax = { 1,  900 } ,							PPATH(throttle.throttle_correction_angle)},


    { "pid_at_min_throttle",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON } ,		CPATH(mixer.pid_at_min_throttle)},
    { "yaw_motor_direction",        VAR_INT8   | MASTER_VALUE, .config.minmax = { -1,  1 } ,							CPATH(mixer.yaw_motor_direction)},
    { "yaw_jump_prevention_limit",  VAR_UINT16 | MASTER_VALUE, .config.minmax = { YAW_JUMP_PREVENTION_LIMIT_LOW,  YAW_JUMP_PREVENTION_LIMIT_HIGH } , CPATH(mixer.yaw_jump_prevention_limit)},

    { "tri_unarmed_servo",          VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON } ,		CPATH(mixer.tri_unarmed_servo)},
    { "servo_lowpass_freq",         VAR_FLOAT  | MASTER_VALUE, .config.minmax = { 10,  400} ,							CPATH(mixer.servo_lowpass_freq)},
    { "servo_lowpass_enable",       VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON } ,		CPATH(mixer.servo_lowpass_enable)},

    { "default_rate_profile",       VAR_UINT8  | PROFILE_VALUE , .config.minmax = { 0,  MAX_CONTROL_RATE_PROFILE_COUNT - 1 } , PPATH(rate.profile_id)},

    { "rc_rate",                    VAR_UINT8  | CONTROL_RATE_VALUE, .config.minmax = { 0,  250 } ,						RPATH(rcRate8)},
    { "rc_expo",                    VAR_UINT8  | CONTROL_RATE_VALUE, .config.minmax = { 0,  100 } ,						RPATH(rcExpo8)},
    { "rc_yaw_expo",                VAR_UINT8  | CONTROL_RATE_VALUE, .config.minmax = { 0,  100 } ,						RPATH(rcYawExpo8)},
    { "thr_mid",                    VAR_UINT8  | CONTROL_RATE_VALUE, .config.minmax = { 0,  100 } ,						RPATH(thrMid8)},
    { "thr_expo",                   VAR_UINT8  | CONTROL_RATE_VALUE, .config.minmax = { 0,  100 } ,						RPATH(thrExpo8)},
    { "roll_rate",                  VAR_UINT8  | CONTROL_RATE_VALUE, .config.minmax = { 0,  CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX } ,			RPATH(rates[ROLL])},
    { "pitch_rate",                 VAR_UINT8  | CONTROL_RATE_VALUE, .config.minmax = { 0,  CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX } ,			RPATH(rates[PITCH])},
    { "yaw_rate",                   VAR_UINT8  | CONTROL_RATE_VALUE, .config.minmax = { 0,  CONTROL_RATE_CONFIG_YAW_RATE_MAX } ,				RPATH(rates[YAW])},
    { "tpa_rate",                   VAR_UINT8  | CONTROL_RATE_VALUE, .config.minmax = { 0,  CONTROL_RATE_CONFIG_TPA_MAX} ,						RPATH(dynThrPID)},
    { "tpa_breakpoint",             VAR_UINT16 | CONTROL_RATE_VALUE, .config.minmax = { PWM_RANGE_MIN,  PWM_RANGE_MAX} ,						RPATH(tpa_breakpoint)},

    { "failsafe_delay",             VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  200 } ,								CPATH(failsafe.failsafe_delay)},
    { "failsafe_off_delay",         VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  200 } ,								CPATH(failsafe.failsafe_off_delay)},
    { "failsafe_throttle",          VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_RANGE_MIN,  PWM_RANGE_MAX } ,			CPATH(failsafe.failsafe_throttle)},
    { "failsafe_kill_switch",       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON } ,			CPATH(failsafe.failsafe_kill_switch)},
    { "failsafe_throttle_low_delay",VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0,  300 } ,								CPATH(failsafe.failsafe_throttle_low_delay)},
    { "failsafe_procedure",         VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  1 } ,									CPATH(failsafe.failsafe_procedure)},

    { "gimbal_mode",                VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GIMBAL_MODE },		PPATH(gimbal.mode)},

    { "acc_hardware",               VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  ACC_MAX } ,							CPATH(sensors.selection.acc_hardware)},

    { "acc_cut_hz",                 VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 } ,								PPATH(acc.acc_cut_hz)},
    { "accxy_deadband",             VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  100 } ,								PPATH(acc.accDeadband.xy)},
    { "accz_deadband",              VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  100 } ,								PPATH(acc.accDeadband.z)},
    { "accz_lpf_cutoff",            VAR_FLOAT  | PROFILE_VALUE, .config.minmax = { 1,  20 } ,								PPATH(acc.accz_lpf_cutoff)},
    { "acc_unarmedcal",             VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON } ,			PPATH(acc.acc_unarmedcal)},
    { "acc_trim_pitch",             VAR_INT16  | PROFILE_VALUE, .config.minmax = { -300,  300 } ,							PPATH(acc.trims.values.pitch)},
    { "acc_trim_roll",              VAR_INT16  | PROFILE_VALUE, .config.minmax = { -300,  300 } ,							PPATH(acc.trims.values.roll)},

    { "baro_tab_size",              VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  BARO_SAMPLE_COUNT_MAX } ,			PPATH(baro.baro_sample_count)},
    { "baro_noise_lpf",             VAR_FLOAT  | PROFILE_VALUE, .config.minmax = { 0 , 1 } ,								PPATH(baro.baro_noise_lpf)},
    { "baro_cf_vel",                VAR_FLOAT  | PROFILE_VALUE, .config.minmax = { 0 , 1 } ,								PPATH(baro.baro_cf_vel)},
    { "baro_cf_alt",                VAR_FLOAT  | PROFILE_VALUE, .config.minmax = { 0 , 1 } ,								PPATH(baro.baro_cf_alt)},

    { "baro_hardware",              VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  BARO_MAX } ,							CPATH(sensors.selection.baro_hardware)},

    { "mag_hardware",               VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  MAG_MAX } ,							CPATH(sensors.selection.mag_hardware)},
    { "mag_declination",            VAR_INT16  | PROFILE_VALUE, .config.minmax = { -18000,  18000 } ,						PPATH(mag.mag_declination)},

    { "pid_controller",             VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_PID_CONTROLLER } ,	PPATH(pid.pidController)},

    { "p_pitch",                    VAR_UINT8  | PROFILE_VALUE, .config.minmax = { PID_MIN,  PID_MAX } ,					PPATH(pid.P8[FD_PITCH])},
    { "i_pitch",                    VAR_UINT8  | PROFILE_VALUE, .config.minmax = { PID_MIN,  PID_MAX } ,					PPATH(pid.I8[FD_PITCH])},
    { "d_pitch",                    VAR_UINT8  | PROFILE_VALUE, .config.minmax = { PID_MIN,  PID_MAX } ,					PPATH(pid.D8[FD_PITCH])},
    { "p_roll",                     VAR_UINT8  | PROFILE_VALUE, .config.minmax = { PID_MIN,  PID_MAX } ,					PPATH(pid.P8[FD_ROLL])},
    { "i_roll",                     VAR_UINT8  | PROFILE_VALUE, .config.minmax = { PID_MIN,  PID_MAX } ,					PPATH(pid.I8[FD_ROLL])},
    { "d_roll",                     VAR_UINT8  | PROFILE_VALUE, .config.minmax = { PID_MIN,  PID_MAX } ,					PPATH(pid.D8[FD_ROLL])},
    { "p_yaw",                      VAR_UINT8  | PROFILE_VALUE, .config.minmax = { PID_MIN,  PID_MAX } ,					PPATH(pid.P8[FD_YAW])},
    { "i_yaw",                      VAR_UINT8  | PROFILE_VALUE, .config.minmax = { PID_MIN,  PID_MAX } ,					PPATH(pid.I8[FD_YAW])},
    { "d_yaw",                      VAR_UINT8  | PROFILE_VALUE, .config.minmax = { PID_MIN,  PID_MAX } ,					PPATH(pid.D8[FD_YAW])},

    { "p_alt",                      VAR_UINT8  | PROFILE_VALUE, .config.minmax = { PID_MIN,  PID_MAX } ,					PPATH(pid.P8[PIDALT])},
    { "i_alt",                      VAR_UINT8  | PROFILE_VALUE, .config.minmax = { PID_MIN,  PID_MAX } ,					PPATH(pid.I8[PIDALT])},
    { "d_alt",                      VAR_UINT8  | PROFILE_VALUE, .config.minmax = { PID_MIN,  PID_MAX } ,					PPATH(pid.D8[PIDALT])},

    { "p_level",                    VAR_UINT8  | PROFILE_VALUE, .config.minmax = { PID_MIN,  PID_MAX } ,					PPATH(pid.P8[PIDLEVEL])},
    { "i_level",                    VAR_UINT8  | PROFILE_VALUE, .config.minmax = { PID_MIN,  PID_MAX } ,					PPATH(pid.I8[PIDLEVEL])},
    { "d_level",                    VAR_UINT8  | PROFILE_VALUE, .config.minmax = { PID_MIN,  PID_MAX } ,					PPATH(pid.D8[PIDLEVEL])},

    { "p_vel",                      VAR_UINT8  | PROFILE_VALUE, .config.minmax = { PID_MIN,  PID_MAX } ,					PPATH(pid.P8[PIDVEL])},
    { "i_vel",                      VAR_UINT8  | PROFILE_VALUE, .config.minmax = { PID_MIN,  PID_MAX } ,					PPATH(pid.I8[PIDVEL])},
    { "d_vel",                      VAR_UINT8  | PROFILE_VALUE, .config.minmax = { PID_MIN,  PID_MAX } ,					PPATH(pid.D8[PIDVEL])},

    { "yaw_p_limit",                VAR_UINT16 | PROFILE_VALUE, .config.minmax = { YAW_P_LIMIT_MIN, YAW_P_LIMIT_MAX } ,		PPATH(pid.yaw_p_limit)},
    { "dterm_cut_hz",               VAR_UINT16 | PROFILE_VALUE, .config.minmax = {0, 500 } ,								PPATH(pid.dterm_cut_hz)},

    { "gtune_loP_rll",              VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 10,  200 } ,								PPATH(gtune.gtune_lolimP[FD_ROLL])},
    { "gtune_loP_ptch",             VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 10,  200 } ,								PPATH(gtune.gtune_lolimP[FD_PITCH])},
    { "gtune_loP_yw",               VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 10,  200 } ,								PPATH(gtune.gtune_lolimP[FD_YAW])},
    { "gtune_hiP_rll",              VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 } ,								PPATH(gtune.gtune_hilimP[FD_ROLL])},
    { "gtune_hiP_ptch",             VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 } ,								PPATH(gtune.gtune_hilimP[FD_PITCH])},
    { "gtune_hiP_yw",               VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 } ,								PPATH(gtune.gtune_hilimP[FD_YAW])},
    { "gtune_pwr",                  VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  10 } ,								PPATH(gtune.gtune_pwr)},
    { "gtune_settle_time",          VAR_UINT16 | PROFILE_VALUE, .config.minmax = { 200,  1000 } ,							PPATH(gtune.gtune_settle_time)},
    { "gtune_average_cycles",       VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 8,  128 } ,								PPATH(gtune.gtune_average_cycles)},

    { "blackbox_rate_num",          VAR_UINT8  | MASTER_VALUE, .config.minmax = { 1,  32 } ,								CPATH(blackbox.rate_num)},
    { "blackbox_rate_denom",        VAR_UINT8  | MASTER_VALUE, .config.minmax = { 1,  32 } ,								CPATH(blackbox.rate_denom)},
    { "blackbox_device",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BLACKBOX_DEVICE } ,	CPATH(blackbox.device)},

    { "acczero_x",                  VAR_INT16  | MASTER_VALUE, .config.minmax = { -32768,  32767 } ,						CPATH(sensors.trims.accZero.raw[X])},
    { "acczero_y",                  VAR_INT16  | MASTER_VALUE, .config.minmax = { -32768,  32767 } ,						CPATH(sensors.trims.accZero.raw[Y])},
    { "acczero_z",                  VAR_INT16  | MASTER_VALUE, .config.minmax = { -32768,  32767 } ,						CPATH(sensors.trims.accZero.raw[Z])},

    { "magzero_x",                  VAR_INT16  | MASTER_VALUE, .config.minmax = { -32768,  32767 } ,						CPATH(sensors.trims.magZero.raw[X])},
    { "magzero_y",                  VAR_INT16  | MASTER_VALUE, .config.minmax = { -32768,  32767 } ,						CPATH(sensors.trims.magZero.raw[Y])},
    { "magzero_z",                  VAR_INT16  | MASTER_VALUE, .config.minmax = { -32768,  32767 } ,						CPATH(sensors.trims.magZero.raw[Z])},
};

typedef union {
    int32_t int_value;
    float float_value;
} int_float_value_t;

static void cliSetVar(struct cli *self, const clivalue_t *var, const int_float_value_t value);
static void cliPrintVar(struct cli *self, const clivalue_t *var, uint32_t full);
static void cliPrint(struct cli *self, const char *str);
static void cliPrintf(struct cli *self, const char *fmt, ...);
static void cliWrite(struct cli *self, uint8_t ch);

static void cliPrompt(struct cli *self)
{
    cliPrint(self, "\r\n# ");
    bufWriterFlush(self->cliWriter);
}

static void cliShowParseError(struct cli *self)
{
    cliPrint(self, "Parse error\r\n");
}

static void cliShowArgumentRangeError(struct cli *self, const char *name, int min, int max)
{
    cliPrintf(self, "%s must be between %d and %d\r\n", name, min, max);
}

static char *processChannelRangeArgs(char *ptr, channelRange_t *range, uint8_t *validArgumentCount)
{
    int val;

    for (int argIndex = 0; argIndex < 2; argIndex++) {
        ptr = strchr(ptr, ' ');
        if (ptr) {
            val = atoi(++ptr);
            val = CHANNEL_VALUE_TO_STEP(val);
            if (val >= MIN_MODE_RANGE_STEP && val <= MAX_MODE_RANGE_STEP) {
                if (argIndex == 0) {
                    range->startStep = val;
                } else {
                    range->endStep = val;
                }
                (*validArgumentCount)++;
            }
        }
    }

    return ptr;
}

// Check if a string's length is zero
static bool isEmpty(const char *string)
{
    return !string || *string == '\0';
}

static void cliRxFail(struct cli *self, char *cmdline){
    uint8_t channel;
    char buf[3];

    if (isEmpty(cmdline)) {
        // print out rxConfig failsafe settings
        for (channel = 0; channel < RX_MAX_SUPPORTED_RC_CHANNELS; channel++) {
            cliRxFail(self, itoa(channel, buf, 10));
        }
    } else {
        char *ptr = cmdline;
        channel = atoi(ptr++);
        if ((channel < RX_MAX_SUPPORTED_RC_CHANNELS)) {

            struct rx_failsafe_chan_config *failsafeChannelConfig = &self->config->rx_output.failsafe[channel];

            uint16_t value;
            rxFailsafeChannelType_e type = (channel < RX_NON_AUX_CHANNEL_COUNT) ? RX_FAILSAFE_TYPE_FLIGHT : RX_FAILSAFE_TYPE_AUX;
            rxFailsafeChannelMode_e mode = failsafeChannelConfig->mode;
            bool requireValue = failsafeChannelConfig->mode == RX_FAILSAFE_MODE_SET;

            ptr = strchr(ptr, ' ');
            if (ptr) {
                char *p = strchr(rxFailsafeModeCharacters, *(++ptr));
                if (p) {
                    uint8_t requestedMode = p - rxFailsafeModeCharacters;
                    mode = rxFailsafeModesTable[type][requestedMode];
                } else {
                    mode = RX_FAILSAFE_MODE_INVALID;
                }
                if (mode == RX_FAILSAFE_MODE_INVALID) {
                    cliShowParseError(self);
                    return;
                }

                requireValue = mode == RX_FAILSAFE_MODE_SET;

                ptr = strchr(ptr, ' ');
                if (ptr) {
                    if (!requireValue) {
                        cliShowParseError(self);
                        return;
                    }
                    value = atoi(++ptr);
                    value = CHANNEL_VALUE_TO_RXFAIL_STEP(value);
                    if (value > MAX_RXFAIL_RANGE_STEP) {
                        cliPrint(self, "Value out of range\r\n");
                        return;
                    }

                    failsafeChannelConfig->step = value;
                } else if (requireValue) {
                    cliShowParseError(self);
                    return;
                }
                failsafeChannelConfig->mode = mode;

            }

            char modeCharacter = rxFailsafeModeCharacters[failsafeChannelConfig->mode];

            // triple use of cliPrintf below
            // 1. acknowledge interpretation on command,
            // 2. query current setting on single item,
            // 3. recursive use for full list.

            if (requireValue) {
                cliPrintf(self, "rxfail %u %c %d\r\n",
                    channel,
                    modeCharacter,
                    RXFAIL_STEP_TO_CHANNEL_VALUE(failsafeChannelConfig->step)
                );
            } else {
                cliPrintf(self, "rxfail %u %c\r\n",
                    channel,
                    modeCharacter
                );
            }
        } else {
            cliShowArgumentRangeError(self, "channel", 0, RX_MAX_SUPPORTED_RC_CHANNELS - 1);
        }
    }
}

static void cliAux(struct cli *self, char *cmdline){
	(void)self;
    int i, val = 0;
    char *ptr;

    if (isEmpty(cmdline)) {
        // print out aux channel settings
        for (i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
            struct rc_func_range *mac = &config_get_profile_rw(self->config)->rc_funcs.ranges[i];
            cliPrintf(self, "aux %u %u %u %u %u\r\n",
                i,
                mac->modeId,
                mac->auxChannelIndex,
                MODE_STEP_TO_CHANNEL_VALUE(mac->range.startStep),
                MODE_STEP_TO_CHANNEL_VALUE(mac->range.endStep)
            );
        }
    } else {
        ptr = cmdline;
        i = atoi(ptr++);
        if (i < MAX_MODE_ACTIVATION_CONDITION_COUNT) {
            struct rc_func_range *mac = &config_get_profile_rw(self->config)->rc_funcs.ranges[i];
            uint8_t validArgumentCount = 0;
            ptr = strchr(ptr, ' ');
            if (ptr) {
                val = atoi(++ptr);
                if (val >= 0 && val < CHECKBOX_ITEM_COUNT) {
                    mac->modeId = val;
                    validArgumentCount++;
                }
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                val = atoi(++ptr);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    mac->auxChannelIndex = val;
                    validArgumentCount++;
                }
            }
            ptr = processChannelRangeArgs(ptr, &mac->range, &validArgumentCount);

            if (validArgumentCount != 4) {
                memset(mac, 0, sizeof(struct rc_func_range));
            }
        } else {
            cliShowArgumentRangeError(self, "index", 0, MAX_MODE_ACTIVATION_CONDITION_COUNT - 1);
        }
    }
}

static void cliSerial(struct cli *self, char *cmdline)
{
	(void)self;
    int i, val;
    char *ptr;

    if (isEmpty(cmdline)) {
        for (i = 0; i < SERIAL_PORT_COUNT; i++) {
			struct serial_port_config *port = &self->config->serial.portConfigs[i];
            if (!serialIsPortAvailable(port->identifier)) {
                continue;
            };
            cliPrintf(self, "serial %d %d %ld %ld %ld %ld\r\n" ,
                port->identifier,
                port->functionMask,
                baudRates[port->msp_baudrateIndex],
                baudRates[port->gps_baudrateIndex],
                baudRates[port->telemetry_baudrateIndex],
                baudRates[port->blackbox_baudrateIndex]
            );
        }
        return;
    }

    struct serial_port_config portConfig;
    memset(&portConfig, 0 , sizeof(portConfig));

    struct serial_port_config *currentConfig = NULL;
    uint8_t validArgumentCount = 0;

    ptr = cmdline;

    val = atoi(ptr++);
	for (int index = 0; index < SERIAL_PORT_COUNT; index++) {
		if(self->config->serial.portConfigs[index].identifier == val){
			currentConfig = &self->config->serial.portConfigs[index];
			break;
		}
	}

    if (currentConfig) {
        portConfig.identifier = val;
        validArgumentCount++;
    }

    ptr = strchr(ptr, ' ');
    if (ptr) {
        val = atoi(++ptr);
        portConfig.functionMask = val & 0xFFFF;
        validArgumentCount++;
    }

    for (i = 0; i < 4; i ++) {
        ptr = strchr(ptr, ' ');
        if (!ptr) {
            break;
        }

        val = atoi(++ptr);

        uint8_t baudRateIndex = lookupBaudRateIndex(val);
        if (baudRates[baudRateIndex] != (uint32_t) val) {
            break;
        }

        switch(i) {
            case 0:
                if (baudRateIndex < BAUD_9600 || baudRateIndex > BAUD_115200) {
                    continue;
                }
                portConfig.msp_baudrateIndex = baudRateIndex;
                break;
            case 1:
                if (baudRateIndex < BAUD_9600 || baudRateIndex > BAUD_115200) {
                    continue;
                }
                portConfig.gps_baudrateIndex = baudRateIndex;
                break;
            case 2:
                if (baudRateIndex != BAUD_AUTO && baudRateIndex > BAUD_115200) {
                    continue;
                }
                portConfig.telemetry_baudrateIndex = baudRateIndex;
                break;
            case 3:
                if (baudRateIndex < BAUD_19200 || baudRateIndex > BAUD_250000) {
                    continue;
                }
                portConfig.blackbox_baudrateIndex = baudRateIndex;
                break;
			default:break;
        }

        validArgumentCount++;
    }

    if (validArgumentCount < 6) {
        cliShowParseError(self);
        return;
    }

    memcpy(currentConfig, &portConfig, sizeof(portConfig));

}

static void cliAdjustmentRange(struct cli *self, char *cmdline)
{
	(void)self;
    int i, val = 0;
    char *ptr;

    if (isEmpty(cmdline)) {
        // print out adjustment ranges channel settings
        for (i = 0; i < MAX_ADJUSTMENT_RANGE_COUNT; i++) {
            adjustmentRange_t *ar = &config_get_profile_rw(self->config)->rc_adj.adjustmentRanges[i];
            cliPrintf(self, "adjrange %u %u %u %u %u %u %u\r\n",
                i,
                ar->adjustmentIndex,
                ar->auxChannelIndex,
                MODE_STEP_TO_CHANNEL_VALUE(ar->range.startStep),
                MODE_STEP_TO_CHANNEL_VALUE(ar->range.endStep),
                ar->adjustmentFunction,
                ar->auxSwitchChannelIndex
            );
        }
    } else {
        ptr = cmdline;
        i = atoi(ptr++);
        if (i < MAX_ADJUSTMENT_RANGE_COUNT) {
            adjustmentRange_t *ar = &config_get_profile_rw(self->config)->rc_adj.adjustmentRanges[i];
            uint8_t validArgumentCount = 0;

            ptr = strchr(ptr, ' ');
            if (ptr) {
                val = atoi(++ptr);
                if (val >= 0 && val < MAX_SIMULTANEOUS_ADJUSTMENT_COUNT) {
                    ar->adjustmentIndex = val;
                    validArgumentCount++;
                }
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                val = atoi(++ptr);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    ar->auxChannelIndex = val;
                    validArgumentCount++;
                }
            }

            ptr = processChannelRangeArgs(ptr, &ar->range, &validArgumentCount);

            ptr = strchr(ptr, ' ');
            if (ptr) {
                val = atoi(++ptr);
                if (val >= 0 && val < ADJUSTMENT_FUNCTION_COUNT) {
                    ar->adjustmentFunction = val;
                    validArgumentCount++;
                }
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                val = atoi(++ptr);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    ar->auxSwitchChannelIndex = val;
                    validArgumentCount++;
                }
            }

            if (validArgumentCount != 6) {
                memset(ar, 0, sizeof(adjustmentRange_t));
                cliShowParseError(self);
            }
        } else {
            cliShowArgumentRangeError(self, "index", 0, MAX_ADJUSTMENT_RANGE_COUNT - 1);
        }
    }
}

static void cliMotorMix(struct cli *self, char *cmdline)
{
	// TODO: custom mixing rules
	(void)self; (void)cmdline;
	#if 0
	if(USE_QUAD_MIXER_ONLY) return;

    int i, check = 0;
    int num_motors = 0;
    uint8_t len;
    char ftoaBuffer[FTOA_BUFFER_SIZE];
    char *ptr;

    if (!cmdline || isEmpty(cmdline)) {
        cliPrint(self, "Motor\tThr\tRoll\tPitch\tYaw\r\n");
        for (i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            if (fabsf(customMotorMixer(i)->throttle) < 1e-6f)
                break;
            num_motors++;
            cliPrintf(self, "#%d:\t", i);
            cliPrintf(self, "%s\t", ftoa(customMotorMixer(i)->throttle, ftoaBuffer));
            cliPrintf(self, "%s\t", ftoa(customMotorMixer(i)->roll, ftoaBuffer));
            cliPrintf(self, "%s\t", ftoa(customMotorMixer(i)->pitch, ftoaBuffer));
            cliPrintf(self, "%s\r\n", ftoa(customMotorMixer(i)->yaw, ftoaBuffer));
        }
        return;
    } else if (strncasecmp(cmdline, "reset", 5) == 0) {
        // erase custom mixer
        for (i = 0; i < MAX_SUPPORTED_MOTORS; i++)
            customMotorMixer(i)->throttle = 0.0f;
    } else if (strncasecmp(cmdline, "load", 4) == 0) {
        ptr = strchr(cmdline, ' ');
        if (ptr) {
            len = strlen(++ptr);
            for (i = 0; ; i++) {
                if (mixerNames[i] == NULL) {
                    cliPrint(self, "Invalid name\r\n");
                    break;
                }
                if (strncasecmp(ptr, mixerNames[i], len) == 0) {
					// TODO: fix this once we have fixed custom mixer support
                    //mixer_copy_rules(&mixer, i, customMotorMixer(0), MIXER_MAX_RULES);
                    cliPrintf(self, "Loaded %s\r\n", mixerNames[i]);
                    cliMotorMix(self, NULL);
                    break;
                }
            }
        }
    } else {
        ptr = cmdline;
        i = atoi(ptr); // get motor number
        if (i < MAX_SUPPORTED_MOTORS) {
            ptr = strchr(ptr, ' ');
            if (ptr) {
                customMotorMixer(i)->throttle = fastA2F(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                customMotorMixer(i)->roll = fastA2F(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                customMotorMixer(i)->pitch = fastA2F(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                customMotorMixer(i)->yaw = fastA2F(++ptr);
                check++;
            }
            if (check != 4) {
                cliShowParseError(self);
            } else {
                cliMotorMix(self, NULL);
            }
        } else {
            cliShowArgumentRangeError(self, "index", 0, MAX_SUPPORTED_MOTORS - 1);
        }
    }
	#endif
}

static const __attribute__((unused)) char *_channel_name(uint8_t chan){
	switch(chan){
		case ROLL: return "ROLL"; 
		case PITCH: return "PITCH"; 
		case YAW: return "YAW"; 
		case THROTTLE: return "THROTTLE";
		case AUX1: return "AUX1"; 
		case AUX2: return "AUX2"; 
		default: return "UNKNOWN";
	}
}

static void cliTilt(struct cli *self, char *cmdline)
{
	(void)self;
    struct tilt_config *tilt = &self->config->tilt;

	if(!USE_TILT){
		cliPrintf(self, "Not supported!\n"); 
	} else {
		if (isEmpty(cmdline)) {
			// print out settings
			cliPrintf(self, "mode: %s\n", (tilt->mode == MIXER_TILT_MODE_DYNAMIC)?"dynamic":"static"); 
			cliPrintf(self, "control channel: %s\n", _channel_name(tilt->control_channel)); 
			cliPrintf(self, "compensate: "); 
			if(tilt->compensation_flags & MIXER_TILT_COMPENSATE_THRUST) cliPrintf(self, "THRUST "); 
			if(tilt->compensation_flags & MIXER_TILT_COMPENSATE_TILT) cliPrintf(self, "TILT "); 
			if(tilt->compensation_flags & MIXER_TILT_COMPENSATE_BODY) cliPrintf(self, "BODY "); 
			cliPrintf(self, "\n"); 
		} else {
			char params[4][16]; 
			if(sscanf(cmdline, "%s %s", params[0], params[1]) == 2){
				if(strcmp(params[0], "mode") == 0){
					if(strcmp(params[1], "dynamic") == 0){
						tilt->mode = MIXER_TILT_MODE_DYNAMIC; 
					} else if(strcmp(params[1], "static") == 0){
						tilt->mode = MIXER_TILT_MODE_STATIC; 
					} else {
						cliPrintf(self, "Valid values: static,dynamic\n"); 
					}
				} else if(strcmp(params[0], "in") == 0){
					if(strcmp(params[1], "PITCH") == 0){
						tilt->control_channel = PITCH; 
					} else if(strcmp(params[1], "ROLL") == 0){
						tilt->control_channel = ROLL; 
					} else if(strcmp(params[1], "AUX1") == 0){
						tilt->control_channel = AUX1; 
					} else if(strcmp(params[1], "AUX2") == 0){
						tilt->control_channel = AUX2; 
					} else {
						cliPrintf(self, "Unsupported control channel!\n"); 
					}
				} else if(strcmp(params[0], "compon") == 0){
					if(strcmp(params[1], "body") == 0)
						tilt->compensation_flags |= MIXER_TILT_COMPENSATE_BODY; 
					else if(strcmp(params[1], "thrust") == 0)
						tilt->compensation_flags |= MIXER_TILT_COMPENSATE_THRUST; 
					else if(strcmp(params[1], "tilt") == 0)
						tilt->compensation_flags |= MIXER_TILT_COMPENSATE_TILT; 
					else
						cliPrintf(self, "Invalid argument!\n"); 
				} else if(strcmp(params[0], "compoff") == 0){
					if(strcmp(params[1], "body") == 0)
						tilt->compensation_flags &= ~MIXER_TILT_COMPENSATE_BODY; 
					else if(strcmp(params[1], "thrust") == 0)
						tilt->compensation_flags &= ~MIXER_TILT_COMPENSATE_THRUST; 
					else if(strcmp(params[1], "tilt") == 0)
						tilt->compensation_flags &= ~MIXER_TILT_COMPENSATE_TILT; 
					else
						cliPrintf(self, "Invalid argument!\n"); 
				} else {
					cliPrintf(self, "Invalid argument!\n"); 
				}
			} else {
				cliPrintf(self, "Invalid argument!\n"); 
			}
		}
	}
}

static void cliRxRange(struct cli *self, char *cmdline)
{
	(void)self;
    int i, validArgumentCount = 0;
    char *ptr;

    if (isEmpty(cmdline)) {
        for (i = 0; i < RX_NON_AUX_CHANNEL_COUNT; i++) {
            struct rx_channel_range_config *channelRangeConfiguration = &self->config->rx_output.range[i];
            cliPrintf(self, "rxrange %u %u %u\r\n", i, channelRangeConfiguration->min, channelRangeConfiguration->max);
        }
    }
	// TODO: reset range
	/*else if (strcasecmp(cmdline, "reset") == 0) {
        PG_RESET_CURRENT(channelRanges);
    }*/
	else {
        ptr = cmdline;
        i = atoi(ptr);
        if (i >= 0 && i < RX_NON_AUX_CHANNEL_COUNT) {
            int rangeMin, rangeMax;

            ptr = strchr(ptr, ' ');
            if (ptr) {
                rangeMin = atoi(++ptr);
                validArgumentCount++;
            }

            ptr = strchr(ptr, ' ');
            if (ptr) {
                rangeMax = atoi(++ptr);
                validArgumentCount++;
            }

            if (validArgumentCount != 2) {
                cliShowParseError(self);
            } else if (rangeMin < PWM_PULSE_MIN || rangeMin > PWM_PULSE_MAX || rangeMax < PWM_PULSE_MIN || rangeMax > PWM_PULSE_MAX) {
                cliShowParseError(self);
            } else {
                struct rx_channel_range_config *channelRangeConfiguration = &self->config->rx_output.range[i];
                channelRangeConfiguration->min = rangeMin;
                channelRangeConfiguration->max = rangeMax;
            }
        } else {
            cliShowArgumentRangeError(self, "channel", 0, RX_NON_AUX_CHANNEL_COUNT - 1);
        }
    }
}

#ifdef LED_STRIP
/*
#define CHUNK_BUFFER_SIZE 11
static bool _parse_ledstrip_config(struct ledstrip_config *self, int ledIndex, const char *config){
	static const char directionCodes[LED_DIRECTION_COUNT] = { 'N', 'E', 'S', 'W', 'U', 'D' };
	static const char functionCodes[LED_FUNCTION_COUNT]   = { 'I', 'W', 'F', 'A', 'T', 'R', 'C', 'G', 'S', 'B' };

	if (ledIndex >= LED_MAX_STRIP_LENGTH)
		return false;

	enum parseState_e {
		X_COORDINATE,
		Y_COORDINATE,
		DIRECTIONS,
		FUNCTIONS,
		RING_COLORS,
		PARSE_STATE_COUNT
	};
	static const char chunkSeparators[PARSE_STATE_COUNT] = {',', ':', ':',':', '\0'};

	struct led_config *ledConfig = &self->leds[ledIndex];
	memset(ledConfig, 0, sizeof(struct led_config));

	int x = 0, y = 0, color = 0;   // initialize to prevent warnings
	int flags = 0;
	for(enum parseState_e parseState = 0; parseState < PARSE_STATE_COUNT; parseState++) {
		char chunk[CHUNK_BUFFER_SIZE];
		{
			char chunkSeparator = chunkSeparators[parseState];
			int chunkIndex = 0;
			while (*config  && *config != chunkSeparator && chunkIndex < CHUNK_BUFFER_SIZE-1) {
				chunk[chunkIndex++] = *config++;
			}
			chunk[chunkIndex++] = 0; // zero-terminate chunk
			if (*config != chunkSeparator) {
				return false;
			}
			config++;   // skip separator
		}
		switch(parseState) {
			case X_COORDINATE:
				x = atoi(chunk);
				break;
			case Y_COORDINATE:
				y = atoi(chunk);
				break;
			case DIRECTIONS:
				for (char* ch = chunk; *ch; ch++) {
					for (ledDirectionId_e dir = 0; dir < LED_DIRECTION_COUNT; dir++) {
						if (directionCodes[dir] == *ch) {
							flags |= LED_FLAG_DIRECTION(dir);
							break;
						}
					}
				}
				break;
			case FUNCTIONS:
				for (char* ch = chunk; *ch; ch++) {
					for (ledFunctionId_e fn = 0; fn < LED_FUNCTION_COUNT; fn++) {
						if (functionCodes[fn] == *ch) {
							flags |= LED_FLAG_FUNCTION(fn);
							break;
						}
					}
				}
				break;
			case RING_COLORS:
				color = atoi(chunk);
				if (color >= LED_CONFIGURABLE_COLOR_COUNT)
					color = 0;
				break;
			default:
			case PARSE_STATE_COUNT:; // prevent warning
		}
	}
	ledSetXY(ledConfig, x, y);
	ledConfig->color = color;
	ledConfig->flags = flags;

	return true;
}

void ledstrip_genconfig(struct ledstrip *self, int ledIndex, char *ledConfigBuffer, size_t bufferSize){
	(void)self;
	char functions[LED_FUNCTION_COUNT + 1];
	char directions[LED_DIRECTION_COUNT + 1];

	const struct led_config *ledConfig = &self->config->ledstrip.leds[ledIndex];

	memset(ledConfigBuffer, 0, bufferSize);
	char *fptr = functions;
	for (ledFunctionId_e fn = 0; fn < LED_FUNCTION_COUNT; fn++) {
		if (ledConfig->flags & LED_FLAG_FUNCTION(fn)) {
			*fptr++ = functionCodes[fn];
		}
	}
	*fptr = 0;
	char *dptr = directions;
	for (ledDirectionId_e dir = 0; dir < LED_DIRECTION_COUNT; dir++) {
		if (ledConfig->flags & LED_FLAG_DIRECTION(dir)) {
			*dptr++ = directionCodes[dir];
		}
	}
	*dptr = 0;
	// TODO - check buffer length
	//sprintf(ledConfigBuffer, "%u,%u:%s:%s:%u", ledGetX(ledConfig), ledGetY(ledConfig), directions, functions, ledConfig->color);
}


*/
static void cliLed(struct cli *self, char *cmdline)
{
	(void)self;
	(void)cmdline;
	// TODO: ledstrip cli support
	/*
    int i;
    char ledConfigBuffer[20];

    if (isEmpty(cmdline)) {
        for (i = 0; i < LED_MAX_STRIP_LENGTH; i++) {
            ledstrip_reload_config(&ninja->ledstrip, i, ledConfigBuffer, sizeof(ledConfigBuffer));
            cliPrintf(self, "led %u %s\r\n", i, ledConfigBuffer);
        }
    } else {
        char *ptr = cmdline;
        i = atoi(ptr);
        if (i < LED_MAX_STRIP_LENGTH) {
            ptr = strchr(ptr, ' ');
            if (!ptr || !_parse_ledstrip_config(i, ptr + 1)) {
                cliShowParseError(self);
            }
        } else {
            cliShowArgumentRangeError(self, "index", 0, LED_MAX_STRIP_LENGTH - 1);
        }
    }
	*/
}


static void cliColor(struct cli *self, char *cmdline)
{
    int i;

    if (isEmpty(cmdline)) {
        for (i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++) {
            cliPrintf(self, "color %u %d,%u,%u\r\n",
                i,
                self->config->ledstrip.colors[i].h,
                self->config->ledstrip.colors[i].s,
                self->config->ledstrip.colors[i].v
            );
        }
    } else {
        char *ptr = cmdline;
        i = atoi(ptr);
        if (i < LED_CONFIGURABLE_COLOR_COUNT) {
            ptr = strchr(ptr, ' ');
            if (!ptr || !ledstrip_config_set_color(&self->config->ledstrip, i, ptr + 1)) {
                cliShowParseError(self);
            }
        } else {
            cliShowArgumentRangeError(self, "index", 0, LED_CONFIGURABLE_COLOR_COUNT - 1);
        }
    }
}

/*
 * Redefine a color in a mode.
 * */
static bool ledstrip_set_mode_color(struct ledstrip_config *self, ledModeIndex_e modeIndex, int modeColorIndex, int colorIndex){
	(void)self;
	// check color
	if(colorIndex < 0 || colorIndex >= LED_CONFIGURABLE_COLOR_COUNT)
		return false;
	if(modeIndex < LED_MODE_COUNT) {  // modeIndex_e is unsigned, so one-sided test is enough
		if(modeColorIndex < 0 || modeColorIndex >= LED_DIRECTION_COUNT)
			return false;
		self->modeColors[modeIndex].color[modeColorIndex] = colorIndex;
	} else if(modeIndex == LED_SPECIAL) {
		if(modeColorIndex < 0 || modeColorIndex >= LED_SPECIAL_COLOR_COUNT)
			return false;
		self->spcColors[0].color[modeColorIndex] = colorIndex;
	} else {
		return false;
	}
	return true;
}


static void cliModeColor(struct cli *self, char *cmdline)
{
    if (isEmpty(cmdline)) {
        for (int i = 0; i < LED_MODE_COUNT; i++) {
            for (int j = 0; j < LED_DIRECTION_COUNT; j++) {
                int colorIndex = self->config->ledstrip.modeColors[i].color[j];
                cliPrintf(self, "mode_color %u %u %u\r\n", i, j, colorIndex);
            }
        }

        for (int j = 0; j < LED_SPECIAL_COLOR_COUNT; j++) {
            int colorIndex = self->config->ledstrip.spcColors[0].color[j];
            cliPrintf(self, "mode_color %u %u %u\r\n", LED_SPECIAL, j, colorIndex);
        }
    } else {
        enum {MODE = 0, FUNCTION, COLOR, ARGS_COUNT};
        int args[ARGS_COUNT];
        int argNo = 0;
        char* ptr = strtok(cmdline, " ");
        while (ptr && argNo < ARGS_COUNT) {
            args[argNo++] = atoi(ptr);
            ptr = strtok(NULL, " ");
        }

        if (ptr != NULL || argNo != ARGS_COUNT) {
            cliShowParseError(self);
            return;
        }

        int modeIdx  = args[MODE];
        int funIdx = args[FUNCTION];
        int color = args[COLOR];
        if(!ledstrip_set_mode_color(&self->config->ledstrip, modeIdx, funIdx, color)) {
            cliShowParseError(self);
            return;
        }
        // values are validated
        cliPrintf(self, "mode_color %u %u %u\r\n", modeIdx, funIdx, color);
    }
}

static void cliServo(struct cli *self, char *cmdline)
{
    enum { SERVO_ARGUMENT_COUNT = 8 };
    int16_t arguments[SERVO_ARGUMENT_COUNT];

    struct servo_config *servo;

    int i;
    char *ptr;

    if (isEmpty(cmdline)) {
        // print out servo settings
        for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            servo = &config_get_profile_rw(self->config)->servos.servoConf[i];

            cliPrintf(self, "servo %u %d %d %d %d %d %d %d\r\n",
                i,
                servo->min,
                servo->max,
                servo->middle,
                servo->angleAtMin,
                servo->angleAtMax,
                servo->rate,
                servo->forwardFromChannel
            );
        }
    } else {
        int validArgumentCount = 0;

        ptr = cmdline;

        // Command line is integers (possibly negative) separated by spaces, no other characters allowed.

        // If command line doesn't fit the format, don't modify the config
        while (*ptr) {
            if (*ptr == '-' || (*ptr >= '0' && *ptr <= '9')) {
                if (validArgumentCount >= SERVO_ARGUMENT_COUNT) {
                    cliShowParseError(self);
                    return;
                }

                arguments[validArgumentCount++] = atoi(ptr);

                do {
                    ptr++;
                } while (*ptr >= '0' && *ptr <= '9');
            } else if (*ptr == ' ') {
                ptr++;
            } else {
                cliShowParseError(self);
                return;
            }
        }

        enum {INDEX = 0, MIN, MAX, MIDDLE, ANGLE_AT_MIN, ANGLE_AT_MAX, RATE, FORWARD};

        i = arguments[INDEX];

        // Check we got the right number of args and the servo index is correct (don't validate the other values)
        if (validArgumentCount != SERVO_ARGUMENT_COUNT || i < 0 || i >= MAX_SUPPORTED_SERVOS) {
            cliShowParseError(self);
            return;
        }

        servo = &config_get_profile_rw(self->config)->servos.servoConf[i];

        if (
            arguments[MIN] < PWM_PULSE_MIN || arguments[MIN] > PWM_PULSE_MAX ||
            arguments[MAX] < PWM_PULSE_MIN || arguments[MAX] > PWM_PULSE_MAX ||
            arguments[MIDDLE] < arguments[MIN] || arguments[MIDDLE] > arguments[MAX] ||
            arguments[MIN] > arguments[MAX] || arguments[MAX] < arguments[MIN] ||
            arguments[RATE] < -100 || arguments[RATE] > 100 ||
            arguments[FORWARD] >= RX_MAX_SUPPORTED_RC_CHANNELS ||
            arguments[ANGLE_AT_MIN] < 0 || arguments[ANGLE_AT_MIN] > 180 ||
            arguments[ANGLE_AT_MAX] < 0 || arguments[ANGLE_AT_MAX] > 180
        ) {
            cliShowParseError(self);
            return;
        }

        servo->min = arguments[1];
        servo->max = arguments[2];
        servo->middle = arguments[3];
        servo->angleAtMin = arguments[4];
        servo->angleAtMax = arguments[5];
        servo->rate = arguments[6];
        servo->forwardFromChannel = arguments[7];
    }
}

static void cliServoMix(struct cli *self, char *cmdline)
{
	(void)self; (void)cmdline;
	// TODO: custom servo rules
	#if 0
    int i;
    char *ptr;
    int args[8], check = 0;

    if (!cmdline || strlen(cmdline) == 0) {

        cliPrint(self, "Rule\tServo\tSource\tRate\tSpeed\tMin\tMax\tBox\r\n");

        for (i = 0; i < MAX_SERVO_RULES; i++) {
            if (customServoMixer(i)->rate == 0)
                break;

            cliPrintf(self, "#%d:\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",
                i,
                customServoMixer(i)->targetChannel,
                customServoMixer(i)->inputSource,
                customServoMixer(i)->rate,
                customServoMixer(i)->speed,
                customServoMixer(i)->min,
                customServoMixer(i)->max,
                customServoMixer(i)->box
            );
        }
        cliPrintf(self, "\r\n");
        return;
    } else if (strncasecmp(cmdline, "reset", 5) == 0) {
        // erase custom mixer
        memset(customServoMixer_arr(), 0, sizeof(*customServoMixer_arr()));
        for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            servoProfile()->servoConf[i].reversedSources = 0;
        }
    } else if (strncasecmp(cmdline, "load", 4) == 0) {
        ptr = strchr(cmdline, ' ');
        if (ptr) {
            int len = strlen(++ptr);
            for (i = 0; ; i++) {
                if (mixerNames[i] == NULL) {
                    cliPrintf(self, "Invalid name\r\n");
                    break;
                }
                if (strncasecmp(ptr, mixerNames[i], len) == 0) {
					// TODO: this needs to be fixed
                    //servoMixerLoadMix(i, customServoMixer(0));
                    cliPrintf(self, "Loaded %s\r\n", mixerNames[i]);
                    cliServoMix(self, NULL);
                    break;
                }
            }
        }
    } else if (strncasecmp(cmdline, "reverse", 7) == 0) {
        enum {SERVO = 0, INPUT, REVERSE, ARGS_COUNT};
        //int servoIndex;
		int inputSource;
        ptr = strchr(cmdline, ' ');

        int len = strlen(ptr);
        if (len == 0) {
            cliPrintf(self, "s");
            for (inputSource = 0; inputSource < MIXER_INPUT_COUNT; inputSource++)
                cliPrintf(self, "\ti%d", inputSource);
            cliPrintf(self, "\r\n");
			// TODO: this does not work currently
			/*
            for (servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS; servoIndex++) {
                cliPrintf(self, "%d", servoIndex);
                for (inputSource = 0; inputSource < INPUT_SOURCE_COUNT; inputSource++)
                    cliPrintf(self, "\t%s  ", (servoProfile()->servoConf[servoIndex].reversedSources & (1 << inputSource)) ? "r" : "n");
                cliPrintf(self, "\r\n");
            }
			*/
            return;
        }

        ptr = strtok(ptr, " ");
        while (ptr != NULL && check < ARGS_COUNT - 1) {
            args[check++] = atoi(ptr);
            ptr = strtok(NULL, " ");
        }

        if (ptr == NULL || check != ARGS_COUNT - 1) {
            cliShowParseError(self);
            return;
        }
		//TODO: this does not work right now.
		/*
        if (args[SERVO] >= 0 && args[SERVO] < MAX_SUPPORTED_SERVOS
                && args[INPUT] >= 0 && args[INPUT] < INPUT_SOURCE_COUNT
                && (*ptr == 'r' || *ptr == 'n')) {
            if (*ptr == 'r')
                servoProfile()->servoConf[args[SERVO]].reversedSources |= 1 << args[INPUT];
            else
                servoProfile()->servoConf[args[SERVO]].reversedSources &= ~(1 << args[INPUT]);
        } else
            cliShowParseError(self);
		*/
        //cliServoMix("reverse");
    } else {
        enum {RULE = 0, TARGET, INPUT, RATE, SPEED, MIN, MAX, BOX, ARGS_COUNT};
        ptr = strtok(cmdline, " ");
        while (ptr != NULL && check < ARGS_COUNT) {
            args[check++] = atoi(ptr);
            ptr = strtok(NULL, " ");
        }

        if (ptr != NULL || check != ARGS_COUNT) {
            cliShowParseError(self);
            return;
        }

        i = args[RULE];
        if (i >= 0 && i < MAX_SERVO_RULES &&
            args[TARGET] >= 0 && args[TARGET] < MAX_SUPPORTED_SERVOS &&
            args[INPUT] >= 0 && args[INPUT] < MIXER_INPUT_COUNT &&
            args[RATE] >= -100 && args[RATE] <= 100 &&
            args[SPEED] >= 0 && args[SPEED] <= MAX_SERVO_SPEED &&
            args[MIN] >= 0 && args[MIN] <= 100 &&
            args[MAX] >= 0 && args[MAX] <= 100 && args[MIN] < args[MAX] &&
            args[BOX] >= 0 && args[BOX] <= MAX_SERVO_BOXES) {
            customServoMixer(i)->targetChannel = args[TARGET];
            customServoMixer(i)->inputSource = args[INPUT];
            customServoMixer(i)->rate = args[RATE];
            customServoMixer(i)->speed = args[SPEED];
            customServoMixer(i)->min = args[MIN];
            customServoMixer(i)->max = args[MAX];
            customServoMixer(i)->box = args[BOX];
            cliServoMix(self, NULL);
        } else {
            cliShowParseError(self);
        }
    }
	#endif
}
#endif

#ifdef USE_SDCARD

static void cliWriteBytes(const uint8_t *buffer, int count)
{
    while (count > 0) {
        cliWrite(*buffer);
        buffer++;
        count--;
    }
}

static void cliSdInfo(struct cli *self, char *cmdline)
{
    UNUSED(cmdline);

    cliPrint(self, "SD card: ");

    if (!sdcard_isInserted()) {
        cliPrint(self, "None inserted\r\n");
        return;
    }

    if (!sdcard_isInitialized()) {
        cliPrint(self, "Startup failed\r\n");
        return;
    }

    const sdcardMetadata_t *metadata = sdcard_getMetadata();

    cliPrintf(self, "Manufacturer 0x%x, %ukB, %02d/%04d, v%d.%d, '",
        metadata->manufacturerID,
        metadata->numBlocks / 2, /* One block is half a kB */
        metadata->productionMonth,
        metadata->productionYear,
        metadata->productRevisionMajor,
        metadata->productRevisionMinor
    );

    cliWriteBytes((uint8_t*)metadata->productName, sizeof(metadata->productName));

    cliPrint(self, "'\r\n" "Filesystem: ");

    switch (afatfs_getFilesystemState()) {
        case AFATFS_FILESYSTEM_STATE_READY:
            cliPrint(self, "Ready");
        break;
        case AFATFS_FILESYSTEM_STATE_INITIALIZATION:
            cliPrint(self, "Initializing");
        break;
        case AFATFS_FILESYSTEM_STATE_UNKNOWN:
        case AFATFS_FILESYSTEM_STATE_FATAL:
            cliPrint(self, "Fatal");

            switch (afatfs_getLastError()) {
                case AFATFS_ERROR_BAD_MBR:
                    cliPrint(self, " - no FAT MBR partitions");
                break;
                case AFATFS_ERROR_BAD_FILESYSTEM_HEADER:
                    cliPrint(self, " - bad FAT header");
                break;
                case AFATFS_ERROR_GENERIC:
                case AFATFS_ERROR_NONE:
                    ; // Nothing more detailed to print
                break;
            }

            cliPrint(self, "\r\n");
        break;
    }
}

#endif

#ifdef USE_FLASHFS
#include "drivers/flashfs.h"
static void cliFlashInfo(struct cli *self, char *cmdline)
{
    const flashGeometry_t *layout = flashfsGetGeometry();

    UNUSED(cmdline);

    cliPrintf(self, "Flash sectors=%u, sectorSize=%u, pagesPerSector=%u, pageSize=%u, totalSize=%u, usedSize=%u\r\n",
            layout->sectors, layout->sectorSize, layout->pagesPerSector, layout->pageSize, layout->totalSize, flashfsGetOffset());
}

static void cliFlashErase(struct cli *self, char *cmdline)
{
    UNUSED(cmdline);

    cliPrintf(self, "Erasing...\r\n");
    flashfsEraseCompletely();

    while (!flashfsIsReady()) {
        usleep(100000);
    }

    cliPrintf(self, "Done.\r\n");
}

#ifdef USE_FLASH_TOOLS

static void cliFlashWrite(struct cli *self, char *cmdline)
{
    uint32_t address = atoi(cmdline);
    char *text = strchr(cmdline, ' ');

    if (!text) {
        cliShowParseError(self);
    } else {
        flashfsSeekAbs(address);
        flashfsWrite((uint8_t*)text, strlen(text), true);
        flashfsFlushSync();

        cliPrintf(self, "Wrote %u bytes at %u.\r\n", strlen(text), address);
    }
}

static void cliFlashRead(struct cli *self, char *cmdline)
{
    uint32_t address = atoi(cmdline);
    uint32_t length;
    int i;

    uint8_t buffer[32];

    char *nextArg = strchr(cmdline, ' ');

    if (!nextArg) {
        cliShowParseError(self);
    } else {
        length = atoi(nextArg);

        cliPrintf(self, "Reading %u bytes at %u:\r\n", length, address);

        while (length > 0) {
            int bytesRead;

            bytesRead = flashfsReadAbs(address, buffer, length < sizeof(buffer) ? length : sizeof(buffer));

            for (i = 0; i < bytesRead; i++) {
                cliWrite(buffer[i]);
            }

            length -= bytesRead;
            address += bytesRead;

            if (bytesRead == 0) {
                //Assume we reached the end of the volume or something fatal happened
                break;
            }
        }
        cliPrintf(self, "\r\n");
    }
}

#endif
#endif

static void dumpValues(struct cli *self, uint16_t valueSection)
{
    uint32_t i;
    const clivalue_t *value;
    for (i = 0; i < ARRAYLEN(valueTable); i++) {
        value = &valueTable[i];

        if ((value->type & VALUE_SECTION_MASK) != valueSection) {
            continue;
        }

        cliPrintf(self, "set %s = ", valueTable[i].name);
        cliPrintVar(self, value, 0);
        cliPrint(self, "\r\n");
    }
}

typedef enum {
    DUMP_MASTER = (1 << 0),
    DUMP_PROFILE = (1 << 1),
    DUMP_CONTROL_RATE_PROFILE = (1 << 2)
} dumpFlags_e;

#define DUMP_ALL (DUMP_MASTER | DUMP_PROFILE | DUMP_CONTROL_RATE_PROFILE)


static const char* const sectionBreak = "\r\n";

#define printSectionBreak() cliPrintf(self, (char *)sectionBreak)

static void cliDump(struct cli *self, char *cmdline)
{
    unsigned int i;
    char buf[16];
    uint32_t mask;

    //float thr, roll, pitch, yaw;

    uint8_t dumpMask = DUMP_ALL;
    if (strcasecmp(cmdline, "master") == 0) {
        dumpMask = DUMP_MASTER; // only
    }
    if (strcasecmp(cmdline, "profile") == 0) {
        dumpMask = DUMP_PROFILE; // only
    }
    if (strcasecmp(cmdline, "rates") == 0) {
        dumpMask = DUMP_CONTROL_RATE_PROFILE; // only
    }

    if (dumpMask & DUMP_MASTER) {

        cliPrint(self, "\r\n# version\r\n");
        cliVersion(self, NULL);

        cliPrint(self, "\r\n# dump master\r\n");
        cliPrint(self, "\r\n# mixer\r\n");

		/*
		// TODO: mixer 
		if(!USE_QUAD_MIXER_ONLY){
			cliPrintf(self, "mixer %s\r\n", mixerNames[self->config->mixer.mixerMode - 1]);
		}
		cliPrintf(self, "mmix reset\r\n");

		for (i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
			if (fabsf(customMotorMixer(i)->throttle) < 1e-6f)
				break;
			thr = customMotorMixer(i)->throttle;
			roll = customMotorMixer(i)->roll;
			pitch = customMotorMixer(i)->pitch;
			yaw = customMotorMixer(i)->yaw;
			cliPrintf(self, "mmix %d", i);
			if (thr < 0)
				cliWrite(self, ' ');
			cliPrintf(self, "%s", ftoa(thr, buf));
			if (roll < 0)
				cliWrite(self, ' ');
			cliPrintf(self, "%s", ftoa(roll, buf));
			if (pitch < 0)
				cliWrite(self, ' ');
			cliPrintf(self, "%s", ftoa(pitch, buf));
			if (yaw < 0)
				cliWrite(self, ' ');
			cliPrintf(self, "%s\r\n", ftoa(yaw, buf));
		}

#ifdef USE_SERVOS
		// print custom servo mixer if exists
		cliPrintf(self, "smix reset\r\n");

		for (i = 0; i < MAX_SERVO_RULES; i++) {

			if (customServoMixer(i)->rate == 0)
				break;

			cliPrintf(self, "smix %d %d %d %d %d %d %d %d\r\n",
				i,
				customServoMixer(i)->targetChannel,
				customServoMixer(i)->inputSource,
				customServoMixer(i)->rate,
				customServoMixer(i)->speed,
				customServoMixer(i)->min,
				customServoMixer(i)->max,
				customServoMixer(i)->box
			);
		}

	#endif
*/
        cliPrint(self, "\r\n\r\n# feature\r\n");

        mask = featureMask(self->config);
        for (i = 0; ; i++) { // disable all feature first
            if (featureNames[i] == NULL)
                break;
            cliPrintf(self, "feature -%s\r\n", featureNames[i]);
        }
        for (i = 0; ; i++) {  // reenable what we want.
            if (featureNames[i] == NULL)
                break;
            if (mask & (1 << i))
                cliPrintf(self, "feature %s\r\n", featureNames[i]);
        }

        cliPrint(self, "\r\n\r\n# map\r\n");

        for (i = 0; i < 8; i++)
            buf[self->config->rx.rcmap[i]] = rx_config_channel_letter(i);
        buf[i] = '\0';
        cliPrintf(self, "map %s\r\n", buf);

        cliPrint(self, "\r\n\r\n# serial\r\n");
        cliSerial(self, NULL);

#ifdef LED_STRIP
        cliPrint(self, "\r\n\r\n# led\r\n");
        cliLed(self, NULL);

        cliPrint(self, "\r\n\r\n# color\r\n");
        cliColor(self, NULL);

        cliPrint(self, "\r\n\r\n# mode_color\r\n");
        cliModeColor(self, NULL);
#endif
        printSectionBreak();
        dumpValues(self, MASTER_VALUE);

        cliPrint(self, "\r\n# rxfail\r\n");
        cliRxFail(self, NULL);
    }

    if (dumpMask & DUMP_PROFILE) {
        cliPrint(self, "\r\n# dump profile\r\n");

        cliPrint(self, "\r\n# profile\r\n");
        cliProfile(self, NULL);

        cliPrint(self, "\r\n# aux\r\n");

        cliAux(self, NULL);

        cliPrint(self, "\r\n# adjrange\r\n");

        cliAdjustmentRange(self, NULL);

        cliPrintf(self, "\r\n# rxrange\r\n");

        cliRxRange(self, NULL);

#ifdef USE_SERVOS
        cliPrint(self, "\r\n# servo\r\n");

        cliServo(self, NULL);

        // print servo directions

		/*
		// TODO: this currently does not work
        unsigned int channel;
        for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            for (channel = 0; channel < MIXER_INPUT_COUNT; channel++) {
                //if (servoDirection(i, channel) < 0) {
                    cliPrintf(self, "smix reverse %d %d r\r\n", i , channel);
                //}
            }
        }
		*/
#endif

        printSectionBreak();

        dumpValues(self, PROFILE_VALUE);
    }

    if (dumpMask & DUMP_CONTROL_RATE_PROFILE) {
        cliPrint(self, "\r\n# dump rates\r\n");

        cliPrint(self, "\r\n# rateprofile\r\n");
        cliRateProfile(self, NULL);

        printSectionBreak();

        dumpValues(self, CONTROL_RATE_VALUE);
    }
}

void cli_start(struct cli *self, serialPort_t *serialPort)
{
    self->cliMode = 1;
    self->cliPort = serialPort;
    setPrintfSerialPort(serialPort);
    self->cliWriter = bufWriterInit(self->cliWriteBuffer, sizeof(self->cliWriteBuffer),
                              (bufWrite_t)serialWriteBufShim, serialPort);
    
    cliPrint(self, "\r\nEntering CLI Mode, type 'exit' to return, or 'help'\r\n");
    cliPrompt(self);
}

static void cliExit(struct cli *self, char *cmdline)
{
    UNUSED(cmdline);

    cliPrint(self, "\r\nLeaving CLI mode, unsaved changes lost.\r\n");
    bufWriterFlush(self->cliWriter);
   
   // TODO: do not reboot the board when exiting cli
    *self->cliBuffer = '\0';
    self->bufferIndex = 0;
    self->cliMode = 0;
	// will basically reset the mixer (stopping all motors if any of them have been left running during test)
	// TODO: add oneshot stop motors support
	//pwmStopMotors(feature(FEATURE_ONESHOT125));
    //mixer_reset(&ninja.mixer);
    cliReboot(self);

    self->cliWriter = NULL;
}

static void cliFeature(struct cli *self, char *cmdline)
{
	(void)self;
    uint32_t i;
    uint32_t len;
    uint32_t mask;

    len = strlen(cmdline);
    mask = featureMask(self->config);

    if (len == 0) {
        cliPrint(self, "Enabled: ");
        for (i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            if (mask & (1 << i))
                cliPrintf(self, "%s ", featureNames[i]);
        }
        cliPrint(self, "\r\n");
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        cliPrint(self, "Available: ");
        for (i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            cliPrintf(self, "%s ", featureNames[i]);
        }
        cliPrint(self, "\r\n");
        return;
    } else {
        bool remove = false;
        if (cmdline[0] == '-') {
            // remove feature
            remove = true;
            cmdline++; // skip over -
            len--;
        }

        for (i = 0; ; i++) {
            if (featureNames[i] == NULL) {
                cliPrint(self, "Invalid name\r\n");
                break;
            }

            if (strncasecmp(cmdline, featureNames[i], len) == 0) {

                mask = 1 << i;
#ifndef GPS
                if (mask & FEATURE_GPS) {
                    cliPrint(self, "unavailable\r\n");
                    break;
                }
#endif
#ifndef SONAR
                if (mask & FEATURE_SONAR) {
                    cliPrint(self, "unavailable\r\n");
                    break;
                }
#endif
                if (remove) {
                    featureClear(self->config, mask);
                    cliPrint(self, "Disabled");
                } else {
                    featureSet(self->config, mask);
                    cliPrint(self, "Enabled");
                }
                cliPrintf(self, " %s\r\n", featureNames[i]);
                break;
            }
        }
    }
}

#ifdef GPS
static void cliGpsPassthrough(struct cli *self, char *cmdline)
{
    UNUSED(cmdline);
	(void)self;

	// TODO: cli gps passthrough
    //gps_enable_passthrough(self->cliPort);
}
#endif

static void cliHelp(struct cli *self, char *cmdline)
{
	(void)self;
    uint32_t i = 0;

    UNUSED(cmdline);

    for (i = 0; i < CMD_COUNT; i++) {
        cliPrint(self, cmdTable[i].name);
#ifndef SKIP_CLI_COMMAND_HELP
        if (cmdTable[i].description) {
            cliPrintf(self, " - %s", cmdTable[i].description);
        }
        if (cmdTable[i].args) {
            cliPrintf(self, "\r\n\t%s", cmdTable[i].args);
        }
#endif
        cliPrint(self, "\r\n");
    }
}

static void cliMap(struct cli *self, char *cmdline)
{
    uint32_t len;
    uint32_t i;
    char out[9];

    len = strlen(cmdline);

    if (len == 8) {
        // uppercase it
        for (i = 0; i < 8; i++)
            cmdline[i] = toupper((unsigned char)cmdline[i]);
		/*
		TODO: validate rc channel letters
        for (i = 0; i < 8; i++) {
            if (strchr(rcChannelLetters, cmdline[i]) && !strchr(cmdline + i + 1, cmdline[i]))
                continue;
            cliShowParseError(self);
            return;
        }
		*/
        rx_config_set_mapping(&self->config->rx, cmdline);
    }
    cliPrint(self, "Map: ");
    for (i = 0; i < 8; i++)
        out[self->config->rx.rcmap[i]] = rx_config_channel_letter(i);
    out[i] = '\0';
    cliPrintf(self, "%s\r\n", out);
}

static void __attribute__((unused)) cliMixer(struct cli *self, char *cmdline)
{
	if(USE_QUAD_MIXER_ONLY) return;
    int i;
    int len;

    len = strlen(cmdline);

    if (len == 0) {
        cliPrintf(self, "Mixer: %s\r\n", mixerNames[self->config->mixer.mixerMode - 1]);
        return;
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        cliPrint(self, "Available mixers: ");
        for (i = 0; ; i++) {
            if (mixerNames[i] == NULL)
                break;
            cliPrintf(self, "%s ", mixerNames[i]);
        }
        cliPrint(self, "\r\n");
        return;
    }

    for (i = 0; ; i++) {
        if (mixerNames[i] == NULL) {
            cliPrint(self, "Invalid name\r\n");
            return;
        }
        if (strncasecmp(cmdline, mixerNames[i], len) == 0) {
            self->config->mixer.mixerMode = i + 1;
            break;
        }
    }

    cliMixer(self, NULL);
}

static void cliMotor(struct cli *self, char *cmdline)
{
    int motor_index = 0;
    int motor_value = 0;
    int index = 0;
    char *pch = NULL;
    char *saveptr;

    if (isEmpty(cmdline)) {
        cliShowParseError(self);
        return;
    }

    pch = strtok_r(cmdline, " ", &saveptr);
    while (pch != NULL) {
        switch (index) {
            case 0:
                motor_index = atoi(pch);
                break;
            case 1:
                motor_value = atoi(pch);
                break;
			default:
				break;
        }
        index++;
        pch = strtok_r(NULL, " ", &saveptr);
    }

    if (motor_index < 0 || motor_index >= MAX_SUPPORTED_MOTORS) {
        cliShowArgumentRangeError(self, "index", 0, MAX_SUPPORTED_MOTORS - 1);
        return;
    }

    if (index == 2) {
        if (motor_value < PWM_RANGE_MIN || motor_value > PWM_RANGE_MAX) {
            cliShowArgumentRangeError(self, "value", 1000, 2000);
            return;
        } else {
			mixer_input_command(&self->ninja->mixer, MIXER_INPUT_GROUP_MOTOR_PASSTHROUGH + index,  motor_value);
        }
    }

	// TODO: read motor output
    //cliPrintf(self, "motor %d: %d\r\n", motor_index, mixer_get_motor_value(&ninja.mixer, motor_index));
}

static void cliPlaySound(struct cli *self, char *cmdline)
{
	(void)self; (void)cmdline;
#if 0
// TODO: beeper play sound cli
    int i;
    const char *name;
    static int lastSoundIdx = -1;

    if (isEmpty(cmdline)) {
        i = lastSoundIdx + 1;     //next sound index
        if ((name=beeperNameForTableIndex(i)) == NULL) {
            while (true) {   //no name for index; try next one
                if (++i >= beeperTableEntryCount())
                    i = 0;   //if end then wrap around to first entry
                if ((name=beeperNameForTableIndex(i)) != NULL)
                    break;   //if name OK then play sound below
                if (i == lastSoundIdx + 1) {     //prevent infinite loop
                    cliPrintf(self, "Error playing sound\r\n");
                    return;
                }
            }
        }
    } else {       //index value was given
        i = atoi(cmdline);
        if ((name=beeperNameForTableIndex(i)) == NULL) {
            cliPrintf(self, "No sound for index %d\r\n", i);
            return;
        }
    }
    lastSoundIdx = i;
    beeperSilence();
    cliPrintf(self, "Playing sound %d: %s\r\n", i, name);
    beeper(beeperModeForTableIndex(i));
#endif
}

static void cliProfile(struct cli *self, char *cmdline)
{
    int i;

    if (isEmpty(cmdline)) {
        cliPrintf(self, "profile %d\r\n", self->config->profile.profile_id);
        return;
    } else {
        i = atoi(cmdline);
        if (i >= 0 && i < MAX_PROFILE_COUNT) {
			// TODO: chane profile
            //changeProfile(self, i);
            cliProfile(self, NULL);
        }
    }
}

static void cliRateProfile(struct cli *self, char *cmdline)
{
    int i;

    if (isEmpty(cmdline)) {
        //cliPrintf(self, "rateprofile %d\r\n", getCurrentControlRateProfile());
        return;
    } else {
        i = atoi(cmdline);
        if (i >= 0 && i < MAX_CONTROL_RATE_PROFILE_COUNT) {
            //changeControlRateProfile(i);
			// TODO: cli change control rate profile
            cliRateProfile(self, NULL);
        }
    }
}

static void cliReboot(struct cli *self)
{
    cliPrint(self, "\r\nRebooting");
    bufWriterFlush(self->cliWriter);
    waitForSerialPortToFinishTransmitting(self->cliPort);
	// TODO: pwm oneshot stop
	//pwmStopMotors(feature(FEATURE_ONESHOT125));
    //handleOneshotFeatureChangeOnRestart();
    systemReset();
}

static void cliSave(struct cli *self, char *cmdline)
{
    UNUSED(cmdline);

    cliPrint(self, "Saving");
	ninja_config_save(self->ninja);
    cliReboot(self);
}

static void cliDefaults(struct cli *self, char *cmdline)
{
    UNUSED(cmdline);

    cliPrint(self, "Resetting to defaults");
	ninja_config_reset(self->ninja);
    cliReboot(self);
}

static void cliPrint(struct cli *self, const char *str)
{
    while (*str)
        bufWriterAppend(self->cliWriter, *str++);
}

static void cliPutp(void *p, char ch)
{
    bufWriterAppend(p, ch);
}

static void cliPrintf(struct cli *self, const char *fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    tfp_format(self->cliWriter, cliPutp, fmt, va);
    va_end(va);
}

static void cliWrite(struct cli *self, uint8_t ch)
{
    bufWriterAppend(self->cliWriter, ch);
}

static void* cliVarPtr(struct cli *self, const clivalue_t *var)
{
	(void)self;

    switch (var->type & VALUE_SECTION_MASK) {
        case MASTER_VALUE:
            return ((uint8_t*)&self->config) + var->offset;
        case CONTROL_RATE_VALUE:
            return ((uint8_t*)config_get_rate_profile(self->config)) + var->offset;
        case PROFILE_VALUE:
            return ((uint8_t*)config_get_profile(self->config)) + var->offset;
		default:break;
    }
    return NULL;
}

static void cliPrintVar(struct cli *self, const clivalue_t *var, uint32_t full)
{
    int32_t value = 0;
    char ftoaBuffer[FTOA_BUFFER_SIZE];

    void *ptr = cliVarPtr(self, var);

    if (!ptr) {
        return;
    }

    switch (var->type & VALUE_TYPE_MASK) {
        case VAR_UINT8:
            value = *(uint8_t*)ptr;
            break;

        case VAR_INT8:
            value = *(int8_t *)ptr;
            break;

        case VAR_UINT16:
            value = *(uint16_t *)ptr;
            break;

        case VAR_INT16:
            value = *(int16_t *)ptr;
            break;

        case VAR_UINT32:
            value = *(uint32_t *)ptr;
            break;

        case VAR_FLOAT:
            cliPrintf(self, "%s", ftoa(*(float *)ptr, ftoaBuffer));
            if (full && (var->type & VALUE_MODE_MASK) == MODE_DIRECT) {
                cliPrintf(self, " %s", ftoa((float)var->config.minmax.min, ftoaBuffer));
                cliPrintf(self, " %s", ftoa((float)var->config.minmax.max, ftoaBuffer));
            }
            return; // return from case for float only
		default:break;
    }

    switch(var->type & VALUE_MODE_MASK) {
        case MODE_DIRECT:
            cliPrintf(self, "%d", value);
            if (full) {
                cliPrintf(self, " %d %d", var->config.minmax.min, var->config.minmax.max);
            }
            break;
        case MODE_LOOKUP:
            cliPrintf(self, lookupTables[var->config.lookup.tableIndex].values[value]);
            break;
		default:break;
    }
}

static void cliSetVar(struct cli *self, const clivalue_t *var, const int_float_value_t value)
{
    void *ptr = cliVarPtr(self, var);

    if (!ptr) {
        return;
    }

    switch (var->type & VALUE_TYPE_MASK) {
        case VAR_UINT8:
        case VAR_INT8:
            *(int8_t *)ptr = value.int_value;
            break;

        case VAR_UINT16:
        case VAR_INT16:
            *(int16_t *)ptr = value.int_value;
            break;

        case VAR_UINT32:
            *(uint32_t *)ptr = value.int_value;
            break;

        case VAR_FLOAT:
            *(float *)ptr = (float)value.float_value;
            break;
		default:break;
    }
}

static void cliSet(struct cli *self, char *cmdline)
{
    uint32_t i;
    uint32_t len;
    const clivalue_t *val;
    char *eqptr = NULL;

    len = strlen(cmdline);

    if (len == 0 || (len == 1 && cmdline[0] == '*')) {
        cliPrint(self, "Current settings: \r\n");
        for (i = 0; i < ARRAYLEN(valueTable); i++) {
            val = &valueTable[i];
            cliPrintf(self, "%s = ", valueTable[i].name);
            cliPrintVar(self, val, len); // when len is 1 (when * is passed as argument), it will print min/max values as well, for gui
            cliPrint(self, "\r\n");
        }
    } else if ((eqptr = strstr(cmdline, "=")) != NULL) {
        // has equals

        char *lastNonSpaceCharacter = eqptr;
        while (*(lastNonSpaceCharacter - 1) == ' ') {
            lastNonSpaceCharacter--;
        }
        uint8_t variableNameLength = lastNonSpaceCharacter - cmdline;

        // skip the '=' and any ' ' characters
        eqptr++;
        while (*(eqptr) == ' ') {
            eqptr++;
        }

        for (i = 0; i < ARRAYLEN(valueTable); i++) {
            val = &valueTable[i];
            // ensure exact match when setting to prevent setting variables with shorter names
            if (strncasecmp(cmdline, valueTable[i].name, strlen(valueTable[i].name)) == 0 && variableNameLength == strlen(valueTable[i].name)) {

                bool changeValue = false;
                int_float_value_t tmp;
				tmp.int_value = 0; 
                switch (valueTable[i].type & VALUE_MODE_MASK) {
                    case MODE_DIRECT: {
                            int32_t value = 0;
                            float valuef = 0;

                            value = atoi(eqptr);
                            valuef = fastA2F(eqptr);

                            if (valuef >= valueTable[i].config.minmax.min && valuef <= valueTable[i].config.minmax.max) { // note: compare float value

                                if ((valueTable[i].type & VALUE_TYPE_MASK) == VAR_FLOAT)
                                    tmp.float_value = valuef;
                                else
                                    tmp.int_value = value;

                                changeValue = true;
                            }
                        }
                        break;
                    case MODE_LOOKUP: {
                            const lookupTableEntry_t *tableEntry = &lookupTables[valueTable[i].config.lookup.tableIndex];
                            bool matched = false;
                            for (uint8_t tableValueIndex = 0; tableValueIndex < tableEntry->valueCount && !matched; tableValueIndex++) {
                                matched = strcasecmp(tableEntry->values[tableValueIndex], eqptr) == 0;

                                if (matched) {
                                    tmp.int_value = tableValueIndex;
                                    changeValue = true;
                                }
                            }
                        }
                        break;
					default:break;
                }

                if (changeValue) {
                    cliSetVar(self, val, tmp);

                    cliPrintf(self, "%s set to ", valueTable[i].name);
                    cliPrintVar(self, val, 0);
                } else {
                    cliPrint(self, "Invalid value\r\n");
                }

                return;
            }
        }
        cliPrint(self, "Invalid name\r\n");
    } else {
        // no equals, check for matching variables.
        cliGet(self, cmdline);
    }
}

static void cliGet(struct cli *self, char *cmdline)
{
    uint32_t i;
    const clivalue_t *val;
    int matchedCommands = 0;

    for (i = 0; i < ARRAYLEN(valueTable); i++) {
        if (strstr(valueTable[i].name, cmdline)) {
            val = &valueTable[i];
            cliPrintf(self, "%s = ", valueTable[i].name);
            cliPrintVar(self, val, 0);
            cliPrint(self, "\r\n");

            matchedCommands++;
        }
    }


    if (matchedCommands) {
    	return;
    }

    cliPrint(self, "Invalid name\r\n");
}

static void cliStatus(struct cli *self, char *cmdline)
{
    UNUSED(cmdline);

    cliPrintf(self, "System Uptime: %d seconds, Voltage: %d * 0.1V (%dS battery - %s), System load: %d.%02d\r\n",
        sys_millis(self->ninja->system) / 1000,
        battery_get_voltage(&self->ninja->bat),
        battery_get_cell_count(&self->ninja->bat),
        battery_get_state_str(&self->ninja->bat),
        ninja_sched_get_load(&self->ninja->sched) / 100,
        ninja_sched_get_load(&self->ninja->sched) % 100
    );

	// TODO: cli core clock
    //cliPrintf(self, "CPU Clock=%dMHz", (SystemCoreClock / 1000000));

#ifndef CJMCU
#if 0
	// TODO: to print sensors, use a less intrusive way
    uint8_t i;
    uint32_t mask;
    uint32_t detectedSensorsMask = sensorsMask();

    for (i = 0; ; i++) {

        if (sensorTypeNames[i] == NULL)
            break;

        mask = (1 << i);
        if ((detectedSensorsMask & mask) && (mask & SENSOR_NAMES_MASK)) {
            const char *sensorHardware;
            uint8_t sensorHardwareIndex = detectedSensors[i];
            sensorHardware = sensorHardwareNames[i][sensorHardwareIndex];

            cliPrintf(self, ", %s=%s", sensorTypeNames[i], sensorHardware);

            if (mask == SENSOR_ACC && acc.revisionCode) {
                cliPrintf(self, ".%c", acc.revisionCode);
            }
        }
    }
#endif
#endif
    cliPrint(self, "\r\n");

#ifdef USE_I2C
    uint16_t i2cErrorCounter = i2cGetErrorCounter();
#else
    uint16_t i2cErrorCounter = 0;
#endif

    //cliPrintf(self, "Cycle Time: %d, I2C Errors: %d, registry size: %d\r\n", cycleTime, i2cErrorCounter, PG_REGISTRY_SIZE);
    cliPrintf(self, "I2C Errors: %d, registry size: %d\r\n", i2cErrorCounter, sizeof(struct config));
}

#ifndef SKIP_TASK_STATISTICS
#include <FreeRTOS.h>
#include <task.h>
static void cliTasks(struct cli *self, char *cmdline)
{
    UNUSED(cmdline);

    cfTaskId_e taskId;
    cfTaskInfo_t taskInfo;

	cliPrintf(self, "current time: %u\r\n", sys_micros(self->system));
    cliPrintf(self, "Task list          max/us  avg/us rate/hz maxload avgload     total/ms\r\n");
    for (taskId = 0; taskId < TASK_COUNT; taskId++) {
        ninja_sched_get_task_info(&self->ninja->sched, taskId, &taskInfo);
        if (taskInfo.isEnabled) {
            const int taskFrequency = (int)(1000000.0f / ((float)taskInfo.latestDeltaTime));
            const int maxLoad = (taskInfo.maxExecutionTime * taskFrequency + 5000) / 1000;
            const int averageLoad = (taskInfo.averageExecutionTime * taskFrequency + 5000) / 1000;
            cliPrintf(self, "%2d - %12s  %6d   %5d   %5d %4d.%1d%% %4d.%1d%%  %8d\r\n",
                    taskId, taskInfo.taskName, taskInfo.maxExecutionTime, taskInfo.averageExecutionTime,
                    taskFrequency, maxLoad/10, maxLoad%10, averageLoad/10, averageLoad%10, taskInfo.totalExecutionTime / 1000);
        }
    }
	// realtime tasks
	TaskStatus_t status[4]; // 4 is just arbitrary.
	uint32_t total_time;
	int16_t ret = uxTaskGetSystemState(status, sizeof(status)/sizeof(status[0]), &total_time);
	cliPrintf(self, "== realtime tasks\r\n");
	cliPrintf(self, "time: %lu\r\n", total_time);
	cliPrintf(self, "heap: %lu free of %lu bytes\r\n", xPortGetFreeHeapSize(), configTOTAL_HEAP_SIZE);
	cliPrintf(self, "%17s %8s\r\n", "name", "stack");
	if(ret > 0){
		for(int c = 0; c < ret; c++){
			TaskStatus_t *stat = &status[c];
            cliPrintf(self, "%2d - %12s  %6d\r\n",
					c, stat->pcTaskName, stat->usStackHighWaterMark);
		}
	} else {
		cliPrintf(self, "(none)\n");
	}
}
#endif

static void cliVersion(struct cli *self, char *cmdline)
{
    UNUSED(cmdline);
	cliPrintf(self, "# Ninjaflight");
	// TODO: build date etc for sitl
/*
    cliPrintf(self, "# Ninjaflight/%s %s %s / %s (%s)",
        targetName,
        FC_VERSION_STRING,
        buildDate,
        buildTime,
        shortGitRevision
    );
	*/
}

void cli_update(struct cli *self)
{
    if (!self->cliWriter) {
        return;
    }

    // Be a little bit tricky.  Flush the last inputs buffer, if any.
    bufWriterFlush(self->cliWriter);
    
    while (serialRxBytesWaiting(self->cliPort)) {
        uint8_t c = serialRead(self->cliPort);
        if (c == '\t' || c == '?') {
            // do tab completion
            const clicmd_t *cmd, *pstart = NULL, *pend = NULL;
            uint32_t i = self->bufferIndex;
            for (cmd = cmdTable; cmd < cmdTable + CMD_COUNT; cmd++) {
                if (self->bufferIndex && (strncasecmp(self->cliBuffer, cmd->name, self->bufferIndex) != 0))
                    continue;
                if (!pstart)
                    pstart = cmd;
                pend = cmd;
            }
            if (pstart) {    /* Buffer matches one or more commands */
                for (; ; self->bufferIndex++) {
                    if (pstart->name[self->bufferIndex] != pend->name[self->bufferIndex])
                        break;
                    if (!pstart->name[self->bufferIndex] && self->bufferIndex < sizeof(self->cliBuffer) - 2) {
                        /* Unambiguous -- append a space */
                        self->cliBuffer[self->bufferIndex++] = ' ';
                        self->cliBuffer[self->bufferIndex] = '\0';
                        break;
                    }
                    self->cliBuffer[self->bufferIndex] = pstart->name[self->bufferIndex];
                }
            }
            if (!self->bufferIndex || pstart != pend) {
                /* Print list of ambiguous matches */
                cliPrint(self, "\r\033[K");
                for (cmd = pstart; cmd <= pend; cmd++) {
                    cliPrint(self, cmd->name);
                    cliWrite(self, '\t');
                }
                cliPrompt(self);
                i = 0;    /* Redraw prompt */
            }
            for (; i < self->bufferIndex; i++)
                cliWrite(self, self->cliBuffer[i]);
        } else if (!self->bufferIndex && c == 4) {   // CTRL-D
            cliExit(self, self->cliBuffer);
            return;
        } else if (c == 12) {                  // NewPage / CTRL-L
            // clear screen
            cliPrint(self, "\033[2J\033[1;1H");
            cliPrompt(self);
        } else if (self->bufferIndex && (c == '\n' || c == '\r')) {
            // enter pressed
            cliPrint(self, "\r\n");

            // Strip comment starting with # from line
            char *p = self->cliBuffer;
            p = strchr(p, '#');
            if (NULL != p) {
                self->bufferIndex = (uint32_t)(p - self->cliBuffer);
            }

            // Strip trailing whitespace
            while (self->bufferIndex > 0 && self->cliBuffer[self->bufferIndex - 1] == ' ') {
                self->bufferIndex--;
            }

            // Process non-empty lines
            if (self->bufferIndex > 0) {
                self->cliBuffer[self->bufferIndex] = 0; // null terminate

                const clicmd_t *cmd;
                for (cmd = cmdTable; cmd < cmdTable + CMD_COUNT; cmd++) {
                    if(!strncasecmp(self->cliBuffer, cmd->name, strlen(cmd->name))   // command names match
                       && !isalnum((unsigned)self->cliBuffer[strlen(cmd->name)]))    // next characted in bufffer is not alphanumeric (command is correctly terminated)
                        break;
                }
                if(cmd < cmdTable + CMD_COUNT)
                    cmd->func(self, self->cliBuffer + strlen(cmd->name) + 1);
                else
                    cliPrint(self, "Unknown command, try 'help'");
                self->bufferIndex = 0;
            }

            memset(self->cliBuffer, 0, sizeof(self->cliBuffer));

            // 'exit' will reset this flag, so we don't need to print prompt again
            if (!self->cliMode)
                return;

            cliPrompt(self);
        } else if (c == 127) {
            // backspace
            if (self->bufferIndex) {
                self->cliBuffer[--self->bufferIndex] = 0;
                cliPrint(self, "\010 \010");
            }
        } else if (self->bufferIndex < sizeof(self->cliBuffer) && c >= 32 && c <= 126) {
            if (!self->bufferIndex && c == ' ')
                continue; // Ignore leading spaces
            self->cliBuffer[self->bufferIndex++] = c;
            cliWrite(self, c);
        }
    }
}

bool cli_is_active(struct cli *self){
	return self->cliMode;
}

void cli_init(struct cli *self, struct ninja *ninja, struct config *cfg, const struct system_calls *system){
	memset(self, 0, sizeof(struct cli));
	self->ninja = ninja;
	self->config = cfg;
	self->system = system;
}

/** @} */
/** @} */
