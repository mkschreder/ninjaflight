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

#include "io/rc_controls.h"
#include "io/rc_adjustments.h"
#include "io/beeper.h"
#include "io/serial.h"

#include "sensors/sensors.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"

#include "telemetry/telemetry.h"

#include "flight/rate_profile.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/anglerate_controller.h"
#include "flight/navigation.h"


// FIXME remove the includes below when target specific configuration is moved out of this file
#include "sensors/battery.h"
#include "io/motor_and_servo.h"

// BLACKBOX
#include "blackbox/blackbox_io.h"
PG_REGISTER_WITH_RESET_TEMPLATE(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 0);

#ifdef ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_FLASH
#elif defined(ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT)
#define DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_SDCARD
#else
#define DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_SERIAL
#endif

PG_RESET_TEMPLATE(blackboxConfig_t, blackboxConfig,
        .device = DEFAULT_BLACKBOX_DEVICE,
        .rate_num = 1,
        .rate_denom = 1,
);

// BATTERY
PG_REGISTER_WITH_RESET_TEMPLATE(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 0);

PG_RESET_TEMPLATE(batteryConfig_t, batteryConfig,
    .vbatscale = VBAT_SCALE_DEFAULT,
    .vbatresdivval = VBAT_RESDIVVAL_DEFAULT,
    .vbatresdivmultiplier = VBAT_RESDIVMULTIPLIER_DEFAULT,
    .vbatmaxcellvoltage = 43,
    .vbatmincellvoltage = 33,
    .vbatwarningcellvoltage = 35,
    .currentMeterScale = 400, // for Allegro ACS758LCB-100U (40mV/A)
    .currentMeterType = CURRENT_SENSOR_ADC,
);

// BOARD
PG_REGISTER(struct board_alignment_config, boardAlignment, PG_BOARD_ALIGNMENT, 0);

// PID 
#ifndef DEFAULT_RX_FEATURE
#define DEFAULT_RX_FEATURE FEATURE_RX_PARALLEL_PWM
#endif

PG_REGISTER_PROFILE_WITH_RESET_TEMPLATE(struct pid_config, pidProfile, PG_PID_PROFILE, 0);

PG_RESET_TEMPLATE(struct pid_config, pidProfile,
    .pidController = PID_CONTROLLER_MWREWRITE,
    .P8[PIDROLL] = 40,
    .I8[PIDROLL] = 30,
    .D8[PIDROLL] = 23,
    .P8[PIDPITCH] = 40,
    .I8[PIDPITCH] = 30,
    .D8[PIDPITCH] = 23,
    .P8[PIDYAW] = 85,
    .I8[PIDYAW] = 45,
    .D8[PIDYAW] = 0,
    .P8[PIDALT] = 50,
    .I8[PIDALT] = 0,
    .D8[PIDALT] = 0,
    .P8[PIDPOS] = 15,   // POSHOLD_P * 100
    .I8[PIDPOS] = 0,    // POSHOLD_I * 100
    .D8[PIDPOS] = 0,
    .P8[PIDPOSR] = 34,  // POSHOLD_RATE_P * 10
    .I8[PIDPOSR] = 14,  // POSHOLD_RATE_I * 100
    .D8[PIDPOSR] = 53,  // POSHOLD_RATE_D * 1000
    .P8[PIDNAVR] = 25,  // NAV_P * 10
    .I8[PIDNAVR] = 33,  // NAV_I * 100
    .D8[PIDNAVR] = 83,  // NAV_D * 1000
    .P8[PIDLEVEL] = 20,
    .I8[PIDLEVEL] = 10,
    .D8[PIDLEVEL] = 100,
    .P8[PIDMAG] = 40,
    .P8[PIDVEL] = 120,
    .I8[PIDVEL] = 45,
    .D8[PIDVEL] = 1,

    .yaw_p_limit = YAW_P_LIMIT_MAX,
    .dterm_cut_hz = 0,
);

// SENSORS
PG_REGISTER(sensorSelectionConfig_t, sensorSelectionConfig, PG_SENSOR_SELECTION_CONFIG, 0);
PG_REGISTER(sensorAlignmentConfig_t, sensorAlignmentConfig, PG_SENSOR_ALIGNMENT_CONFIG, 0);
PG_REGISTER(sensorTrims_t, sensorTrims, PG_SENSOR_TRIMS, 0);

// ACCELEROMETER
PG_REGISTER_PROFILE_WITH_RESET_FN(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);

void pgResetFn_accelerometerConfig(accelerometerConfig_t *instance)
{
    RESET_CONFIG_2(accelerometerConfig_t, instance,
        .acc_cut_hz = 15,
        .accz_lpf_cutoff = 5.0f,
        .accDeadband.z = 40,
        .accDeadband.xy = 40,
        .acc_unarmedcal = 1,
    );
    resetRollAndPitchTrims(&instance->accelerometerTrims);
}

// RATE PROFILE
static void pgResetFn_controlRateProfiles(struct rate_config *instance){
    for (int i = 0; i < MAX_CONTROL_RATE_PROFILE_COUNT; i++) {
        RESET_CONFIG(struct rate_config, &instance[i],
            .rcRate8 = 90,
            .rcExpo8 = 65,
            .thrMid8 = 50,
            .tpa_breakpoint = 1500,
        );
    }
}

PG_REGISTER_PROFILE(rateProfileSelection_t, rateProfileSelection, PG_RATE_PROFILE_SELECTION, 0);
PG_REGISTER_ARR_WITH_RESET_FN(struct rate_config, MAX_CONTROL_RATE_PROFILE_COUNT, controlRateProfiles, PG_CONTROL_RATE_PROFILES, 0);

// MIXER
PG_REGISTER_ARR(struct motor_mixer, MAX_SUPPORTED_MOTORS, customMotorMixer, PG_MOTOR_MIXER, 0);
PG_REGISTER_WITH_RESET_TEMPLATE(struct mixer_config, mixerConfig, PG_MIXER_CONFIG, 0);
PG_REGISTER_WITH_RESET_TEMPLATE(struct motor_3d_config, motor3DConfig, PG_MOTOR_3D_CONFIG, 0);

PG_RESET_TEMPLATE(struct motor_3d_config, motor3DConfig,
    .deadband3d_low = 1406,
    .deadband3d_high = 1514,
    .neutral3d = 1460,
);

// MOTOR SERVO
PG_REGISTER_WITH_RESET_TEMPLATE(motorAndServoConfig_t, motorAndServoConfig, PG_MOTOR_AND_SERVO_CONFIG, 0);

#define BRUSHED_MOTORS_PWM_RATE 16000
#define BRUSHLESS_MOTORS_PWM_RATE 400

#ifdef BRUSHED_MOTORS
#define DEFAULT_PWM_RATE BRUSHED_MOTORS_PWM_RATE
#else
#define DEFAULT_PWM_RATE BRUSHLESS_MOTORS_PWM_RATE
#endif

PG_RESET_TEMPLATE(motorAndServoConfig_t, motorAndServoConfig,
    .minthrottle = 1150,
    .maxthrottle = 1850,
    .mincommand = 1000,
    .servoCenterPulse = 1500,
    .motor_pwm_rate = DEFAULT_PWM_RATE,
    .servo_pwm_rate = 50,
);


PG_REGISTER_ARR(servoMixer_t, MAX_SERVO_RULES, customServoMixer, PG_SERVO_MIXER, 0);

PG_REGISTER_PROFILE_WITH_RESET_FN(servoProfile_t, servoProfile, PG_SERVO_PROFILE, 0);

void pgResetFn_servoProfile(servoProfile_t *instance)
{
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        RESET_CONFIG(servoParam_t, &instance->servoConf[i],
            .min = DEFAULT_SERVO_MIN,
            .max = DEFAULT_SERVO_MAX,
            .middle = DEFAULT_SERVO_MIDDLE,
            .rate = 100,
            .angleAtMin = DEFAULT_SERVO_MIN_ANGLE,
            .angleAtMax = DEFAULT_SERVO_MAX_ANGLE,
            .forwardFromChannel = CHANNEL_FORWARDING_DISABLED,
        );
    }
}

// NAVIGATION
PG_REGISTER_PROFILE_WITH_RESET_TEMPLATE(gpsProfile_t, gpsProfile, PG_NAVIGATION_CONFIG, 0);

PG_RESET_TEMPLATE(gpsProfile_t, gpsProfile,
    .gps_wp_radius = 200,
    .gps_lpf = 20,
    .nav_slew_rate = 30,
    .nav_controls_heading = 1,
    .nav_speed_min = 100,
    .nav_speed_max = 300,
    .ap_mode = 40,
);

// GTUNE
#ifdef GTUNE
PG_REGISTER_PROFILE_WITH_RESET_TEMPLATE(gtuneConfig_t, gtuneConfig, PG_GTUNE_CONFIG, 0);

PG_RESET_TEMPLATE(gtuneConfig_t, gtuneConfig,
    .gtune_lolimP[FD_ROLL] = 10,          // [0..200] Lower limit of ROLL P during G tune.
    .gtune_lolimP[FD_PITCH] = 10,         // [0..200] Lower limit of PITCH P during G tune.
    .gtune_lolimP[FD_YAW] = 10,           // [0..200] Lower limit of YAW P during G tune.
    .gtune_hilimP[FD_ROLL] = 100,         // [0..200] Higher limit of ROLL P during G tune. 0 Disables tuning for that axis.
    .gtune_hilimP[FD_PITCH] = 100,        // [0..200] Higher limit of PITCH P during G tune. 0 Disables tuning for that axis.
    .gtune_hilimP[FD_YAW] = 100,          // [0..200] Higher limit of YAW P during G tune. 0 Disables tuning for that axis.
    .gtune_pwr = 0,                       // [0..10] Strength of adjustment
    .gtune_settle_time = 450,             // [200..1000] Settle time in ms
    .gtune_average_cycles = 16,           // [8..128] Number of looptime cycles used for gyro average calculation
);
#endif

// GPS
PG_REGISTER_WITH_RESET_FN(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);

void pgResetFn_gpsConfig(gpsConfig_t *instance)
{
    instance->autoConfig = GPS_AUTOCONFIG_ON;
}

// LEDSTRIP
PG_REGISTER_ARR_WITH_RESET_FN(ledConfig_t, LED_MAX_STRIP_LENGTH, ledConfigs, PG_LED_STRIP_CONFIG, 0);
PG_REGISTER_ARR_WITH_RESET_FN(hsvColor_t, LED_CONFIGURABLE_COLOR_COUNT, colors, PG_COLOR_CONFIG, 0);
PG_REGISTER_ARR_WITH_RESET_FN(modeColorIndexes_t, LED_MODE_COUNT, modeColors, PG_MODE_COLOR_CONFIG, 0);
PG_REGISTER_ARR_WITH_RESET_FN(specialColorIndexes_t, 1, specialColors, PG_SPECIAL_COLOR_CONFIG, 0);

// macro for initializer
#define LF(name) LED_FLAG_FUNCTION(LED_FUNCTION_ ## name)
#define LD(name) LED_FLAG_DIRECTION(LED_DIRECTION_ ## name)

#ifdef USE_LED_RING_DEFAULT_CONFIG
static const ledConfig_t defaultLedStripConfig[] = {
    { CALCULATE_LED_XY( 2,  2), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 2,  1), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 2,  0), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 1,  0), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 0,  0), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 0,  1), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 0,  2), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 1,  2), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 1,  1), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 1,  1), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 1,  1), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 1,  1), 3, LF(THRUST_RING)},
};
#else
static const ledConfig_t defaultLedStripConfig[] = {
    { CALCULATE_LED_XY(15, 15), 0, LD(SOUTH) | LD(EAST) | LF(INDICATOR) | LF(ARM_STATE) },

    { CALCULATE_LED_XY(15,  8), 0, LD(EAST)             | LF(FLIGHT_MODE) | LF(WARNING) },
    { CALCULATE_LED_XY(15,  7), 0, LD(EAST)             | LF(FLIGHT_MODE) | LF(WARNING) },

    { CALCULATE_LED_XY(15,  0), 0, LD(NORTH) | LD(EAST) | LF(INDICATOR) | LF(ARM_STATE) },

    { CALCULATE_LED_XY( 8,  0), 0, LD(NORTH)            | LF(FLIGHT_MODE) },
    { CALCULATE_LED_XY( 7,  0), 0, LD(NORTH)            | LF(FLIGHT_MODE) },

    { CALCULATE_LED_XY( 0,  0), 0, LD(NORTH) | LD(WEST) | LF(INDICATOR) | LF(ARM_STATE) },

    { CALCULATE_LED_XY( 0,  7), 0, LD(WEST)             | LF(FLIGHT_MODE) | LF(WARNING) },
    { CALCULATE_LED_XY( 0,  8), 0, LD(WEST)             | LF(FLIGHT_MODE) | LF(WARNING) },

    { CALCULATE_LED_XY( 0, 15), 0, LD(SOUTH) | LD(WEST) | LF(INDICATOR) | LF(ARM_STATE) },

    { CALCULATE_LED_XY( 7, 15), 0, LD(SOUTH)            | LF(FLIGHT_MODE) | LF(WARNING) },
    { CALCULATE_LED_XY( 8, 15), 0, LD(SOUTH)            | LF(FLIGHT_MODE) | LF(WARNING) },

    { CALCULATE_LED_XY( 7,  7), 0, LD(UP)               | LF(FLIGHT_MODE) | LF(WARNING) },
    { CALCULATE_LED_XY( 8,  7), 0, LD(UP)               | LF(FLIGHT_MODE) | LF(WARNING) },
    { CALCULATE_LED_XY( 7,  8), 0, LD(DOWN)             | LF(FLIGHT_MODE) | LF(WARNING) },
    { CALCULATE_LED_XY( 8,  8), 0, LD(DOWN)             | LF(FLIGHT_MODE) | LF(WARNING) },

    { CALCULATE_LED_XY( 8,  9), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 9, 10), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY(10, 11), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY(10, 12), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 9, 13), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 8, 14), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 7, 14), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 6, 13), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 5, 12), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 5, 11), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 6, 10), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 7,  9), 3, LF(THRUST_RING)},

};
#endif
#undef LD
#undef LF

void pgResetFn_ledConfigs(ledConfig_t *instance)
{
    memcpy(instance, &defaultLedStripConfig, sizeof(defaultLedStripConfig));
}

void pgResetFn_colors(hsvColor_t *instance)
{
	const hsvColor_t hsv[] = {
		//                        H    S    V
		[COLOR_BLACK] =        {  0,   0,   0},
		[COLOR_WHITE] =        {  0, 255, 255},
		[COLOR_RED] =          {  0,   0, 255},
		[COLOR_ORANGE] =       { 30,   0, 255},
		[COLOR_YELLOW] =       { 60,   0, 255},
		[COLOR_LIME_GREEN] =   { 90,   0, 255},
		[COLOR_GREEN] =        {120,   0, 255},
		[COLOR_MINT_GREEN] =   {150,   0, 255},
		[COLOR_CYAN] =         {180,   0, 255},
		[COLOR_LIGHT_BLUE] =   {210,   0, 255},
		[COLOR_BLUE] =         {240,   0, 255},
		[COLOR_DARK_VIOLET] =  {270,   0, 255},
		[COLOR_MAGENTA] =      {300,   0, 255},
		[COLOR_DEEP_PINK] =    {330,   0, 255},
	};

    // copy hsv colors as default
    BUILD_BUG_ON(ARRAYLEN(*colors_arr()) <= ARRAYLEN(hsv));

    for (unsigned colorIndex = 0; colorIndex < ARRAYLEN(hsv); colorIndex++) {
        *instance++ = hsv[colorIndex];
    }
}

void pgResetFn_modeColors(modeColorIndexes_t *instance){
	static const modeColorIndexes_t defaultModeColors[] = {
		//                          NORTH             EAST               SOUTH            WEST             UP          DOWN
		[LED_MODE_ORIENTATION] = {{ COLOR_WHITE,      COLOR_DARK_VIOLET, COLOR_RED,       COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE }},
		[LED_MODE_HEADFREE]    = {{ COLOR_LIME_GREEN, COLOR_DARK_VIOLET, COLOR_ORANGE,    COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE }},
		[LED_MODE_HORIZON]     = {{ COLOR_BLUE,       COLOR_DARK_VIOLET, COLOR_YELLOW,    COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE }},
		[LED_MODE_ANGLE]       = {{ COLOR_CYAN,       COLOR_DARK_VIOLET, COLOR_YELLOW,    COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE }},
		[LED_MODE_MAG]         = {{ COLOR_MINT_GREEN, COLOR_DARK_VIOLET, COLOR_ORANGE,    COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE }},
		[LED_MODE_BARO]        = {{ COLOR_LIGHT_BLUE, COLOR_DARK_VIOLET, COLOR_RED,       COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE }},
	};

    memcpy(instance, &defaultModeColors, sizeof(defaultModeColors));
}

void pgResetFn_specialColors(specialColorIndexes_t *instance){
	static const specialColorIndexes_t defaultSpecialColors[] = {
		{{ [LED_SCOLOR_DISARMED]        = COLOR_GREEN,
		   [LED_SCOLOR_ARMED]           = COLOR_BLUE,
		   [LED_SCOLOR_ANIMATION]       = COLOR_WHITE,
		   [LED_SCOLOR_BACKGROUND]      = COLOR_BLACK,
		   [LED_SCOLOR_BLINKBACKGROUND] = COLOR_BLACK,
		   [LED_SCOLOR_GPSNOSATS]       = COLOR_RED,
		   [LED_SCOLOR_GPSNOLOCK]       = COLOR_ORANGE,
		   [LED_SCOLOR_GPSLOCKED]       = COLOR_GREEN,
		}}
	};

    memcpy(instance, &defaultSpecialColors, sizeof(defaultSpecialColors));
}


// TRANSPONDER
PG_REGISTER_WITH_RESET_TEMPLATE(transponderConfig_t, transponderConfig, PG_TRANSPONDER_CONFIG, 0);
PG_RESET_TEMPLATE(transponderConfig_t, transponderConfig,
    .data =  { 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC }, // Note, this is NOT a valid transponder code, it's just for testing production hardware
);

// TELEMETRY

#ifdef STM32F303xC
// hardware supports serial port inversion, make users life easier for those that want to connect SBus RX's
#define DEFAULT_TELEMETRY_INVERSION 1
#else
#define DEFAULT_TELEMETRY_INVERSION 0
#endif

PG_REGISTER(frskyTelemetryConfig_t, frskyTelemetryConfig, PG_FRSKY_TELEMETRY_CONFIG, 0);
PG_REGISTER_WITH_RESET_TEMPLATE(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 0);
PG_RESET_TEMPLATE(telemetryConfig_t, telemetryConfig,
    .telemetry_inversion = DEFAULT_TELEMETRY_INVERSION,
);


// HOTT
PG_REGISTER_WITH_RESET_TEMPLATE(hottTelemetryConfig_t, hottTelemetryConfig, PG_HOTT_TELEMETRY_CONFIG, 0);

PG_RESET_TEMPLATE(hottTelemetryConfig_t, hottTelemetryConfig,
    .hottAlarmSoundInterval = 5,
);


// GIMBAL
PG_REGISTER_PROFILE(gimbalConfig_t, gimbalConfig, PG_GIMBAL_CONFIG, 0);

// RX
PG_REGISTER_WITH_RESET_TEMPLATE(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);

PG_REGISTER_ARR_WITH_RESET_FN(rxFailsafeChannelConfig_t, MAX_SUPPORTED_RC_CHANNEL_COUNT, failsafeChannelConfigs, PG_FAILSAFE_CHANNEL_CONFIG, 0);
PG_REGISTER_ARR_WITH_RESET_FN(rxChannelRangeConfiguration_t, NON_AUX_CHANNEL_COUNT, channelRanges, PG_CHANNEL_RANGE_CONFIG, 0);

PG_RESET_TEMPLATE(rxConfig_t, rxConfig,
    .sbus_inversion = 1,
    .midrc = 1500,
    .mincheck = 1100,
    .maxcheck = 1900,
    .rx_min_usec = 885,          // any of first 4 channels below this value will trigger rx loss detection
    .rx_max_usec = 2115,         // any of first 4 channels above this value will trigger rx loss detection
    .rssi_scale = RSSI_SCALE_DEFAULT,
);

PG_REGISTER_PROFILE(adjustmentProfile_t, adjustmentProfile, PG_ADJUSTMENT_PROFILE, 0);

void pgResetFn_channelRanges(rxChannelRangeConfiguration_t *instance)
{
    // set default calibration to full range and 1:1 mapping
    for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        instance[i].min = PWM_RANGE_MIN;
        instance[i].max = PWM_RANGE_MAX;
    }
}

void pgResetFn_failsafeChannelConfigs(rxFailsafeChannelConfig_t *instance)
{
    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        instance[i].mode = (i < NON_AUX_CHANNEL_COUNT) ? RX_FAILSAFE_MODE_AUTO : RX_FAILSAFE_MODE_HOLD;
        instance[i].step = (i == THROTTLE)
            ? CHANNEL_VALUE_TO_RXFAIL_STEP(rxConfig()->rx_min_usec)
            : CHANNEL_VALUE_TO_RXFAIL_STEP(rxConfig()->midrc);
    }
}


// RC CONTROLS
PG_REGISTER_WITH_RESET_TEMPLATE(armingConfig_t, armingConfig, PG_ARMING_CONFIG, 0);

PG_REGISTER_PROFILE_WITH_RESET_TEMPLATE(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);
PG_REGISTER_PROFILE(modeActivationProfile_t, modeActivationProfile, PG_MODE_ACTIVATION_PROFILE, 0);

PG_RESET_TEMPLATE(rcControlsConfig_t, rcControlsConfig,
    .deadband = 0,
    .yaw_deadband = 0,
    .alt_hold_deadband = 40,
    .alt_hold_fast_change = 1,
    .yaw_control_direction = 1,
    .deadband3d_throttle = 50,
);

PG_RESET_TEMPLATE(armingConfig_t, armingConfig,
    .disarm_kill_switch = 1,
    .auto_disarm_delay = 5,
    .max_arm_angle = 25,
);


// BARO
#ifdef BARO
PG_REGISTER_PROFILE_WITH_RESET_TEMPLATE(barometerConfig_t, barometerConfig, PG_BAROMETER_CONFIG, 0);
PG_RESET_TEMPLATE(barometerConfig_t, barometerConfig,
    .baro_sample_count = 21,
    .baro_noise_lpf = 0.6f,
    .baro_cf_vel = 0.985f,
    .baro_cf_alt = 0.965f,
);
#endif

// COMPASS
PG_REGISTER_PROFILE_WITH_RESET_TEMPLATE(compassConfig_t, compassConfig, PG_COMPASS_CONFIGURATION, 0);

PG_RESET_TEMPLATE(compassConfig_t, compassConfig,
    .mag_declination = 0,
);


// GYRO
PG_REGISTER_WITH_RESET_TEMPLATE(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 0);

PG_RESET_TEMPLATE(gyroConfig_t, gyroConfig,
    .gyro_lpf = 1,                 // supported by all gyro drivers now. In case of ST gyro, will default to 32Hz instead
    .soft_gyro_lpf_hz = 60,        // Software based lpf filter for gyro

    .gyroMovementCalibrationThreshold = 32,
);

// IMU
PG_REGISTER_WITH_RESET_TEMPLATE(struct imu_config, imuConfig, PG_IMU_CONFIG, 0);
PG_REGISTER_PROFILE_WITH_RESET_TEMPLATE(struct throttle_correction_config, throttleCorrectionConfig, PG_THROTTLE_CORRECTION_CONFIG, 0);

PG_RESET_TEMPLATE(struct imu_config, imuConfig,
    .dcm_kp = 2500,                // 1.0 * 10000
    .looptime = 2000,
    .gyroSync = 1,
    .gyroSyncDenominator = 1,
    .small_angle = 25,
    .max_angle_inclination = 500,    // 50 degrees
);

PG_RESET_TEMPLATE(struct throttle_correction_config, throttleCorrectionConfig,
    .throttle_correction_value = 0,      // could 10 with althold or 40 for fpv
    .throttle_correction_angle = 800,    // could be 80.0 deg with atlhold or 45.0 for fpv
);

// MIXER
#ifdef USE_SERVOS
PG_RESET_TEMPLATE(struct mixer_config, mixerConfig,
    .mixerMode = MIXER_QUADX,
    .pid_at_min_throttle = 1,
    .yaw_motor_direction = 1,
    .yaw_jump_prevention_limit = 200,

    .tri_unarmed_servo = 1,
    .servo_lowpass_freq = 400.0f,
);
#else
PG_RESET_TEMPLATE(struct mixer_config, mixerConfig,
    .mixerMode = MIXER_QUADX,
    .pid_at_min_throttle = 1,
    .yaw_motor_direction = 1,
    .yaw_jump_prevention_limit = 200,
);
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(struct mixer_tilt_config, mixerTiltConfig, PG_MIXER_TILT_CONFIG, 0); 
PG_RESET_TEMPLATE(struct mixer_tilt_config, mixerTiltConfig,
	.mode = MIXER_TILT_MODE_DYNAMIC, 
	.control_channel = AUX1,
	.compensation_flags = MIXER_TILT_COMPENSATE_THRUST | MIXER_TILT_COMPENSATE_TILT | MIXER_TILT_COMPENSATE_BODY, 
	.servo_angle_min = -45, 
	.servo_angle_max = 45
); 

// FAILSAFE
PG_REGISTER_WITH_RESET_TEMPLATE(failsafeConfig_t, failsafeConfig, PG_FAILSAFE_CONFIG, 0);

PG_RESET_TEMPLATE(failsafeConfig_t, failsafeConfig,
    .failsafe_delay = 10,              // 1sec
    .failsafe_off_delay = 200,         // 20sec
    .failsafe_throttle = 1000,         // default throttle off.
    .failsafe_throttle_low_delay = 100, // default throttle low delay for "just disarm" on failsafe condition
);

// ALTITUDE
PG_REGISTER_WITH_RESET_TEMPLATE(airplaneConfig_t, airplaneConfig, PG_AIRPLANE_ALT_HOLD_CONFIG, 0);

PG_RESET_TEMPLATE(airplaneConfig_t, airplaneConfig,
    .fixedwing_althold_dir = 1,
);


// SYSTEM
PG_REGISTER_WITH_RESET_TEMPLATE(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 0);
PG_REGISTER(pwmRxConfig_t, pwmRxConfig, PG_DRIVER_PWM_RX_CONFIG, 0);

PG_RESET_TEMPLATE(systemConfig_t, systemConfig,
    .i2c_highspeed = 1,
);

// SERIAL
PG_REGISTER_WITH_RESET_FN(serialConfig_t, serialConfig, PG_SERIAL_CONFIG, 0);

const serialPortIdentifier_e serialPortIdentifiers[SERIAL_PORT_COUNT] = {
#ifdef USE_VCP
    SERIAL_PORT_USB_VCP,
#endif
#ifdef USE_UART1
    SERIAL_PORT_UART1,
#endif
#ifdef USE_UART2
    SERIAL_PORT_UART2,
#endif
#ifdef USE_UART3
    SERIAL_PORT_UART3,
#endif
#ifdef USE_UART4
    SERIAL_PORT_UART4,
#endif
#ifdef USE_UART5
    SERIAL_PORT_UART5,
#endif
#ifdef USE_SOFTSERIAL1
    SERIAL_PORT_SOFTSERIAL1,
#endif
#ifdef USE_SOFTSERIAL2
    SERIAL_PORT_SOFTSERIAL2,
#endif
};


void pgResetFn_serialConfig(serialConfig_t *serialConfig)
{
    memset(serialConfig, 0, sizeof(serialConfig_t));

    serialPortConfig_t portConfig_Reset = {
        .msp_baudrateIndex = BAUD_115200,
        .gps_baudrateIndex = BAUD_57600,
        .telemetry_baudrateIndex = BAUD_AUTO,
        .blackbox_baudrateIndex = BAUD_115200,
    };

    for (int i = 0; i < SERIAL_PORT_COUNT; i++) {
        memcpy(&serialConfig->portConfigs[i], &portConfig_Reset, sizeof(serialConfig->portConfigs[i]));
        serialConfig->portConfigs[i].identifier = serialPortIdentifiers[i];
    }

    serialConfig->portConfigs[0].functionMask = FUNCTION_MSP;

#if defined(USE_VCP)
    // This allows MSP connection via USART & VCP so the board can be reconfigured.
    serialConfig->portConfigs[1].functionMask = FUNCTION_MSP;
#endif

    serialConfig->reboot_character = 'R';
}



static void configureRateProfileSelection(uint8_t profileIndex, uint8_t rateProfileIndex){
    rateProfileSelection_Storage[profileIndex].defaultRateProfileIndex = rateProfileIndex % MAX_CONTROL_RATE_PROFILE_COUNT;
}
// Default settings
STATIC_UNIT_TESTED void resetConf(void)
{
    pgResetAll(MAX_PROFILE_COUNT);

    setProfile(0);
    pgActivateProfile(0);

    setControlRateProfile(0);

    parseRcChannels("AETR1234", rxConfig());

    featureClearAll();

    featureSet(DEFAULT_RX_FEATURE | FEATURE_FAILSAFE | FEATURE_BLACKBOX);
#ifdef DEFAULT_FEATURES
    featureSet(DEFAULT_FEATURES);
#endif

#ifdef BOARD_HAS_VOLTAGE_DIVIDER
    // only enable the VBAT feature by default if the board has a voltage divider otherwise
    // the user may see incorrect readings and unexpected issues with pin mappings may occur.
    featureSet(FEATURE_VBAT);
#endif

#if defined(COLIBRI_RACE)
    // alternative defaults settings for COLIBRI RACE targets
    imuConfig()->looptime = 1000;
#endif

    // alternative defaults settings for ALIENFLIGHTF1 and ALIENFLIGHTF3 targets
#ifdef ALIENFLIGHT
#ifdef ALIENFLIGHTF3
    serialConfig()->portConfigs[2].functionMask = FUNCTION_RX_SERIAL;
    batteryConfig()->vbatscale = 20;
    sensorSelectionConfig()->mag_hardware = MAG_NONE;            // disabled by default
# else
    serialConfig()->portConfigs[1].functionMask = FUNCTION_RX_SERIAL;
# endif
    rxConfig()->serialrx_provider = SERIALRX_SPEKTRUM2048;
    rxConfig()->spektrum_sat_bind = 5;
    motorAndServoConfig()->minthrottle = 1000;
    motorAndServoConfig()->maxthrottle = 2000;
    motorAndServoConfig()->motor_pwm_rate = 32000;
    imuConfig()->looptime = 2000;
    pidProfile()->pidController = PID_CONTROLLER_LUX_FLOAT;
    failsafeConfig()->failsafe_delay = 2;
    failsafeConfig()->failsafe_off_delay = 0;
    mixerConfig()->yaw_jump_prevention_limit = YAW_JUMP_PREVENTION_LIMIT_HIGH;
    currentControlRateProfile->rcRate8 = 100;
    currentControlRateProfile->rates[PITCH] = 20;
    currentControlRateProfile->rates[ROLL] = 20;
    currentControlRateProfile->rates[YAW] = 20;
    parseRcChannels("TAER1234", rxConfig());

#if 0
	// TODO: finish porting this
	tiltConfig->flagEnabled = TILT_ARM_ENABLE_PITCH_DIVIDER;
	tiltConfig->pitchDivisior = 30;
	tiltConfig->thrustLiftoffPercent = 0;
	tiltConfig->gearRatioPercent = 100;
	tiltConfig->channel = AUX1;
#endif

    *customMotorMixer(0) = (struct motor_mixer){ 1.0f, -0.414178f,  1.0f, -1.0f };    // REAR_R
    *customMotorMixer(1) = (struct motor_mixer){ 1.0f, -0.414178f, -1.0f,  1.0f };    // FRONT_R
    *customMotorMixer(2) = (struct motor_mixer){ 1.0f,  0.414178f,  1.0f,  1.0f };    // REAR_L
    *customMotorMixer(3) = (struct motor_mixer){ 1.0f,  0.414178f, -1.0f, -1.0f };    // FRONT_L
    *customMotorMixer(4) = (struct motor_mixer){ 1.0f, -1.0f, -0.414178f, -1.0f };    // MIDFRONT_R
    *customMotorMixer(5) = (struct motor_mixer){ 1.0f,  1.0f, -0.414178f,  1.0f };    // MIDFRONT_L
    *customMotorMixer(6) = (struct motor_mixer){ 1.0f, -1.0f,  0.414178f,  1.0f };    // MIDREAR_R
    *customMotorMixer(7) = (struct motor_mixer){ 1.0f,  1.0f,  0.414178f, -1.0f };    // MIDREAR_L
#endif

    // copy first profile into remaining profile
    PG_FOREACH_PROFILE(reg) {
        for (int i = 1; i < MAX_PROFILE_COUNT; i++) {
            memcpy(reg->address + i * pgSize(reg), reg->address, pgSize(reg));
        }
    }
    for (int i = 1; i < MAX_PROFILE_COUNT; i++) {
        configureRateProfileSelection(i, i % MAX_CONTROL_RATE_PROFILE_COUNT);
    }
}

// TODO: remove
extern struct imu_runtime_config imuRuntimeConfig;
static void activateConfig(void)
{
    activateControlRateConfig();

    resetAdjustmentStates();

    useRcControlsConfig(modeActivationProfile()->modeActivationConditions);

    anglerate_controller_set_algo(&default_controller, pidProfile()->pidController);

#ifdef GPS
    gpsUsePIDs(pidProfile());
#endif

    useFailsafeConfig();
    setAccelerationTrims(&sensorTrims()->accZero);

#ifdef USE_SERVOS
    mixerUseConfigs(servoProfile()->servoConf);
#endif

    recalculateMagneticDeclination();

	// TODO: this is called on boot for an imu structure that has not been initialized yet. We initialize it later. 
	imu_configure(
		&default_imu, 
		imuConfig(),
		accelerometerConfig(),
		throttleCorrectionConfig(),
		gyro.scale, 
		acc.acc_1G
	); 
}

static void validateAndFixConfig(void)
{
    if (!(featureConfigured(FEATURE_RX_PARALLEL_PWM) || featureConfigured(FEATURE_RX_PPM) || featureConfigured(FEATURE_RX_SERIAL) || featureConfigured(FEATURE_RX_MSP))) {
        featureSet(DEFAULT_RX_FEATURE);
    }

    if (featureConfigured(FEATURE_RX_PPM)) {
        featureClear(FEATURE_RX_PARALLEL_PWM | FEATURE_RX_SERIAL | FEATURE_RX_MSP);
    }

    if (featureConfigured(FEATURE_RX_MSP)) {
        featureClear(FEATURE_RX_SERIAL | FEATURE_RX_PARALLEL_PWM | FEATURE_RX_PPM);
    }

    if (featureConfigured(FEATURE_RX_SERIAL)) {
        featureClear(FEATURE_RX_PARALLEL_PWM | FEATURE_RX_MSP | FEATURE_RX_PPM);
    }

    if (featureConfigured(FEATURE_RX_PARALLEL_PWM)) {
        featureClear(FEATURE_RX_SERIAL | FEATURE_RX_MSP | FEATURE_RX_PPM);
    }

    // The retarded_arm setting is incompatible with pid_at_min_throttle because full roll causes the craft to roll over on the ground.
    // The pid_at_min_throttle implementation ignores yaw on the ground, but doesn't currently ignore roll when retarded_arm is enabled.
    if (armingConfig()->retarded_arm && mixerConfig()->pid_at_min_throttle) {
        mixerConfig()->pid_at_min_throttle = 0;
    }


#ifdef STM32F10X
    // avoid overloading the CPU on F1 targets when using gyro sync and GPS.
    if (imuConfig()->gyroSync && imuConfig()->gyroSyncDenominator < 2 && featureConfigured(FEATURE_GPS)) {
        imuConfig()->gyroSyncDenominator = 2;
    }
#endif


#if defined(LED_STRIP)
#if (defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2))
    if (featureConfigured(FEATURE_SOFTSERIAL) && (
            0
#ifdef USE_SOFTSERIAL1
            || (LED_STRIP_TIMER == SOFTSERIAL_1_TIMER)
#endif
#ifdef USE_SOFTSERIAL2
            || (LED_STRIP_TIMER == SOFTSERIAL_2_TIMER)
#endif
    )) {
        // led strip needs the same timer as softserial
        featureClear(FEATURE_LED_STRIP);
    }
#endif

#if defined(TRANSPONDER) && !defined(UNIT_TEST)
    if ((WS2811_DMA_TC_FLAG == TRANSPONDER_DMA_TC_FLAG) && featureConfigured(FEATURE_TRANSPONDER) && featureConfigured(FEATURE_LED_STRIP)) {
        featureClear(FEATURE_LED_STRIP);
    }
#endif
#endif // LED_STRIP

#if defined(CC3D)
#if defined(DISPLAY) && defined(USE_UART3)
    if (featureConfigured(FEATURE_DISPLAY) && doesConfigurationUsePort(SERIAL_PORT_UART3)) {
        featureClear(FEATURE_DISPLAY);
    }
#endif

#if defined(SONAR) && defined(USE_SOFTSERIAL1)
    if (featureConfigured(FEATURE_SONAR) && featureConfigured(FEATURE_SOFTSERIAL)) {
        featureClear(FEATURE_SONAR);
    }
#endif

#if defined(SONAR) && defined(USE_SOFTSERIAL1) && defined(RSSI_ADC_GPIO)
    // shared pin
    if ((featureConfigured(FEATURE_SONAR) + featureConfigured(FEATURE_SOFTSERIAL) + featureConfigured(FEATURE_RSSI_ADC)) > 1) {
        featureClear(FEATURE_SONAR);
        featureClear(FEATURE_SOFTSERIAL);
        featureClear(FEATURE_RSSI_ADC);
    }
#endif
#endif // CC3D

#if defined(COLIBRI_RACE)
    serialConfig()->portConfigs[0].functionMask = FUNCTION_MSP;
    if (featureConfigured(FEATURE_RX_SERIAL)) {
        serialConfig()->portConfigs[2].functionMask = FUNCTION_RX_SERIAL;
    }
#endif

    if (!isSerialConfigValid(serialConfig())) {
        PG_RESET_CURRENT(serialConfig);
    }

#if defined(USE_VCP)
    serialConfig()->portConfigs[0].functionMask = FUNCTION_MSP;
#endif
}

void readEEPROM(void)
{
    suspendRxSignal();

    // Sanity check, read flash
    if (!scanEEPROM(true)) {
        failureMode(FAILURE_INVALID_EEPROM_CONTENTS);
    }

    pgActivateProfile(getCurrentProfile());

    setControlRateProfile(rateProfileSelection()->defaultRateProfileIndex);

    validateAndFixConfig();
    activateConfig();

    resumeRxSignal();
}

void writeEEPROM(void)
{
    suspendRxSignal();

    writeConfigToEEPROM();

    resumeRxSignal();
}

void ensureEEPROMContainsValidData(void)
{
    if (isEEPROMContentValid()) {
        return;
    }
    resetEEPROM();
}

void resetEEPROM(void)
{
    resetConf();
    writeEEPROM();
}

void saveConfigAndNotify(void)
{
    writeEEPROM();
    readEEPROM();
    beeperConfirmationBeeps(1);
}

void changeProfile(uint8_t profileIndex)
{
    setProfile(profileIndex);
    writeEEPROM();
    readEEPROM();
}

void handleOneshotFeatureChangeOnRestart(void)
{
    // Shutdown PWM on all motors prior to soft restart
	// TODO: this is not supposed to be called from here and with new changes this is very apparent. So fix this after done refactoring. 
    //StopPwmAllMotors(&default_mixer);
    delay(50);
    // Apply additional delay when OneShot125 feature changed from on to off state
    if (feature(FEATURE_ONESHOT125) && !featureConfigured(FEATURE_ONESHOT125)) {
        delay(ONESHOT_FEATURE_CHANGED_DELAY_ON_BOOT_MS);
    }
}
