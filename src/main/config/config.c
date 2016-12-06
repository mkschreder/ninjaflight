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
#include <errno.h>

#include <platform.h>

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config.h"
#include "config_eeprom.h"
#include "feature.h"
#include "profile.h"
#include "system_calls.h"

#ifdef ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_FLASH
#elif defined(ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT)
#define DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_SDCARD
#else
#define DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_SERIAL
#endif

#define BRUSHED_MOTORS_PWM_RATE 16000
#define BRUSHLESS_MOTORS_PWM_RATE 400

#ifdef BRUSHED_MOTORS
#define DEFAULT_PWM_RATE BRUSHED_MOTORS_PWM_RATE
#else
#define DEFAULT_PWM_RATE BRUSHLESS_MOTORS_PWM_RATE
#endif

#ifdef STM32F303xC
// hardware supports serial port inversion, make users life easier for those that want to connect SBus RX's
#define DEFAULT_TELEMETRY_INVERSION 1
#else
#define DEFAULT_TELEMETRY_INVERSION 0
#endif

static void _reset_rate_profile(struct rate_profile *self){
	*self = (struct rate_profile) {
		.rcRate8 = 90,
		.rcExpo8 = 65,
		.thrMid8 = 50,
		.tpa_breakpoint = 1500,
	};
}

static void _reset_servo_profile(struct servo_config *self){
	*self = (struct servo_config){
		.min = DEFAULT_SERVO_MIN,
		.max = DEFAULT_SERVO_MAX,
		.middle = DEFAULT_SERVO_MIDDLE,
		.rate = 100,
		.angleAtMin = DEFAULT_SERVO_MIN_ANGLE,
		.angleAtMax = DEFAULT_SERVO_MAX_ANGLE,
		.forwardFromChannel = CHANNEL_FORWARDING_DISABLED,
	};
}

static void _reset_led_colors(struct ledstrip_config *self){
	static const hsvColor_t hsv[] = {
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

    for (unsigned colorIndex = 0; colorIndex < ARRAYLEN(hsv); colorIndex++) {
        self->colors[colorIndex] = hsv[colorIndex];
    }
}

static void _reset_led_mode_colors(struct ledstrip_config *self){
	static const struct led_mode_color_indices defaultModeColors[] = {
		//                          NORTH             EAST               SOUTH            WEST             UP          DOWN
		[LED_MODE_ORIENTATION] = {{ COLOR_WHITE,      COLOR_DARK_VIOLET, COLOR_RED,       COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE }},
		[LED_MODE_HEADFREE]    = {{ COLOR_LIME_GREEN, COLOR_DARK_VIOLET, COLOR_ORANGE,    COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE }},
		[LED_MODE_HORIZON]     = {{ COLOR_BLUE,       COLOR_DARK_VIOLET, COLOR_YELLOW,    COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE }},
		[LED_MODE_ANGLE]       = {{ COLOR_CYAN,       COLOR_DARK_VIOLET, COLOR_YELLOW,    COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE }},
		[LED_MODE_MAG]         = {{ COLOR_MINT_GREEN, COLOR_DARK_VIOLET, COLOR_ORANGE,    COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE }},
		[LED_MODE_BARO]        = {{ COLOR_LIGHT_BLUE, COLOR_DARK_VIOLET, COLOR_RED,       COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE }},
	};

    memcpy(self->modeColors, &defaultModeColors, sizeof(defaultModeColors));
}

static void _reset_led_spc_colors(struct ledstrip_config *self){
	static const struct led_spc_color_indices defaultSpecialColors[] = {
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

    memcpy(self->spcColors, &defaultSpecialColors, sizeof(defaultSpecialColors));
}

static void _reset_ledstrip_config(struct ledstrip_config *self){
	// macro for initializer
	#define LF(name) LED_FLAG_FUNCTION(LED_FUNCTION_ ## name)
	#define LD(name) LED_FLAG_DIRECTION(LED_DIRECTION_ ## name)

	#ifdef USE_LED_RING_DEFAULT_CONFIG
	static const led_config defaultLedStripConfig[] = {
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
	static const struct led_config defaultLedStripConfig[] = {
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
	memcpy(self->leds, defaultLedStripConfig, sizeof(defaultLedStripConfig));
	_reset_led_colors(self);
	_reset_led_mode_colors(self);
	_reset_led_spc_colors(self);
}

static void _reset_rx_output_config(struct rx_output_config *self, const struct rx_config *rx){
    // set default calibration to full range and 1:1 mapping
    for (int i = 0; i < RX_NON_AUX_CHANNEL_COUNT; i++) {
        self->range[i].min = PWM_RANGE_MIN;
        self->range[i].max = PWM_RANGE_MAX;
    }
    for (int i = 0; i < RX_MAX_SUPPORTED_RC_CHANNELS; i++) {
        self->failsafe[i].mode = (i < RX_NON_AUX_CHANNEL_COUNT) ? RX_FAILSAFE_MODE_AUTO : RX_FAILSAFE_MODE_HOLD;
        self->failsafe[i].step = (i == THROTTLE)
            ? CHANNEL_VALUE_TO_RXFAIL_STEP(rx->rx_min_usec)
            : CHANNEL_VALUE_TO_RXFAIL_STEP(rx->midrc);
    }
}

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


static void _reset_serial_config(struct serial_config *self){
	    struct serial_port_config portConfig_Reset = {
        .msp_baudrateIndex = BAUD_115200,
        .gps_baudrateIndex = BAUD_57600,
        .telemetry_baudrateIndex = BAUD_AUTO,
        .blackbox_baudrateIndex = BAUD_115200,
    };

    for (int i = 0; i < SERIAL_PORT_COUNT; i++) {
        memcpy(&self->portConfigs[i], &portConfig_Reset, sizeof(self->portConfigs[i]));
        self->portConfigs[i].identifier = serialPortIdentifiers[i];
    }

    self->portConfigs[0].functionMask = FUNCTION_MSP;

#if defined(USE_VCP)
    // This allows MSP connection via USART & VCP so the board can be reconfigured.
    self->portConfigs[1].functionMask = FUNCTION_MSP;
#endif

    self->reboot_character = 'R';
}

static void _reset_profile(struct config_profile *self){
	self->pid = (struct pid_config){
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
	};
	self->acc = (struct accelerometer_config){
		.acc_cut_hz = 15,
		.accz_lpf_cutoff = 5.0f,
		.accDeadband.z = 40,
		.accDeadband.xy = 40,
		.acc_unarmedcal = 1,
		.trims = (rollAndPitchTrims_t){
			.values.roll = 0,
			.values.pitch = 0
		}
    };
	for(int c = 0; c < MAX_SUPPORTED_SERVOS; c++){
		_reset_servo_profile(&self->servos.servoConf[c]);
	}
	self->gps = (struct gps_profile){
		.gps_wp_radius = 200,
		.gps_lpf = 20,
		.nav_slew_rate = 30,
		.nav_controls_heading = 1,
		.nav_speed_min = 100,
		.nav_speed_max = 300,
		.ap_mode = 40,
	};
	self->gtune = (struct gtune_config){
		.gtune_lolimP[FD_ROLL] = 10,          // [0..200] Lower limit of ROLL P during G tune.
		.gtune_lolimP[FD_PITCH] = 10,         // [0..200] Lower limit of PITCH P during G tune.
		.gtune_lolimP[FD_YAW] = 10,           // [0..200] Lower limit of YAW P during G tune.
		.gtune_hilimP[FD_ROLL] = 100,         // [0..200] Higher limit of ROLL P during G tune. 0 Disables tuning for that axis.
		.gtune_hilimP[FD_PITCH] = 100,        // [0..200] Higher limit of PITCH P during G tune. 0 Disables tuning for that axis.
		.gtune_hilimP[FD_YAW] = 100,          // [0..200] Higher limit of YAW P during G tune. 0 Disables tuning for that axis.
		.gtune_pwr = 0,                       // [0..10] Strength of adjustment
		.gtune_settle_time = 450,             // [200..1000] Settle time in ms
		.gtune_average_cycles = 16,           // [8..128] Number of looptime cycles used for gyro average calculation
	};
	self->rc = (struct rc_controls_config){
		.deadband = 0,
		.yaw_deadband = 0,
		.alt_hold_deadband = 40,
		.alt_hold_fast_change = 1,
		.yaw_control_direction = 1,
		.deadband3d_throttle = 50,
	};
	self->baro = (struct barometer_config){
		.baro_sample_count = 21,
		.baro_noise_lpf = 0.6f,
		.baro_cf_vel = 0.985f,
		.baro_cf_alt = 0.965f,
	};
	self->mag = (struct mag_config){
		.mag_declination = 0,
	};
	self->throttle = (struct throttle_correction_config){
		.throttle_correction_value = 0,      // could 10 with althold or 40 for fpv
		.throttle_correction_angle = 800,    // could be 80.0 deg with atlhold or 45.0 for fpv
	};
}

void config_reset(struct config *self){
	self->blackbox = (struct blackbox_config){
		.device = DEFAULT_BLACKBOX_DEVICE,
		.rate_num = 1,
		.rate_denom = 1
	};
	self->gyro = (struct gyro_config){
		.gyro_lpf = 1,                 // supported by all gyro drivers now. In case of ST gyro, will default to 32Hz instead
		.soft_gyro_lpf_hz = 60,        // Software based lpf filter for gyro
		.move_threshold = 32,
	};

	self->bat = (struct battery_config){
		.vbatscale = VBAT_SCALE_DEFAULT,
		.vbatresdivval = VBAT_RESDIVVAL_DEFAULT,
		.vbatresdivmultiplier = VBAT_RESDIVMULTIPLIER_DEFAULT,
		.vbatmaxcellvoltage = 43,
		.vbatmincellvoltage = 33,
		.vbatwarningcellvoltage = 35,
		.currentMeterScale = 400, // for Allegro ACS758LCB-100U (40mV/A)
		.currentMeterType = CURRENT_SENSOR_ADC,
	};
	self->motor_3d = (struct motor_3d_config){
		.deadband3d_low = 1406,
		.deadband3d_high = 1514,
		.neutral3d = 1460,
	};
	for(int c = 0; c < MAX_CONTROL_RATE_PROFILE_COUNT; c++){
		_reset_rate_profile(&self->rate.profile[c]);
	}
	for(int c = 0; c < MAX_PROFILE_COUNT; c++){
		_reset_profile(&self->profiles[c]);
	}
	self->pwm_out = (struct pwm_output_config){
		.minthrottle = 1150,
		.maxthrottle = 1850,
		.mincommand = 1000,
		.servoCenterPulse = 1500,
		.motor_pwm_rate = DEFAULT_PWM_RATE,
		.servo_pwm_rate = 50,
	};
	self->gps = (struct gps_config){
		.autoConfig = GPS_AUTOCONFIG_ON
	};
	_reset_ledstrip_config(&self->ledstrip);
	self->transponder = (struct transponder_config){
		.data =  { 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC }, // Note, this is NOT a valid transponder code, it's just for testing production hardware
	};
	self->telemetry = (struct telemetry_config){
		.telemetry_inversion = DEFAULT_TELEMETRY_INVERSION,
	};
	self->hott = (struct hott_telemetry_config){
		.hottAlarmSoundInterval = 5,
	};
	self->rx = (struct rx_config){
		.sbus_inversion = 1,		//!< specifies if sbus signal is inverted
		.midrc = 1500,				//!< middle point in usec of the rx signal (also zero point for all channels besides throttle)
		.mincheck = 1100,			//!< used for checking if a channel is at minimum
		.maxcheck = 1900,			//!< used to check if channel is at maximum
		.rx_min_usec = 885,          //!< any of first 4 channels below this value will trigger rx loss detection
		.rx_max_usec = 2115,         //!< any of first 4 channels above this value will trigger rx loss detection
		.rssi_scale = RSSI_SCALE_DEFAULT,
	};
	rx_config_set_mapping(&self->rx, "AERT1234");
	_reset_rx_output_config(&self->rx_output, &self->rx);
	self->arm = (struct arming_config){
		.disarm_kill_switch = 1,
		.auto_disarm_delay = 5,
		.max_arm_angle = 25,
	};
	self->imu = (struct imu_config){
		.dcm_kp = 2500,                // 1.0 * 10000
		.looptime = 2000,
		.gyroSync = 1,
		.gyroSyncDenominator = 1,
		.small_angle = 25,
		.max_angle_inclination = 500,    // 50 degrees
	};
	self->mixer = (struct mixer_config){
		.mixerMode = MIXER_QUADX,
		.pid_at_min_throttle = 1,
		.yaw_motor_direction = 1,
		.yaw_jump_prevention_limit = 200,

		.tri_unarmed_servo = 1,
		.servo_lowpass_freq = 400.0f,
	};
	self->tilt = (struct tilt_config){
		.mode = MIXER_TILT_MODE_DYNAMIC,
		.control_channel = AUX1,
		.compensation_flags = MIXER_TILT_COMPENSATE_THRUST | MIXER_TILT_COMPENSATE_TILT | MIXER_TILT_COMPENSATE_BODY,
		.servo_angle_min = -45,
		.servo_angle_max = 45
	};
	self->failsafe = (struct failsafe_config){
		.failsafe_delay = 10,              // 1sec
		.failsafe_off_delay = 200,         // 20sec
		.failsafe_throttle = 1000,         // default throttle off.
		.failsafe_throttle_low_delay = 100, // default throttle low delay for "just disarm" on failsafe condition
	};
	self->airplane_althold = (struct airplane_althold_config){
		.fixedwing_althold_dir = 1,
	};
	self->system = (struct system_config){
		.i2c_highspeed = 1
	};
	_reset_serial_config(&self->serial);
}

static int gcd(int num, int denom)
{
	if (denom == 0) {
		return num;
	}

	return gcd(denom, num % denom);
}

static void __attribute__((unused)) _validate_blackbox_config(struct config *self){
	int div;

	if (self->blackbox.rate_num == 0 || self->blackbox.rate_denom == 0
			|| self->blackbox.rate_num >= self->blackbox.rate_denom) {
		self->blackbox.rate_num = 1;
		self->blackbox.rate_denom = 1;
	} else {
		/* Reduce the fraction the user entered as much as possible (makes the recorded/skipped frame pattern repeat
		 * itself more frequently)
		 */
		div = gcd(self->blackbox.rate_num, self->blackbox.rate_denom);

		self->blackbox.rate_num /= div;
		self->blackbox.rate_denom /= div;
	}

	// If we've chosen an unsupported device, change the device to serial
	switch (self->blackbox.device) {
#ifdef USE_FLASHFS
		case BLACKBOX_DEVICE_FLASH:
#endif
#ifdef USE_SDCARD
		case BLACKBOX_DEVICE_SDCARD:
#endif
		case BLACKBOX_DEVICE_SERIAL:
			// Device supported, leave the setting alone
		break;

		default:
			self->blackbox.device = BLACKBOX_DEVICE_SERIAL;
	}
}

const struct config_profile const *config_get_profile(const struct config * const self){
	return &self->profiles[self->profile.profile_id];
}

struct config_profile *config_get_profile_rw(struct config *self){
	return &self->profiles[self->profile.profile_id];
}

struct rate_profile const * config_get_rate_profile(const struct config * const self){
	return &self->rate.profile[config_get_profile(self)->rate.profile_id];
}

struct rate_profile *config_get_rate_profile_rw(struct config *self){
	return &self->rate.profile[config_get_profile(self)->rate.profile_id];
}

void config_save(const struct config *self){
	(void)self;
}

static bool config_store_valid(const struct config_store *self){
	(void)self;
	return true;
}

int config_load(struct config *self, const struct system_calls *system){
	struct config_store store;
	if(sys_eeprom_read(system, &store, 0, sizeof(store)) != 0){
		return -EIO;
	}
	if(!config_store_valid(&store)){
		return -EINVAL;
	}
	memcpy(self, &store.data, sizeof(*self));
	return 0;
}

void handleOneshotFeatureChangeOnRestart(void)
{
    // Shutdown PWM on all motors prior to soft restart
	// TODO: this is not supposed to be called from here and with new changes this is very apparent. So fix this after done refactoring. 
    //StopPwmAllMotors(&default_mixer);
	/*
    usleep(50000);
    // Apply additional delay when OneShot125 feature changed from on to off state
    if (feature(FEATURE_ONESHOT125) && !featureConfigured(FEATURE_ONESHOT125)) {
        usleep(ONESHOT_FEATURE_CHANGED_DELAY_ON_BOOT_MS * 1000);
    }
	*/
}

/*
void configureRateProfileSelection(uint8_t profileIndex, uint8_t rateProfileIndex){
    rateProfileSelection_Storage[profileIndex].defaultRateProfileIndex = rateProfileIndex % MAX_CONTROL_RATE_PROFILE_COUNT;
}
*/


