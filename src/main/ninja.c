#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "system_calls.h"

#include "drivers/system.h"
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
#include "io/rc_curves.h"
#include "io/gps.h"
#include "io/msp.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_cli.h"
#include "io/serial_msp.h"
#include "io/statusindicator.h"
#include "io/asyncfatfs/asyncfatfs.h"
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

// TODO: this is not used so we need to remove it
int16_t rcCommand[4];

struct ninja_state {
	void (*on_event)(struct ninja *self, const struct ninja_state *state, uint8_t ev);
	const struct ninja_state *parent;
};

enum {
	NEV_ENTER,
	NEV_LEAVE,
	NEV_RC,
	NEV_GYRO,
	NEV_ACC
};

#define STATE_HANDLER(name) \
	static void _state_handler_##name (struct ninja __attribute__((unused))  *self, const struct ninja_state __attribute__((unused)) *current_state, uint8_t __attribute__((unused))  ev);\
	extern const struct ninja_state _state_ ##name;

#define DECLARE_STATE(name, parent_state) \
	static void _state_handler_##name (struct ninja __attribute__((unused))  *self, const struct ninja_state __attribute__((unused)) *current_state, uint8_t __attribute__((unused))  ev);\
	const struct ninja_state _state_ ##name = { \
		.on_event = _state_handler_ ##name ,\
		.parent = &_state_ ##parent_state \
	};\
	static void _state_handler_##name (struct ninja __attribute__((unused))  *self, const struct ninja_state __attribute__((unused)) *current_state, uint8_t __attribute__((unused))  ev)

#define PARENT_STATE_HANDLE_EVENT() if(current_state->parent && current_state->parent != current_state){ current_state->parent->on_event(self, current_state->parent, ev); }

#define STATE_TRANSITION(newstate) do { \
	if(self->state) self->state->on_event(self, self->state, NEV_LEAVE); \
	self->state = &_state_ ##newstate;\
	self->state->on_event(self, self->state, NEV_ENTER); \
} while(0);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wredundant-decls"

DECLARE_STATE(NULL, NULL){}

STATE_HANDLER(ST_IDLE2);

DECLARE_STATE(ST_IDLE, NULL){
	switch(ev){
		case NEV_RC: {
			if(self->rc_input.raw[0] < 1200){
				STATE_TRANSITION(ST_IDLE2);
			}
		} break;
	}
}

DECLARE_STATE(ST_IDLE2, NULL){
	switch(ev){
		case NEV_RC: {
			if(self->rc_input.raw[0] > 1200){
				STATE_TRANSITION(ST_IDLE);
			}
		} break;
	}
}

#pragma GCC diagnostic pop

static void ninja_validate_config(struct ninja *self){
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


void ninja_load_config(struct ninja *self){
    rx_suspend_signal(self->rx);

    // Sanity check, read flash
    if (!scanEEPROM(true)) {
        failureMode(FAILURE_INVALID_EEPROM_CONTENTS);
    }

    pgActivateProfile(getCurrentProfile());

	// TODO: need to set the control rate profile here 
    //setControlRateProfile(rateProfileSelection()->defaultRateProfileIndex);

    ninja_validate_config(self);

	// TODO: update control rates somehow here (currently we only use profile 0)
	//activateControlRateConfig();

    resetAdjustmentStates();

    useRcControlsConfig(modeActivationProfile()->modeActivationConditions);

#ifdef GPS
    gpsUsePIDs(pidProfile());
#endif

	// TODO: make sure we move all of this activation stuff into ninja so that we have access to the local state
	// all of this is really architectural failure. Failsafe is part of ninja object and should never be globally accessible.
    //failsafe_reset();
    //setAccelerationTrims(&sensorTrims()->accZero);

    //recalculateMagneticDeclination();

	/*
	// TODO: this is currently called before imu is initialized. We should make sure that we have initialized imu before this is called. 
	imu_configure(
		&default_imu, 
		imuConfig(),
		accelerometerConfig(),
		throttleCorrectionConfig(),
		gyro.scale, 
		acc.acc_1G
	);*/

    rx_resume_signal(self->rx);
}

void ninja_save_config(struct ninja *self){
    rx_suspend_signal(self->rx);

    writeConfigToEEPROM();

    rx_resume_signal();
}


void ninja_init(struct ninja *self, const struct system_calls *syscalls){
	memset(self, 0, sizeof(struct ninja));

	self->syscalls = syscalls;
	mixer_init(&self->mixer,
		mixerConfig(),
		motor3DConfig(),
		motorAndServoConfig(),
		rxConfig(),
		rcControlsConfig(),
		servoProfile()->servoConf,
		&syscalls->pwm,
		customMotorMixer(0), MAX_SUPPORTED_MOTORS);

	ins_init(&self->ins,
		boardAlignment(),
		imuConfig(),
		throttleCorrectionConfig(),
		gyroConfig(),
		compassConfig(),
		sensorTrims(),
		accelerometerConfig()
	);

	anglerate_init(&self->ctrl,
		&self->ins,
		pidProfile(),
		controlRateProfiles(0),
		imuConfig()->max_angle_inclination,
		&accelerometerConfig()->trims,
		rxConfig()
	);
/*
	float gs = 1.0f/16.4f;
	int16_t as = 512;
	self->syscalls->imu.get_gyro_acc_scale(&self->syscalls->imu, &gs, &as);
	ins_set_gyro_scale(&ninja.ins, gs);
	ins_set_acc_scale(&ninja.ins, as);
*/
	/*ins_set_gyro_alignment(&ninja.ins, gyroAlign);
	ins_set_acc_alignment(&ninja.ins, accAlign);
	ins_set_mag_alignment(&ninja.ins, magAlign);
*/

	anglerate_set_algo(&self->ctrl, pidProfile()->pidController);

	battery_init(&self->bat, batteryConfig());
	rx_init(&self->rx, &self->syscalls->pwm, modeActivationProfile()->modeActivationConditions);
	rc_command_init(&self->rc_command);
	rc_command_set_rate_config(&self->rc_command, controlRateProfiles(0)); 

#ifdef GPS
	if (feature(FEATURE_GPS)) {
		gpsInit();
		navigationInit(pidProfile());
	}
#endif

	mspInit(self);
	mspSerialInit();

#ifdef USE_CLI
	cliInit();
#endif

	failsafeInit();

#ifdef SONAR
	if (feature(FEATURE_SONAR)) {
		sonar_init(&default_sonar);
	}
#endif

#ifdef LED_STRIP
	ledStripInit();

	if (feature(FEATURE_LED_STRIP)) {
		ledStripEnable();
	}
#endif

#ifdef TELEMETRY
	if (feature(FEATURE_TELEMETRY)) {
		telemetryInit();
	}
#endif

#ifdef TRANSPONDER
	if (feature(FEATURE_TRANSPONDER)) {
		transponderInit(transponderConfig()->data);
		transponderEnable();
		transponderStartRepeating();
		systemState |= SYSTEM_STATE_TRANSPONDER_ENABLED;
	}
#endif

#ifdef BLACKBOX
	initBlackbox();
#endif

	sys_led_on(self->syscalls, 1);
	sys_led_off(self->syscalls, 0);
	for (uint8_t i = 0; i < 10; i++) {
		sys_led_toggle(self->syscalls, 1);
		sys_led_toggle(self->syscalls, 0);
		usleep(25000);
		sys_beep_on(self->syscalls);
		usleep(25000);
		sys_beep_off(self->syscalls);
	}
	sys_led_off(self->syscalls, 0);
	sys_led_off(self->syscalls, 1);

	ninja_sched_init(&self->sched, &syscalls->time);

	STATE_TRANSITION(ST_IDLE);
}

void ninja_arm(struct ninja *self){
	if (ARMING_FLAG(OK_TO_ARM)) {
		if (ARMING_FLAG(ARMED)) {
			return;
		}
		if (rcModeIsActive(BOXFAILSAFE)) {
			return;
		}
		if (!ARMING_FLAG(PREVENT_ARMING)) {
			ENABLE_ARMING_FLAG(ARMED);
			self->headFreeModeHold = DECIDEGREES_TO_DEGREES(ins_get_yaw_dd(&self->ins));
			mixer_enable_armed(&self->mixer, true);

#ifdef BLACKBOX
			if (feature(FEATURE_BLACKBOX)) {
				serialPort_t *sharedBlackboxAndMspPort = findSharedSerialPort(FUNCTION_BLACKBOX, FUNCTION_MSP);
				if (sharedBlackboxAndMspPort) {
					mspSerialReleasePortIfAllocated(sharedBlackboxAndMspPort);
				}
				startBlackbox();
			}
#endif
			// TODO: fix the delay
			//self->disarmAt = millis() + armingConfig()->auto_disarm_delay * 1000;   // start disarm timeout, will be extended when throttle is nonzero

			//beep to indicate arming
#ifdef GPS
			if (feature(FEATURE_GPS) && STATE(GPS_FIX) && GPS_numSat >= 5)
				beeper(BEEPER_ARMING_GPS_FIX);
			else
				beeper(BEEPER_ARMING);
#else
			beeper(BEEPER_ARMING);
#endif

			return;
		}
	}

	if (!ARMING_FLAG(ARMED)) {
		beeperConfirmationBeeps(1);
	}
}

void ninja_disarm(struct ninja *self){
	if (ARMING_FLAG(ARMED)) {
		// reset mixer minimum values in case something else has changed them
		mixer_enable_armed(&self->mixer, false);

		DISABLE_ARMING_FLAG(ARMED);

#ifdef BLACKBOX
		if (feature(FEATURE_BLACKBOX)) {
			finishBlackbox();
		}
#endif

		beeper(BEEPER_DISARMING);	  // emit disarm tone
	}
}

static void _process_tilt_controls(struct ninja *self){
	(void)self;
	// this is for dynamic pitch mode where the pitch channel drivers the motor tilting angle
	/*
	bool is_tilt = mixerConfig()->mixerMode == MIXER_QUADX_TILT1 || mixerConfig()->mixerMode == MIXER_QUADX_TILT2;

	if(USE_TILT && is_tilt){
		// TODO: refactor this once we have refactored rcCommand
		// in angle mode we set control channel value in RC command to zero.
		int16_t motor_pitch = 0;
		struct tilt_config *tilt = tiltConfig();
		if(FLIGHT_MODE(ANGLE_MODE)){
			// if control channel is pitch or roll then we need to feed 0 to the anglerate controller for corresponding channel
			if((tilt->control_channel == PITCH || tilt->control_channel == ROLL)){
				motor_pitch = rcCommand[tilt->control_channel];
				rcCommand[tilt->control_channel] = 0;
			} else {
				// get the control input for the tilt from the control channel
				motor_pitch = rc_get_channel_value(tilt->control_channel) - 1500;
			}
		} else if(FLIGHT_MODE(HORIZON_MODE)){
			// in horizon mode we do not deprive the anglerate controller of user input but we need to divide the pitch value so that anglerate controller pitches the body half way while we also tilt the propellers
			// this will tilt the body forward as well as tilt the propellers and once stick is all the way forward the quad body will tilt forward like in rate mode.
			// TODO: this mode currently is far from perfect.  Needs testing.
			if((tilt->control_channel == PITCH || tilt->control_channel == ROLL)){
				motor_pitch = rcCommand[tilt->control_channel] >> 1;
				rcCommand[tilt->control_channel] = motor_pitch;
			} else {
				// get the control input for the tilt from the control channel
				motor_pitch = rc_get_channel_value(tilt->control_channel) - 1500;
			}
		} else {
			// in rate mode we only allow manual tilting using one of the aux channels
			if(tilt->control_channel == AUX1 || tilt->control_channel == AUX2){
				motor_pitch = rc_get_channel_value(tilt->control_channel) - 1500;
			} else {
				motor_pitch = 0;
			}
		}

		struct tilt_input_params input = {
			.motor_pitch_dd = motor_pitch,
			.body_pitch_dd = ins_get_pitch_dd(&self->ins),
			.roll = rcCommand[ROLL],
			.pitch = rcCommand[PITCH],
			.yaw = rcCommand[YAW],
			.throttle = rcCommand[THROTTLE]
		};
		struct tilt_output_params output = {0};
		tilt_calculate_compensation(tiltConfig(), &input, &output);
		rcCommand[ROLL] = output.roll;
		rcCommand[PITCH] = output.pitch;
		rcCommand[YAW] = output.yaw;
		rcCommand[THROTTLE] = output.throttle;
	}
	*/
}

void _process_3d_throttle(struct ninja *self){
	// Scale roll/pitch/yaw uniformly to fit within throttle range
	int16_t throttleRange, throttle;
	int16_t throttleMin, throttleMax;
	static int16_t throttlePrevious = 0;   // Store the last throttle direction for deadband transitions in 3D.

	// Find min and max throttle based on condition. Use rcData for 3D to prevent loss of power due to min_check
	if (feature(FEATURE_3D)) {
		if (!ARMING_FLAG(ARMED)) throttlePrevious = rxConfig()->midrc; // When disarmed set to mid_rc. It always results in positive direction after arming.

		if ((rc_get_channel_value(THROTTLE) <= (rxConfig()->midrc - rcControlsConfig()->deadband3d_throttle))) { // Out of band handling
			throttleMax = motor3DConfig()->deadband3d_low;
			throttleMin = motorAndServoConfig()->minthrottle;
			throttlePrevious = throttle = rc_get_channel_value(THROTTLE);
		} else if (rc_get_channel_value(THROTTLE) >= (rxConfig()->midrc + rcControlsConfig()->deadband3d_throttle)) { // Positive handling
			throttleMax = motorAndServoConfig()->maxthrottle;
			throttleMin = motor3DConfig()->deadband3d_high;
			throttlePrevious = throttle = rc_get_channel_value(THROTTLE);
		} else if ((throttlePrevious <= (rxConfig()->midrc - rcControlsConfig()->deadband3d_throttle)))  { // Deadband handling from negative to positive
			throttle = throttleMax = motor3DConfig()->deadband3d_low;
			throttleMin = motorAndServoConfig()->minthrottle;
		} else {  // Deadband handling from positive to negative
			throttleMax = motorAndServoConfig()->maxthrottle;
			throttle = throttleMin = motor3DConfig()->deadband3d_high;
		}
	} else {
		throttle = rcCommand[THROTTLE];
		throttleMin = motorAndServoConfig()->minthrottle;
		throttleMax = motorAndServoConfig()->maxthrottle;
	}

	throttleRange = throttleMax - throttleMin;
	mixer_set_throttle_range(&self->mixer, throttleMin + throttleRange / 2, throttleMin, throttleMax);
	mixer_input_command(&self->mixer, MIXER_INPUT_G0_THROTTLE, throttle - 500);
}

void ninja_run_pid_loop(struct ninja *self, uint32_t dt_us){
	int16_t gyroRaw[3];
	if(self->syscalls->imu.read_gyro(&self->syscalls->imu, gyroRaw) == 0){
		ins_process_gyro(&self->ins, gyroRaw[0], gyroRaw[1], gyroRaw[2]);
		ins_update(&self->ins, dt_us * 1e-6f);
	}

	if(!ins_is_calibrated(&self->ins)){
		return;
	}

	ninja_process_rx(self);

	// TODO: set pid algo when config is applied
	//anglerate_set_algo(&self->ctrl, pidProfile()->pidController);
	anglerate_set_algo(&self->ctrl, PID_CONTROLLER_LUX_FLOAT);

	//if (FLIGHT_MODE(HORIZON_MODE)) {
	if (1) {
		int16_t hp_roll = 100-ABS(rc_get_channel_value(ROLL) - 1500) / 5;
		int16_t hp_pitch = 100-ABS(rc_get_channel_value(PITCH) - 1500) / 5;
		int16_t strength = MIN(hp_roll, hp_pitch);
		anglerate_set_level_percent(&self->ctrl, strength, strength);
	}

	_process_tilt_controls(self);

	//printf("input: %d %d %d\n", rcCommand[ROLL], rcCommand[PITCH], rcCommand[YAW]);
	//printf("gyro: %d %d %d", ins_get_gyro_x(&self->ins), ins_get_gyro_y(&self->ins), ins_get_gyro_z(&self->ins));
	anglerate_input_user(&self->ctrl, rcCommand[ROLL], rcCommand[PITCH], rcCommand[YAW]);
	anglerate_input_body_rates(&self->ctrl, ins_get_gyro_x(&self->ins), ins_get_gyro_y(&self->ins), ins_get_gyro_z(&self->ins));
	anglerate_input_body_angles(&self->ctrl, ins_get_roll_dd(&self->ins), ins_get_pitch_dd(&self->ins), ins_get_yaw_dd(&self->ins));
	anglerate_update(&self->ctrl, dt_us * 1e-6f);

	rcCommand[ROLL] = anglerate_get_roll(&self->ctrl);
	rcCommand[PITCH] = anglerate_get_pitch(&self->ctrl);
	rcCommand[YAW] = anglerate_get_yaw(&self->ctrl);

	//printf("output: %d %d %d\n", rcCommand[ROLL], rcCommand[PITCH], rcCommand[YAW]);
	// TODO: make sure we unit test 3d mode support
	//if(self->flags & NINJA_3D_THROTTLE){
	 //  _process_3d_throttle(self);
	//} else {
		mixer_set_throttle_range(&self->mixer, 1500, motorAndServoConfig()->minthrottle, motorAndServoConfig()->maxthrottle);
		mixer_input_command(&self->mixer, MIXER_INPUT_G0_THROTTLE, rcCommand[THROTTLE]);
	//}

	if (FLIGHT_MODE(PASSTHRU_MODE)) {
		// Direct passthru from RX
		mixer_input_command(&self->mixer, MIXER_INPUT_G0_ROLL, rcCommand[ROLL]);
		mixer_input_command(&self->mixer, MIXER_INPUT_G0_PITCH, rcCommand[PITCH]);
		mixer_input_command(&self->mixer, MIXER_INPUT_G0_YAW, rcCommand[YAW]);
	} else {
		// Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
		mixer_input_command(&self->mixer, MIXER_INPUT_G0_ROLL, anglerate_get_roll(&self->ctrl));
		mixer_input_command(&self->mixer, MIXER_INPUT_G0_PITCH, anglerate_get_pitch(&self->ctrl));
		mixer_input_command(&self->mixer, MIXER_INPUT_G0_YAW, -anglerate_get_yaw(&self->ctrl));
	}

	// center the RC input value around the RC middle value
	// by subtracting the RC middle value from the RC input value, we get:
	// data - middle = input
	// 2000 - 1500 = +500
	// 1500 - 1500 = 0
	// 1000 - 1500 = -500
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_ROLL, rc_get_channel_value(ROLL) - rxConfig()->midrc);
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_PITCH, rc_get_channel_value(PITCH)	- rxConfig()->midrc);
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_YAW, rc_get_channel_value(YAW)	  - rxConfig()->midrc);
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_THROTTLE, rc_get_channel_value(THROTTLE) - rxConfig()->midrc);
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_AUX1, rc_get_channel_value(AUX1)	 - rxConfig()->midrc);
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_AUX2, rc_get_channel_value(AUX2)	 - rxConfig()->midrc);
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_AUX3, rc_get_channel_value(AUX3)	 - rxConfig()->midrc);

	mixer_update(&self->mixer);
}

void ninja_heartbeat(struct ninja *self){
	ninja_sched_run(&self->sched);
}

