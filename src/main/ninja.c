#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "system_calls.h"

#include "target.h"

#include "drivers/system.h"
#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"
#include "common/filter.h"

#include "config/config.h"
#include "config/config_eeprom.h"
#include "config/profile.h"
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

#include "flight/rate_profile.h"
#include "flight/mixer.h"
#include "flight/anglerate.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/gtune.h"
#include "flight/navigation.h"
#include "flight/tilt.h"

#include "sensors/instruments.h"
#include "sensors/gps.h"

#include "config/config.h"
#include "config/feature.h"

#include "msp.h"
#include "ninjaflight.h"
#include "ninja.h"
#include "ninja_sched.h"

static void _rc_key_state_change(struct rc_event_listener *evl, rc_key_t key, rc_key_state_t state){
	struct ninja *self = container_of(evl, struct ninja, rc_evl);
	(void)self;
	(void)key;
	(void)state;
	// forward to the current state if the state has defined a callback
	//if(self->state->on_key_event) self->state->on_key_event(self->state, self, key, state);
}

static void _output_motors_disarmed(struct ninja *self){
	// TODO: here we can output values from the ground control
	for(int c = 0; c < MIXER_MAX_MOTORS; c++){
		sys_motor_write(self->system, c, 1000);
	}
	for(int c = 0; c < MIXER_MAX_SERVOS; c++){
		sys_servo_write(self->system, c, 1500);
	}
}

void ninja_init(struct ninja *self, const struct system_calls *syscalls, struct config *config){
	memset(self, 0, sizeof(struct ninja));

	self->system = syscalls;
	self->config = config;

	config_reset(self->config);

	rc_adj_init(&self->rc_adj, self, self->config);
	mixer_init(&self->mixer, self->config, &syscalls->pwm);
	ins_init(&self->ins, self->config);
	anglerate_init(&self->ctrl, &self->ins, 0, self->config);
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

	beeper_init(&self->beeper, self->system);

	anglerate_set_algo(&self->ctrl, config_get_profile(self->config)->pid.pidController);

	battery_init(&self->bat, &self->config->bat);
	rx_init(&self->rx, self->system, self->config);

	if (feature(self->config, FEATURE_RX_SERIAL))
		rx_set_type(&self->rx, RX_SERIAL);
	else if (feature(self->config, FEATURE_RX_MSP))
		rx_set_type(&self->rx, RX_MSP);
	else if (feature(self->config, FEATURE_RX_PPM))
		rx_set_type(&self->rx, RX_PPM);
	else
		rx_set_type(&self->rx, RX_PWM);

	self->rc_evl = (struct rc_event_listener){
		.on_key_state = _rc_key_state_change,
		.on_key_repeat = NULL
	};

	rc_init(&self->rc, &self->rx, &self->rc_evl, self->config);

	#ifdef GPS
	if (feature(FEATURE_GPS)) {
		if(gps_init(&self->gps, self->system) < 0){
			featureClear(self->config, FEATURE_GPS);
		} else {
			navigationInit(&config_get_profile(self->config)->pid);
		}
	}
#endif

	msp_init(&self->msp, self, self->config);
	serial_msp_init(&self->serial_msp, self->config, &self->msp);

	cli_init(&self->cli, self);

	failsafe_init(&self->failsafe, self, self->config);

#ifdef SONAR
	if (feature(FEATURE_SONAR)) {
		sonar_init(&default_sonar);
	}
#endif

#ifdef LED_STRIP
	ledstrip_init(&self->ledstrip, self->config, self->system, &self->rx, &self->failsafe);

	if (feature(self->config, FEATURE_LED_STRIP)) {
		//ledStripEnable();
	}
#endif

#ifdef TELEMETRY
	if (feature(self->config, FEATURE_TELEMETRY)) {
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
	blackbox_init(&self->blackbox, self);
#endif

	sys_led_on(self->system, 1);
	sys_led_off(self->system, 0);
	for (uint8_t i = 0; i < 10; i++) {
		sys_led_toggle(self->system, 1);
		sys_led_toggle(self->system, 0);
		usleep(25000);
		sys_beeper_on(self->system);
		usleep(25000);
		sys_beeper_off(self->system);
	}
	sys_led_off(self->system, 0);
	sys_led_off(self->system, 1);

	//ninja_config_load(self);

	ninja_sched_init(&self->sched, &self->system->time, self->config);

	_output_motors_disarmed(self);
}

void ninja_arm(struct ninja *self){
	self->is_armed = true;
	mixer_enable_armed(&self->mixer, true);
	//beep to indicate arming
	beeper_start(&self->beeper, BEEPER_ARMING);
}

void ninja_disarm(struct ninja *self){
	(void)self;
	mixer_enable_armed(&self->mixer, false);
	beeper_start(&self->beeper, BEEPER_DISARMING);	  // emit disarm tone
	self->is_armed = false;
}

	/*
static void _process_tilt_controls(struct ninja *self){
	(void)self;
	// this is for dynamic pitch mode where the pitch channel drivers the motor tilting angle
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
}

	*/
/*
TODO: 3d throttle control
void _process_3d_throttle(struct ninja *self){
	// Scale roll/pitch/yaw uniformly to fit within throttle range
	int16_t throttleRange, throttle;
	int16_t throttleMin, throttleMax;
	static int16_t throttlePrevious = 0;   // Store the last throttle direction for deadband transitions in 3D.

	// Find min and max throttle based on condition. Use rcData for 3D to prevent loss of power due to min_check
	if (feature(FEATURE_3D)) {
		if (!ARMING_FLAG(ARMED)) throttlePrevious = rxConfig()->midrc; // When disarmed set to mid_rc. It always results in positive direction after arming.

		if ((rx_get_channel(&self->rx, THROTTLE) <= (rxConfig()->midrc - rcControlsConfig()->deadband3d_throttle))) { // Out of band handling
			throttleMax = motor3DConfig()->deadband3d_low;
			throttleMin = motorAndServoConfig()->minthrottle;
			throttlePrevious = throttle = rx_get_channel(&self->rx, THROTTLE);
		} else if (rx_get_channel(&self->rx, THROTTLE) >= (rxConfig()->midrc + rcControlsConfig()->deadband3d_throttle)) { // Positive handling
			throttleMax = motorAndServoConfig()->maxthrottle;
			throttleMin = motor3DConfig()->deadband3d_high;
			throttlePrevious = throttle = rx_get_channel(&self->rx, THROTTLE);
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
*/

uint32_t ninja_has_sensors(struct ninja *self, sensor_mask_t sensor_mask){
	return !!(self->sensors & sensor_mask);
}

bool ninja_is_armed(struct ninja *self){
	return self->is_armed;
}

/*
void _check_battery(struct ninja *self){
	// TODO: batter low voltage notification
	switch(battery_get_state(&self->battery)){
		case BATTERY_OK:
			if (self->vbat <= (self->batteryWarningVoltage - VBATT_HYSTERESIS)) {
				self->batteryState = BATTERY_WARNING;
				beeper(BEEPER_BAT_LOW);
			}
			break;
		case BATTERY_WARNING:
			if (self->vbat <= (self->batteryCriticalVoltage - VBATT_HYSTERESIS)) {
				self->batteryState = BATTERY_CRITICAL;
				beeper(BEEPER_BAT_CRIT_LOW);
			} else if (self->vbat > (self->batteryWarningVoltage + VBATT_HYSTERESIS)){
				self->batteryState = BATTERY_OK;
			} else {
				beeper(BEEPER_BAT_LOW);
			}
			break;
		case BATTERY_CRITICAL:
			if (self->vbat > (self->batteryCriticalVoltage + VBATT_HYSTERESIS)){
				self->batteryState = BATTERY_WARNING;
				beeper(BEEPER_BAT_LOW);
			} else {
				beeper(BEEPER_BAT_CRIT_LOW);
			}
			break;
		case BATTERY_NOT_PRESENT:
			break;
		default:break;
	}
}
*/

#if 0
// TODO: beeper on off based on box status
static void _update_beeper(struct ninja *self){
	// If beeper option from AUX switch has been selected
	if (rcModeIsActive(BOXBEEPERON)) {
#ifdef GPS
		if (feature(FEATURE_GPS)) {
			beeperGpsStatus();
		} else {
			beeper_start(&self->beeper, BEEPER_RX_SET);
		}
#else
		beeper_start(&self->beeper, BEEPER_RX_SET);
#endif
	}
}
#endif

bool _prearm_checks_ok(struct ninja *self){
	// we can only arm if we are calibrated
	if(!ins_is_calibrated(&self->ins)) return false;
	return true;
}

bool _disarm_checks_ok(struct ninja *self){
	// we don't want to allow disarming if quad is moving
	if(ABS(ins_get_gyro_x(&self->ins)) > 100 || ABS(ins_get_gyro_y(&self->ins)) > 100 || ABS(ins_get_gyro_z(&self->ins))) return false;
	return true;
}

static void _run_control_loop(struct ninja *self){
	//struct ns_armed *state = container_of(_state, struct ns_armed, state);
	sys_micros_t dt_us = self->loop_time;

	// TODO: set pid algo when config is applied
	//anglerate_set_algo(&self->ctrl, pidProfile()->pidController);
	anglerate_set_algo(&self->ctrl, PID_CONTROLLER_LUX_FLOAT);

	if (rc_key_state(&self->rc, RC_KEY_FUNC_LEVEL) == RC_KEY_PRESSED) {
		anglerate_set_level_percent(&self->ctrl, 100, 100);
	} else if(rc_key_state(&self->rc, RC_KEY_FUNC_BLEND) == RC_KEY_PRESSED){
		int16_t hp_roll = 100-ABS(rx_get_channel(&self->rx, ROLL) - 1500) / 5;
		int16_t hp_pitch = 100-ABS(rx_get_channel(&self->rx, PITCH) - 1500) / 5;
		if(ABS(ins_get_pitch_dd(&self->ins)) > 800)
			hp_roll = hp_pitch = 0;
		int16_t strength = MIN(hp_roll, hp_pitch);
		anglerate_set_level_percent(&self->ctrl, strength, strength);
	} else {
		anglerate_set_level_percent(&self->ctrl, 0, 0);
	}
	//_process_tilt_controls(self);

	int16_t roll = rc_get_command(&self->rc, ROLL);
	int16_t pitch =  rc_get_command(&self->rc, PITCH);
	int16_t yaw = rc_get_command(&self->rc, YAW);

	// prevent spinup when just armed
	if(rc_get_command(&self->rc, THROTTLE) < -480) yaw = 0;

	float combined = degreesToRadians(DECIDEGREES_TO_DEGREES(ins_get_pitch_dd(&self->ins)));
    float tmpCosine = cos_approx(combined);
	float rollCompensation = roll * tmpCosine;
	float rollCompensationInv = roll - rollCompensation;
	float yawCompensation = yaw * tmpCosine;
	float yawCompensationInv = yaw - yawCompensation;

	roll = (yawCompensationInv + rollCompensation);
	yaw = -(yawCompensation + rollCompensationInv);

	anglerate_input_user(&self->ctrl, roll, pitch, yaw);
	anglerate_input_body_rates(&self->ctrl, ins_get_gyro_x(&self->ins), ins_get_gyro_y(&self->ins), ins_get_gyro_z(&self->ins));
	anglerate_input_body_angles(&self->ctrl, ins_get_roll_dd(&self->ins), ins_get_pitch_dd(&self->ins), ins_get_yaw_dd(&self->ins));
	anglerate_update(&self->ctrl, dt_us * 1e-6f);

	mixer_set_throttle_range(&self->mixer, 1500, self->config->pwm_out.minthrottle, self->config->pwm_out.maxthrottle);
	mixer_input_command(&self->mixer, MIXER_INPUT_G0_THROTTLE, rc_get_command(&self->rc, THROTTLE));

	// TODO: flight modes
	//if (FLIGHT_MODE(PASSTHRU_MODE)) {
		// Direct passthru from RX
		//mixer_input_command(&self->mixer, MIXER_INPUT_G0_ROLL, rcCommand[ROLL]);
		//mixer_input_command(&self->mixer, MIXER_INPUT_G0_PITCH, rcCommand[PITCH]);
		//mixer_input_command(&self->mixer, MIXER_INPUT_G0_YAW, rcCommand[YAW]);
	//} else {
		// Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
		mixer_input_command(&self->mixer, MIXER_INPUT_G0_ROLL, anglerate_get_roll(&self->ctrl));
		mixer_input_command(&self->mixer, MIXER_INPUT_G0_PITCH, anglerate_get_pitch(&self->ctrl));
		mixer_input_command(&self->mixer, MIXER_INPUT_G0_YAW, -anglerate_get_yaw(&self->ctrl));
	//}

	//printf("out: %d %d %d\n", anglerate_get_roll(&self->ctrl), anglerate_get_pitch(&self->ctrl), anglerate_get_yaw(&self->ctrl));
	// center the RC input value around the RC middle value
	// by subtracting the RC middle value from the RC input value, we get:
	// data - middle = input
	// 2000 - 1500 = +500
	// 1500 - 1500 = 0
	// 1000 - 1500 = -500
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_ROLL, rx_get_channel(&self->rx, ROLL) - self->config->rx.midrc);
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_PITCH, rx_get_channel(&self->rx, PITCH)	- self->config->rx.midrc);
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_YAW, rx_get_channel(&self->rx, YAW)	  - self->config->rx.midrc);
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_THROTTLE, rx_get_channel(&self->rx, THROTTLE) - self->config->rx.midrc);
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_AUX1, rx_get_channel(&self->rx, AUX1)	 - self->config->rx.midrc);
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_AUX2, rx_get_channel(&self->rx, AUX2)	 - self->config->rx.midrc);
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_AUX3, rx_get_channel(&self->rx, AUX3)	 - self->config->rx.midrc);

	mixer_update(&self->mixer);

	//self->disarmAt = millis() + armingConfig()->auto_disarm_delay * 1000;   // start disarm timeout, will be extended when throttle is nonzero
}

/**
 * This is the seemingly "complicated" (it's actually quite smart)
 * arming/disarming state routine. We add support for delay when arming and we
 * also handle special cases such as duplicate disarms and automatic disarm
 * after a certain idle period - all in the same little code block. It is
 * written with protothread concept making it quite compact. Not that this is a
 * state machine and not an ordinary function. Since we jump around in it back
 * and forth, using local variables must be avoided.
 */
static PT_THREAD(_fsm_arming(struct ninja *self)){
	// specifies how long we need to keep stick in arming position until we actually arm
	static const int NINJA_ARM_DURATION = 500;
	// specifies how long sticks need to be idle before disarm (throttle low, rpy centered)
	static const int NINJA_AUTO_DISARM_DURATION = 5000;
	PT_BEGIN(&self->state_arming);
	// spin for ever
	while(true){
		// we assume disarmed state
wait_for_arm:
		// wait untill arming is allowed and user triggers arming with controls
		// we also check for the disarm action so we can sound the beeper if user tries to do it
		PT_WAIT_UNTIL(&self->state_arming,
			(_prearm_checks_ok(self) &&
			(rc_key_state(&self->rc, RC_KEY_STICK_ARM) == RC_KEY_PRESSED ||
			rc_key_state(&self->rc, RC_KEY_FUNC_ARM) == RC_KEY_PRESSED)) ||
			rc_key_state(&self->rc, RC_KEY_STICK_DISARM) == RC_KEY_PRESSED);
		// if user mistakenly tries to disarm when we are disarmed then we sound the beeper and yield
		if(rc_key_state(&self->rc, RC_KEY_STICK_DISARM) == RC_KEY_PRESSED){
			// use arming delay here because it is not used for anything
			self->arming_delay = sys_millis(self->system) + NINJA_ARM_DURATION;
			beeper_start(&self->beeper, BEEPER_DISARM_REPEAT);
			PT_YIELD(&self->state_arming);
			goto wait_for_arm;
		}
		// if arming via sticks then we need to do the delay
		if(rc_key_state(&self->rc, RC_KEY_STICK_ARM) == RC_KEY_PRESSED){
			self->arming_delay = sys_millis(self->system) + NINJA_ARM_DURATION;
			// wait until either the timer times out or user aborts the arming by releasing the sticks
			PT_WAIT_UNTIL(&self->state_arming,
				rc_key_state(&self->rc, RC_KEY_STICK_ARM) == RC_KEY_RELEASED || (self->arming_delay - sys_millis(self->system) < 0));
			// if it was aborted then we restart the operation
			if(rc_key_state(&self->rc, RC_KEY_STICK_ARM) == RC_KEY_RELEASED)
				goto wait_for_arm;
		}
		// arm the copter and if arming for whatever reason fails we fall back to the top
		ninja_arm(self);
wait_for_disarm:
		// wait until either the stick disarm is initiated or disarm switch is activated
		PT_WAIT_UNTIL(&self->state_arming,
			rc_key_state(&self->rc, RC_KEY_IDLE) == RC_KEY_PRESSED ||
			rc_key_state(&self->rc, RC_KEY_STICK_DISARM) == RC_KEY_PRESSED ||
			rc_key_state(&self->rc, RC_KEY_FUNC_ARM) == RC_KEY_RELEASED);

		// if we wake up because user has put sticks into idle position then we need
		// to start a timer and wait while stick remains in that position and timer has not expired.
		// if stick remains in idle position for the required time then we disarm
		if(rc_key_state(&self->rc, RC_KEY_IDLE) == RC_KEY_PRESSED){
			self->disarm_timeout = sys_millis(self->system) + NINJA_AUTO_DISARM_DURATION;
			PT_WAIT_WHILE(&self->state_arming,
				rc_key_state(&self->rc, RC_KEY_IDLE) == RC_KEY_PRESSED &&
				(self->disarm_timeout - sys_millis(self->system)) >= 0);
			// if timeout has expired then disarm
			if(_disarm_checks_ok(self) && (self->disarm_timeout - sys_millis(self->system)) < 0){
				ninja_disarm(self);
				goto wait_for_arm;
			}
			// otherwise go back to loop
			goto wait_for_disarm;
		}
		// if it is the disarm timeout then we need to check that sticks are idle and disarm if this is the case
		if((self->disarm_timeout - sys_millis(self->system)) < 0){
			// we check if disarming is a good idea too (and that sticks are in default position)
			if(_disarm_checks_ok(self) && rc_key_state(&self->rc, RC_KEY_IDLE) == RC_KEY_PRESSED){
				ninja_disarm(self);
				goto wait_for_arm;
			}
			// otherwise we extend the timeout and go back to sleep
			self->disarm_timeout = sys_millis(self->system) + NINJA_AUTO_DISARM_DURATION;
			goto wait_for_disarm;
		}
		// if disarming using sticks then we need to do the disarm delay sequence
		if(rc_key_state(&self->rc, RC_KEY_STICK_DISARM) == RC_KEY_PRESSED){
			// if we disarm using sticks then we want to do the disarm checks.
			// switch disarm is instantaneous though and does not do this check
			if(!_disarm_checks_ok(self)){
				// have to yield because above PT_WAIT_UNTIL will no yield as long as sticks are in this position
				PT_YIELD(&self->state_arming);
				goto wait_for_disarm;
			}
			self->arming_delay = sys_millis(self->system) + NINJA_ARM_DURATION;
			// wait until either the timer times out or user aborts disarm by releasing the sticks
			PT_WAIT_UNTIL(&self->state_arming,
				rc_key_state(&self->rc, RC_KEY_STICK_DISARM) == RC_KEY_RELEASED || (self->arming_delay < sys_millis(self->system)));
			// if it was aborted then we restart the operation
			if(rc_key_state(&self->rc, RC_KEY_STICK_DISARM) == RC_KEY_RELEASED)
				goto wait_for_disarm;
		}
		// when we get here we disarm and start over from the start waiting for arming command
		ninja_disarm(self);
	}
	PT_END(&self->state_arming);
}

static PT_THREAD(_fsm_controller(struct ninja *self)){
	PT_BEGIN(&self->state_ctrl);
	while(true){
		// controller starts off by waiting until all sensors are calibrated
		PT_WAIT_UNTIL(&self->state_ctrl, ins_is_calibrated(&self->ins));
		// while armed we run the controller at each tick
		while(true){
			if(self->is_armed) {
				_run_control_loop(self);

				// these inputs are valid for armed mode
				if(rc_key_state(&self->rc, RC_KEY_ACC_INFLIGHT_CALIB) == RC_KEY_PRESSED){
					// TODO: handle inflight calibration
				}
			} else {
				_output_motors_disarmed(self);

				// these inputs will only be allowed when we are disarmed
				if(rc_key_state(&self->rc, RC_KEY_GYROCAL) == RC_KEY_PRESSED){
					// TODO: maybe also handle baro calibration here
					ins_start_gyro_calibration(&self->ins);
					// wait until gyro has been calibrated
					PT_WAIT_UNTIL(&self->state_ctrl, ins_is_calibrated(&self->ins));
				}

				if(rc_key_state(&self->rc, RC_KEY_ACCCAL) == RC_KEY_PRESSED){
					ins_start_acc_calibration(&self->ins);
					PT_WAIT_UNTIL(&self->state_ctrl, ins_is_calibrated(&self->ins));
				}

				if(rc_key_state(&self->rc, RC_KEY_MAGCAL) == RC_KEY_PRESSED){
					// TODO: mag calibration involves multiple states which we need to code
					ins_start_mag_calibration(&self->ins);
					PT_WAIT_UNTIL(&self->state_ctrl, ins_is_calibrated(&self->ins));
				}

				if(rc_key_state(&self->rc, RC_KEY_PROFILE1) == RC_KEY_PRESSED){
					// TODO
					//ninja_config_change_profile(self, 0);
					beeper_multi_beeps(&self->beeper, 1);
				}
				if(rc_key_state(&self->rc, RC_KEY_PROFILE2) == RC_KEY_PRESSED){
					// TODO
					//ninja_config_change_profile(self, 1);
					beeper_multi_beeps(&self->beeper, 2);
				}
				if(rc_key_state(&self->rc, RC_KEY_PROFILE3) == RC_KEY_PRESSED){
					// TODO
					//ninja_config_change_profile(self, 2);
					beeper_multi_beeps(&self->beeper, 3);
				}

				if(rc_key_state(&self->rc, RC_KEY_SAVE) == RC_KEY_PRESSED){
					// TODO
					//ninja_config_save(self);
				}
			}
			// we must remember to yield, otherwise we will lock up
			PT_YIELD(&self->state_ctrl);
		}
	}
	PT_END(&self->state_ctrl);
}

void ninja_run_pid_loop(struct ninja *self, uint32_t dt_us){
	int16_t gyroRaw[3];
	if(sys_gyro_read(self->system, gyroRaw) < 0){
		self->sensors &= ~NINJA_SENSOR_GYRO;
		return;
	}
	self->sensors |= NINJA_SENSOR_GYRO;

	self->loop_time = dt_us;

	ins_process_gyro(&self->ins, gyroRaw[0], gyroRaw[1], gyroRaw[2]);
	ins_update(&self->ins, dt_us * 1e-6f);

	// handle arming/disarming
	_fsm_arming(self);

	_fsm_controller(self);
}

/**
 * @callgraph
 */
void ninja_heartbeat(struct ninja *self){
	ninja_sched_run(&self->sched);
}

/**
 * @defgroup INDICATORS Indicators
 */
