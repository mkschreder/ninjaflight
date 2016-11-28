#include <math.h>
#include <stdio.h>
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

void ninja_init(struct ninja *self, const struct system_calls *syscalls){
	memset(self, 0, sizeof(struct ninja));

	self->system = syscalls;

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
	rx_init(&self->rx, self->system);

	if (feature(FEATURE_RX_SERIAL))
		rx_set_type(&self->rx, RX_SERIAL);
	else if (feature(FEATURE_RX_MSP))
		rx_set_type(&self->rx, RX_MSP);
	else if (feature(FEATURE_RX_PPM))
		rx_set_type(&self->rx, RX_PPM);
	else
		rx_set_type(&self->rx, RX_PWM);

	rc_command_init(&self->rc_command, &self->rx);
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

	failsafe_init(&self->failsafe, self);

#ifdef SONAR
	if (feature(FEATURE_SONAR)) {
		sonar_init(&default_sonar);
	}
#endif

#ifdef LED_STRIP
	ledstrip_init(&self->ledstrip, self->system, &self->bat, &self->failsafe, &self->rx);

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

	ninja_config_load(self);

	ninja_sched_init(&self->sched, &self->system->time);
}

void ninja_arm(struct ninja *self){
	if (self->is_armed) {
		return;
	}
	if (rcModeIsActive(BOXFAILSAFE)) {
		return;
	}
	if (!self->is_armed) {
		self->is_armed = true;
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
			beeper_start(&self->beeper, BEEPER_ARMING_GPS_FIX);
		else
			beeper_start(&self->beeper, BEEPER_ARMING);
#else
		beeper_start(&self->beeper, BEEPER_ARMING);
#endif

		return;
	}
}

void ninja_disarm(struct ninja *self){
	if (self->is_armed) {
		// reset mixer minimum values in case something else has changed them
		mixer_enable_armed(&self->mixer, false);

#ifdef BLACKBOX
		if (feature(FEATURE_BLACKBOX)) {
			finishBlackbox();
		}
#endif

		beeper_start(&self->beeper, BEEPER_DISARMING);	  // emit disarm tone
		self->is_armed = false;
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

uint32_t ninja_has_sensors(struct ninja *self, uint32_t sensor_mask){
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

void ninja_run_pid_loop(struct ninja *self, uint32_t dt_us){
	int16_t gyroRaw[3];
	if(sys_gyro_read(self->system, gyroRaw) < 0){
		self->sensors &= ~NINJA_SENSOR_GYRO;
		return;
	}
	self->sensors |= NINJA_SENSOR_GYRO;

	ins_process_gyro(&self->ins, gyroRaw[0], gyroRaw[1], gyroRaw[2]);
	ins_update(&self->ins, dt_us * 1e-6f);

	if(!ins_is_calibrated(&self->ins)){
		return;
	}

	ninja_process_rx(self);

	// TODO: set pid algo when config is applied
	//anglerate_set_algo(&self->ctrl, pidProfile()->pidController);
	anglerate_set_algo(&self->ctrl, PID_CONTROLLER_LUX_FLOAT);

	//if (FLIGHT_MODE(HORIZON_MODE)) {
	if (1) {
		int16_t hp_roll = 100-ABS(rx_get_channel(&self->rx, ROLL) - 1500) / 5;
		int16_t hp_pitch = 100-ABS(rx_get_channel(&self->rx, PITCH) - 1500) / 5;
		int16_t strength = MIN(hp_roll, hp_pitch);
		anglerate_set_level_percent(&self->ctrl, strength, strength);
	}

	_process_tilt_controls(self);

	/*printf("input: %d %d %d, ", rcCommand[ROLL], rcCommand[PITCH], rcCommand[YAW]);
	printf("rx: %d %d %d, ", rx_get_channel(&self->rx, ROLL), rx_get_channel(&self->rx, PITCH), rx_get_channel(&self->rx, YAW));
	printf("gyro: %d %d %d, ", ins_get_gyro_x(&self->ins), ins_get_gyro_y(&self->ins), ins_get_gyro_z(&self->ins));
	*/
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

	// center the RC input value around the RC middle value
	// by subtracting the RC middle value from the RC input value, we get:
	// data - middle = input
	// 2000 - 1500 = +500
	// 1500 - 1500 = 0
	// 1000 - 1500 = -500
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_ROLL, rx_get_channel(&self->rx, ROLL) - rxConfig()->midrc);
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_PITCH, rx_get_channel(&self->rx, PITCH)	- rxConfig()->midrc);
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_YAW, rx_get_channel(&self->rx, YAW)	  - rxConfig()->midrc);
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_THROTTLE, rx_get_channel(&self->rx, THROTTLE) - rxConfig()->midrc);
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_AUX1, rx_get_channel(&self->rx, AUX1)	 - rxConfig()->midrc);
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_AUX2, rx_get_channel(&self->rx, AUX2)	 - rxConfig()->midrc);
	mixer_input_command(&self->mixer, MIXER_INPUT_G3_RC_AUX3, rx_get_channel(&self->rx, AUX3)	 - rxConfig()->midrc);

	mixer_update(&self->mixer);
}

void ninja_heartbeat(struct ninja *self){
	ninja_sched_run(&self->sched);
}

/**
 * Enables the given flight mode.  A beep is sounded if the flight mode
 * has changed.  Returns the new 'flightModeFlags' value.
 */
 /*
 // TODO: flight modes
uint16_t enableFlightMode(struct ninja *self, flightModeFlags_e mask){
    uint16_t oldVal = self->flightModeFlags;

    self->flightModeFlags |= (mask);
    if (self->flightModeFlags != oldVal)
        beeper_multi_beep(&self->beeper, 1);
    return self->flightModeFlags;
}
*/
/**
 * Disables the given flight mode.  A beep is sounded if the flight mode
 * has changed.  Returns the new 'flightModeFlags' value.
 */
 /*
uint16_t disableFlightMode(struct ninja *self, flightModeFlags_e mask){
    uint16_t oldVal = self->flightModeFlags;

    self->flightModeFlags &= ~(mask);
    if (self->flightModeFlags != oldVal)
        beeper_multi_beeps(&self->beeper, 1);
    return self->flightModeFlags;
}
bool sensors_enabled(struct ninja *self, uint32_t mask){
    return self->enabledSensors & mask;
}

void enable_sensor(struct ninja *self, uint32_t mask){
    self->enabledSensors |= mask;
}

void disable_sensor(struct ninja *self, uint32_t mask){
    self->enabledSensors &= ~(mask);
}

uint32_t enabled_sensors(struct ninja *self){
    return self->enabledSensors;
}

*/
/**
 * @defgroup INDICATORS Indicators
 */
