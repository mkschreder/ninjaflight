#include "ninja.h"

static void updateMagHold(void)
{
	// TODO: really refactor this kind of crap. This belongs in flight control code
    if (ABS(rcCommand[YAW]) < 15 && FLIGHT_MODE(MAG_MODE)) {
        int16_t dif = DECIDEGREES_TO_DEGREES(ins_get_yaw_dd(&ninja.ins)) - ninja.magHold;
        if (dif <= -180)
            dif += 360;
        if (dif >= +180)
            dif -= 360;
        dif *= -rcControlsConfig()->yaw_control_direction;
		// TODO: small angle was set in imu before. Generally this kind of logic is horrible. This needs to be scrapped.
        if (STATE(SMALL_ANGLE))
            rcCommand[YAW] -= dif * pidProfile()->P8[PIDMAG] / 30;    // 18 deg
    } else
        ninja.magHold = DECIDEGREES_TO_DEGREES(ins_get_yaw_dd(&ninja.ins));
}


// TODO: untangle all the crap with dt
void ninja_control_run(struct ninja *self, float dt_){
	UNUSED(self);
	UNUSED(dt_);

    currentTime = micros();
	static uint32_t previousIMUUpdateTime = 0;
    float dt = (currentTime - previousIMUUpdateTime) * 1e-6f;
	dT = dt;
    previousIMUUpdateTime = currentTime;

	int16_t rawgyro[3];
	if (gyro.read(rawgyro)) {
		ins_process_gyro(&ninja.ins, rawgyro[0], rawgyro[1], rawgyro[2]);
	}

#if defined(GPS)
/*
	// TODO: rethink how to enter gps yaw angle into ins (probably we enter all gps data and then ins decides how to use the angle!)
    if (STATE(FIXED_WING) && sensors(SENSOR_GPS) && STATE(GPS_FIX) && GPS_numSat >= 5 && GPS_speed >= 300) {
        // In case of a fixed-wing aircraft we can use GPS course over ground to correct heading
        ins_input_yaw_dd(&default_imu, GPS_ground_course);
    }
	*/
#endif

	// TODO: wtf is this crap?
#if defined(BARO) || defined(SONAR)
    haveUpdatedRcCommandsOnce = true;
#endif

    // Read out gyro temperature. can use it for something somewhere. maybe get MCU temperature instead? lots of fun possibilities.
    if (gyro.temperature) {
        gyro.temperature(&self->telemTemperature1);
    }

	if (USE_MAG && sensors(SENSOR_MAG)) {
		updateMagHold();
	}

#ifdef GTUNE
        updateGtuneState();
#endif

#if defined(BARO) || defined(SONAR)
        if (sensors(SENSOR_BARO) || sensors(SENSOR_SONAR)) {
            if (FLIGHT_MODE(BARO_MODE) || FLIGHT_MODE(SONAR_MODE)) {
                applyAltHold();
            }
        }
#endif

    // If we're armed, at minimum throttle, and we do arming via the
    // sticks, do not process yaw input from the rx.  We do this so the
    // motors do not spin up while we are trying to arm or disarm.
    // Allow yaw control for tricopters if the user wants the servo to move even when unarmed.
    if (isUsingSticksForArming() && rc_get_channel_value(THROTTLE) <= rxConfig()->mincheck
#ifdef USE_SERVOS
                && !((mixerConfig()->mixerMode == MIXER_TRI || mixerConfig()->mixerMode == MIXER_CUSTOM_TRI) && mixerConfig()->tri_unarmed_servo)
#endif
                && mixerConfig()->mixerMode != MIXER_AIRPLANE
                && mixerConfig()->mixerMode != MIXER_FLYING_WING
    ) {
        rcCommand[YAW] = 0;
    }

	// TODO: make throttle correction work after refactoring
	/*
    if (throttleCorrectionConfig()->throttle_correction_value && (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE))) {
        rcCommand[THROTTLE] += imu_calc_throttle_angle_correction(&default_imu, throttleCorrectionConfig()->throttle_correction_value);
    }*/

#ifdef GPS
    if (sensors(SENSOR_GPS)) {
        if ((FLIGHT_MODE(GPS_HOME_MODE) || FLIGHT_MODE(GPS_HOLD_MODE)) && STATE(GPS_FIX_HOME)) {
            updateGpsStateForHomeAndHoldMode();
        }
    }
#endif
	// TODO: move this once we have tested current refactored code
	ninja_update_controller(&ninja, dt);

    	// TODO: refactor this
#ifdef GTUNE
	// TODO: unit test this gtune stuff. This may not be the right place to put it.
	if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
		 for(int c = 0; c < 3; c++) calculate_Gtune(c);
	}
#endif

		// TODO: gimbal should be driven directly through the aux channels when enabled.
	//input[INPUT_GIMBAL_PITCH] = scaleRange(self->gimbal_angles[PITCH], -1800, 1800, -500, +500);
	//input[INPUT_GIMBAL_ROLL] = scaleRange(self->gimbal_angles[ROLL], -1800, 1800, -500, +500);

	//input[INPUT_STABILIZED_THROTTLE] = mixer_get_motor_value(&default_mixer, 0) - 1500;  // Since it derives from rcCommand or mincommand and must be [-500:+500]

	// write out all motors and servos
	for (int i = 0; i < MIXER_MAX_MOTORS; i++){
		pwmWriteMotor(i, mixer_get_motor_value(&ninja.mixer, i));
	}
#ifdef USE_SERVOS
	for (int i = 0; i < MIXER_MAX_SERVOS; i++){
		pwmWriteServo(i, mixer_get_servo_value(&ninja.mixer, i));
	}
#endif
	if (feature(FEATURE_ONESHOT125)) {
		pwmCompleteOneshotMotorUpdate(MIXER_MAX_MOTORS);
	}

#ifdef USE_SDCARD
        afatfs_poll();
#endif

#ifdef BLACKBOX
    if (!cliMode && feature(FEATURE_BLACKBOX)) {
        handleBlackbox();
    }
#endif
}

