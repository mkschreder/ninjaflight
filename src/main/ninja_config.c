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

#include "config/config.h"
#include "config/feature.h"

#include "ninjaflight.h"
#include "ninja.h"
#include "ninja_sched.h"


static void ninja_validate_config(struct ninja *self){
	(void)self;
    if (!(feature(self->config, FEATURE_RX_PARALLEL_PWM) || feature(self->config, FEATURE_RX_PPM) || feature(self->config, FEATURE_RX_SERIAL) || feature(self->config, FEATURE_RX_MSP))) {
        featureSet(self->config, DEFAULT_RX_FEATURE);
    }

    if (feature(self->config, FEATURE_RX_PPM)) {
        featureClear(self->config, FEATURE_RX_PARALLEL_PWM | FEATURE_RX_SERIAL | FEATURE_RX_MSP);
    }

    if (feature(self->config, FEATURE_RX_MSP)) {
        featureClear(self->config, FEATURE_RX_SERIAL | FEATURE_RX_PARALLEL_PWM | FEATURE_RX_PPM);
    }

    if (feature(self->config, FEATURE_RX_SERIAL)) {
        featureClear(self->config, FEATURE_RX_PARALLEL_PWM | FEATURE_RX_MSP | FEATURE_RX_PPM);
    }

    if (feature(self->config, FEATURE_RX_PARALLEL_PWM)) {
        featureClear(self->config, FEATURE_RX_SERIAL | FEATURE_RX_MSP | FEATURE_RX_PPM);
    }

    // The retarded_arm setting is incompatible with pid_at_min_throttle because full roll causes the craft to roll over on the ground.
    // The pid_at_min_throttle implementation ignores yaw on the ground, but doesn't currently ignore roll when retarded_arm is enabled.
    if (self->config->arm.retarded_arm && self->config->mixer.pid_at_min_throttle) {
        self->config->mixer.pid_at_min_throttle = 0;
    }


#ifdef STM32F10X
    // avoid overloading the CPU on F1 targets when using gyro sync and GPS.
    if (self->config->imu.gyroSync && self->config->imu.gyroSyncDenominator < 2 && feature(self->config, FEATURE_GPS)) {
        self->config->imu.gyroSyncDenominator = 2;
    }
#endif

/*
// TODO: ledstrip config
#if defined(LED_STRIP)
#if (defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2))
    if (feature(FEATURE_SOFTSERIAL) && (
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
    if ((WS2811_DMA_TC_FLAG == TRANSPONDER_DMA_TC_FLAG) && feature(FEATURE_TRANSPONDER) && feature(FEATURE_LED_STRIP)) {
        featureClear(FEATURE_LED_STRIP);
    }
#endif
#endif // LED_STRIP
*/
#if defined(CC3D)
#if defined(DISPLAY) && defined(USE_UART3)
    if (feature(self->config, FEATURE_DISPLAY) && doesConfigurationUsePort(SERIAL_PORT_UART3)) {
        featureClear(self->config, FEATURE_DISPLAY);
    }
#endif

#if defined(SONAR) && defined(USE_SOFTSERIAL1)
    if (feature(self->config, FEATURE_SONAR) && feature(self->config, FEATURE_SOFTSERIAL)) {
        featureClear(self->config, FEATURE_SONAR);
    }
#endif

#if defined(SONAR) && defined(USE_SOFTSERIAL1) && defined(RSSI_ADC_GPIO)
    // shared pin
    if ((feature(self->config, FEATURE_SONAR) + feature(self->config, FEATURE_SOFTSERIAL) + feature(self->config, FEATURE_RSSI_ADC)) > 1) {
        featureClear(self->config, FEATURE_SONAR);
        featureClear(self->config, FEATURE_SOFTSERIAL);
        featureClear(self->config, FEATURE_RSSI_ADC);
    }
#endif
#endif // CC3D

#if defined(COLIBRI_RACE)
    serialConfig()->portConfigs[0].functionMask = FUNCTION_MSP;
    if (feature(self->config, FEATURE_RX_SERIAL)) {
        serialConfig()->portConfigs[2].functionMask = FUNCTION_RX_SERIAL;
    }
#endif

	/*
	TODO
    if (!isSerialConfigValid(serialConfig())) {
        PG_RESET_CURRENT(serialConfig);
    }
	*/
#if defined(USE_VCP)
    self->config->serial.portConfigs[0].functionMask = FUNCTION_MSP;
#endif
}

void ninja_config_load(struct ninja *self){
    rx_suspend_signal(&self->rx);

    // Sanity check, read flash
	// TODO: load config
	/*
    if (!scanEEPROM(true)) {
        failureMode(FAILURE_INVALID_EEPROM_CONTENTS);
    }
*/
    //pgActivateProfile(getCurrentProfile());

	// TODO: need to set the control rate profile here 
    //setControlRateProfile(rateProfileSelection()->defaultRateProfileIndex);

    ninja_validate_config(self);

	// TODO: update control rates somehow here (currently we only use profile 0)
	//activateControlRateConfig();

    rc_adj_reset(&self->rc_adj);

#ifdef GPS
    gpsUsePIDs(&config_get_profile(self->config)->pid);
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

    rx_resume_signal(&self->rx);
}

void ninja_config_save(struct ninja *self){
    rx_suspend_signal(&self->rx);

	config_save(self->config, self->system);

    rx_resume_signal(&self->rx);
}

void ninja_config_reset(struct ninja *self){
	(void)self;
    //pgResetAll(MAX_PROFILE_COUNT);

    //setProfile(0);
    //pgActivateProfile(0);

	// TODO: all of this function needs to be a callback in order to work correctly
    //setControlRateProfile(0);

    featureClearAll(self->config);

    featureSet(self->config, DEFAULT_RX_FEATURE | FEATURE_FAILSAFE | FEATURE_BLACKBOX);
#ifdef DEFAULT_FEATURES
    featureSet(self->config, DEFAULT_FEATURES);
#endif

#ifdef BOARD_HAS_VOLTAGE_DIVIDER
    // only enable the VBAT feature by default if the board has a voltage divider otherwise
    // the user may see incorrect readings and unexpected issues with pin mappings may occur.
    featureSet(self->config, FEATURE_VBAT);
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
	// TODO: fix this
	/*
    PG_FOREACH_PROFILE(reg) {
        for (int i = 1; i < MAX_PROFILE_COUNT; i++) {
            memcpy(reg->address + i * pgSize(reg), reg->address, pgSize(reg));
        }
    }
	*/
	// TODO: fix this
    for (int i = 1; i < MAX_PROFILE_COUNT; i++) {
        config_get_profile_rw(self->config)->rate.profile_id = i % MAX_CONTROL_RATE_PROFILE_COUNT;
    }
}

void ninja_config_validate(struct ninja *self){
	(void)self;
	// TODO: validate and load config from eeprom
	/*
    if (isEEPROMContentValid()) {
        return;
    }
	ninja_config_reset(self);
    ninja_config_save(self);
	*/

}

// TODO: remove this kind of redundancy after refactoring
void ninja_config_save_and_beep(struct ninja *self){
    ninja_config_save(self);
    ninja_config_load(self);
    beeper_multi_beeps(&self->beeper, 1);
}

void ninja_config_change_profile(struct ninja *self, uint8_t profileIndex){
	(void)profileIndex;
    //setProfile(profileIndex);
    ninja_config_save(self);
    ninja_config_load(self);
}


