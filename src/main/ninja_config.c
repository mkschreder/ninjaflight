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


static void ninja_validate_config(struct ninja *self){
	(void)self;
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

void ninja_config_load(struct ninja *self){
    rx_suspend_signal(&self->rx);

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

    rc_adj_reset(&self->rc_adj);

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

    rx_resume_signal(&self->rx);
}

void ninja_config_save(struct ninja *self){
    rx_suspend_signal(&self->rx);

    writeConfigToEEPROM();

    rx_resume_signal(&self->rx);
}

void ninja_config_reset(struct ninja *self){
	(void)self;
    pgResetAll(MAX_PROFILE_COUNT);

    setProfile(0);
    pgActivateProfile(0);

	// TODO: all of this function needs to be a callback in order to work correctly
    //setControlRateProfile(0);

    //parseRcChannels("AETR1234", rxConfig());

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
	// TODO: fix this
    for (int i = 1; i < MAX_PROFILE_COUNT; i++) {
        configureRateProfileSelection(i, i % MAX_CONTROL_RATE_PROFILE_COUNT);
    }
}

void ninja_config_validate(struct ninja *self){
    if (isEEPROMContentValid()) {
        return;
    }
	ninja_config_reset(self);
    ninja_config_save(self);

}

// TODO: remove this kind of redundancy after refactoring
void ninja_config_save_and_beep(struct ninja *self){
    ninja_config_save(self);
    ninja_config_load(self);
    beeperConfirmationBeeps(1);
}

void ninja_config_change_profile(struct ninja *self, uint8_t profileIndex){
    setProfile(profileIndex);
    ninja_config_save(self);
    ninja_config_load(self);
}

