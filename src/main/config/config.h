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

#pragma once

#define MAX_PROFILE_COUNT 3
#define ONESHOT_FEATURE_CHANGED_DELAY_ON_BOOT_MS 1500

#include "parameter_group.h"

#include "blackbox/blackbox.h"
PG_DECLARE(blackboxConfig_t, blackboxConfig);

#include "flight/anglerate.h"
PG_DECLARE_PROFILE(struct pid_config, pidProfile);

#include "sensors/sensors.h"
PG_DECLARE(sensorSelectionConfig_t, sensorSelectionConfig);
PG_DECLARE(sensorAlignmentConfig_t, sensorAlignmentConfig);
PG_DECLARE(sensorTrims_t, sensorTrims);

#include "sensors/acceleration.h"
PG_DECLARE_PROFILE(accelerometerConfig_t, accelerometerConfig);

#include "flight/rate_profile.h"
#define MAX_CONTROL_RATE_PROFILE_COUNT 3
PG_DECLARE_ARR(struct rate_config, MAX_CONTROL_RATE_PROFILE_COUNT, controlRateProfiles);
PG_DECLARE_PROFILE(rateProfileSelection_t, rateProfileSelection);

#include "flight/imu.h"
PG_DECLARE(struct imu_config, imuConfig);
PG_DECLARE_PROFILE(struct throttle_correction_config, throttleCorrectionConfig);

#include "flight/mixer.h"
PG_DECLARE_ARR(struct motor_mixer, MAX_SUPPORTED_MOTORS, customMotorMixer);
PG_DECLARE(struct mixer_config, mixerConfig);
PG_DECLARE(struct motor_3d_config, motor3DConfig);

PG_DECLARE(struct tilt_config, tiltConfig);

#include "flight/altitudehold.h"
PG_DECLARE(airplaneConfig_t, airplaneConfig);

#include "flight/failsafe.h"
PG_DECLARE(failsafeConfig_t, failsafeConfig);

#include "flight/gtune.h"
#ifdef GTUNE
PG_DECLARE_PROFILE(gtuneConfig_t, gtuneConfig);
#endif

#include "io/motor_and_servo.h"
PG_DECLARE(motorAndServoConfig_t, motorAndServoConfig);
PG_DECLARE_ARR(struct servo_mixer, MAX_SERVO_RULES, customServoMixer);
PG_DECLARE_PROFILE(struct servo_profile, servoProfile);

#include "io/gps.h"
PG_DECLARE(gpsConfig_t, gpsConfig);

#include "flight/navigation.h"
PG_DECLARE_PROFILE(gpsProfile_t, gpsProfile);

#include "io/gimbal.h"
PG_DECLARE_PROFILE(gimbalConfig_t, gimbalConfig);

#include "rx/rx.h"
PG_DECLARE(rxConfig_t, rxConfig);

PG_DECLARE_ARR(rxFailsafeChannelConfig_t, MAX_SUPPORTED_RC_CHANNEL_COUNT, failsafeChannelConfigs);
PG_DECLARE_ARR(rxChannelRangeConfiguration_t, NON_AUX_CHANNEL_COUNT, channelRanges);

#include "io/rc_adjustments.h"
PG_DECLARE_PROFILE(adjustmentProfile_t, adjustmentProfile);

PG_DECLARE_PROFILE(modeActivationProfile_t, modeActivationProfile);
PG_DECLARE(armingConfig_t, armingConfig);

#include "io/transponder_ir.h"
PG_DECLARE(transponderConfig_t, transponderConfig);

#include "drivers/pwm_rx.h"
PG_DECLARE(pwmRxConfig_t, pwmRxConfig);

#include "sensors/barometer.h"
#ifdef BARO
PG_DECLARE_PROFILE(barometerConfig_t, barometerConfig);
#endif

#include "sensors/battery.h"
PG_DECLARE(struct battery_config, batteryConfig);

#include "sensors/boardalignment.h"
PG_DECLARE(struct board_alignment_config, boardAlignment);

#include "sensors/compass.h"
PG_DECLARE_PROFILE(compassConfig_t, compassConfig);

#include "sensors/gyro.h"
PG_DECLARE(gyroConfig_t, gyroConfig);

#include "telemetry/telemetry.h"
PG_DECLARE(telemetryConfig_t, telemetryConfig);

#include "telemetry/frsky.h"
PG_DECLARE(frskyTelemetryConfig_t, frskyTelemetryConfig);

#include "telemetry/hott.h"
PG_DECLARE(hottTelemetryConfig_t, hottTelemetryConfig);

// SERIAL
#include "io/serial.h"
PG_DECLARE(serialConfig_t, serialConfig);

// LED_STRIP
#include "io/ledstrip.h"
#include "common/color.h"
PG_DECLARE_ARR(ledConfig_t, LED_MAX_STRIP_LENGTH, ledConfigs);
PG_DECLARE_ARR(hsvColor_t, LED_CONFIGURABLE_COLOR_COUNT, colors);
PG_DECLARE_ARR(modeColorIndexes_t, LED_MODE_COUNT, modeColors);
PG_DECLARE_ARR(specialColorIndexes_t, 1, specialColors);


void handleOneshotFeatureChangeOnRestart(void);

void initEEPROM(void);
void resetEEPROM(void);
void readEEPROM(void);
void writeEEPROM(void);
void ensureEEPROMContainsValidData(void);
void saveConfigAndNotify(void);

void changeProfile(uint8_t profileIndex);

uint8_t getCurrentControlRateProfile(void);
void changeControlRateProfile(uint8_t profileIndex);

bool canSoftwareSerialBeUsed(void);
