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
#include <math.h>

#include <platform.h>
#include "build_config.h"

#include "common/maths.h"
#include "common/axis.h"

#include "config/config.h"
#include "config/feature.h"
#include "config/sensors.h"

#include "drivers/sonar_hcsr04.h"

#include "sensors/battery.h"
#include "sensors/sonar.h"

// Sonar measurements are in cm, a value of SONAR_OUT_OF_RANGE indicates sonar is not in range.
// Inclination is adjusted by imu
//    float baro_cf_vel;                      // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity)
//    float baro_cf_alt;                      // apply CF to use ACC for height estimation

#ifdef SONAR
/*
int16_t sonarMaxRangeCm;
int16_t sonarMaxAltWithTiltCm;
int16_t sonarCfAltCm; // Complimentary Filter altitude
STATIC_UNIT_TESTED int16_t sonarMaxTiltDeciDegrees;
float sonarMaxTiltCos;

static int32_t calculatedAltitude;
*/
// TODO: this is being abused elsewhere. Make it static
#if 0
const struct sonar_hardware *sonarGetHardwareConfiguration(currentSensor_e  currentMeterType); 
const struct sonar_hardware *sonarGetHardwareConfiguration(currentSensor_e  currentMeterType)
{
#if defined(SONAR_PWM_TRIGGER_PIN)
    static const struct sonar_hardware const sonarPWM = {
        .trigger_pin = SONAR_PWM_TRIGGER_PIN,
        .trigger_gpio = SONAR_PWM_TRIGGER_GPIO,
        .echo_pin = SONAR_PWM_ECHO_PIN,
        .echo_gpio = SONAR_PWM_ECHO_GPIO,
        .exti_line = SONAR_PWM_EXTI_LINE,
        .exti_pin_source = SONAR_PWM_EXTI_PIN_SOURCE,
        .exti_irqn = SONAR_PWM_EXTI_IRQN
    };
#endif
#if !defined(UNIT_TEST)
    static const struct sonar_hardware sonarRC = {
        .trigger_pin = SONAR_TRIGGER_PIN,
        .trigger_gpio = SONAR_TRIGGER_GPIO,
        .echo_pin = SONAR_ECHO_PIN,
        .echo_gpio = SONAR_ECHO_GPIO,
        .exti_line = SONAR_EXTI_LINE,
        .exti_pin_source = SONAR_EXTI_PIN_SOURCE,
        .exti_irqn = SONAR_EXTI_IRQN
    };
#endif
#if defined(SONAR_PWM_TRIGGER_PIN)
    // If we are using softserial, parallel PWM or ADC current sensor, then use motor pins 5 and 6 for sonar, otherwise use RC pins 7 and 8
    if (feature(FEATURE_SOFTSERIAL)
            || feature(FEATURE_RX_PARALLEL_PWM )
            || (feature(FEATURE_CURRENT_METER) && currentMeterType == CURRENT_SENSOR_ADC)) {
        return &sonarPWM;
    } else {
        return &sonarRC;
    }
#elif defined(SONAR_TRIGGER_PIN)
    UNUSED(currentMeterType);
    return &sonarRC;
#elif defined(UNIT_TEST)
    UNUSED(currentMeterType);
    return 0;
#else
#error Sonar not defined for target
#endif
}

#endif

#if 0
// TODO: sonar should be a sensor implemented in the system code
struct sonar default_sonar; 

void sonar_init(struct sonar *self){
    hcsr04_init(&self->dev);
    self->cf_alt_cm = self->dev.max_range_cm / 2;
    self->max_tilt_deci_degrees =  self->detection_cone_extended_deci_degrees / 2;
    self->max_tilt_cos = cos_approx(self->max_tilt_deci_degrees / 10.0f * RAD);
    self->max_alt_with_tilt = self->max_range_cm * self->max_tilt_cos;
}

#define DISTANCE_SAMPLES_MEDIAN 5

static int32_t applySonarMedianFilter(int32_t newSonarReading)
{
    static int32_t sonarFilterSamples[DISTANCE_SAMPLES_MEDIAN];
    static int currentFilterSampleIndex = 0;
    static bool medianFilterReady = false;
    int nextSampleIndex;

    if (newSonarReading > SONAR_OUT_OF_RANGE) // only accept samples that are in range
    {
        nextSampleIndex = (currentFilterSampleIndex + 1);
        if (nextSampleIndex == DISTANCE_SAMPLES_MEDIAN) {
            nextSampleIndex = 0;
            medianFilterReady = true;
        }

        sonarFilterSamples[currentFilterSampleIndex] = newSonarReading;
        currentFilterSampleIndex = nextSampleIndex;
    }
    if (medianFilterReady)
        return quickMedianFilter5(sonarFilterSamples);
    else
        return newSonarReading;
}

void sonar_update(struct sonar *self)
{
    hcsr04_start_reading(self);
}

/**
 * Get the last distance measured by the sonar in centimeters. When the ground is too far away, SONAR_OUT_OF_RANGE is returned.
 */
int32_t sonar_read(struct sonar *self)
{
    hcsr04_get_distance(self);
    if (self->distance > self->max_range_cm)
        self->distance = SONAR_OUT_OF_RANGE;

    return applySonarMedianFilter(self->distance);
}

/**
 * Apply tilt correction to the given raw sonar reading in order to compensate for the tilt of the craft when estimating
 * the altitude. Returns the computed altitude in centimeters.
 *
 * When the ground is too far away or the tilt is too large, SONAR_OUT_OF_RANGE is returned.
 */
int32_t sonar_calc_altitude(struct sonar *self, float cosTiltAngle)
{
    // calculate sonar altitude only if the ground is in the sonar cone
    if (cosTiltAngle <= self->max_tilt_cos)
        self->altitude = SONAR_OUT_OF_RANGE;
    else
        // altitude = distance * cos(tiltAngle), use approximation
        self->altitude = self->distance * cosTiltAngle;
    return self->altitude;
}

/**
 * Get the latest altitude that was computed by a call to sonarCalculateAltitude(), or SONAR_OUT_OF_RANGE if sonarCalculateAltitude
 * has never been called.
 */
int32_t sonar_get_altitude(struct sonar *self)
{
    return self->altitude;
}
#endif
#endif
