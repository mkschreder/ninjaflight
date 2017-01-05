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

#include "sensor.h"

// standard gyro rate on which all dividers are based (8khz)
#define GYRO_STANDARD_RATE 8000
#define GYRO_RATE_DT (1.0f / (float)(GYRO_STANDARD_RATE / (self->config->imu.gyro_sample_div + 1)))

typedef enum {
    GYRO_NONE = 0,
    GYRO_DEFAULT,
    GYRO_MPU6050,
    GYRO_L3G4200D,
    GYRO_MPU3050,
    GYRO_L3GD20,
    GYRO_MPU6000,
    GYRO_MPU6500,
    GYRO_FAKE
} gyroSensor_e;

// Type of accelerometer used/detected
typedef enum {
    ACC_DEFAULT = 0,
    ACC_NONE = 1,
    ACC_ADXL345 = 2,
    ACC_MPU6050 = 3,
    ACC_MMA8452 = 4,
    ACC_BMA280 = 5,
    ACC_LSM303DLHC = 6,
    ACC_MPU6000 = 7,
    ACC_MPU6500 = 8,
    ACC_FAKE = 9,
} accelerationSensor_e;

#define ACC_MAX  ACC_FAKE

typedef struct gyro_s {
    sensorGyroInitFuncPtr init;                             // initialize function
    sensorReadFuncPtr read;                                 // read 3 axis data function
    sensorReadFuncPtr temperature;                          // read temperature if available
    sensorIsDataReadyFuncPtr isDataReady;                   // check if sensor has new readings
	int (*sync)(void);
    float scale;                                            // scalefactor
} gyro_t;

typedef struct acc_s {
    sensorAccInitFuncPtr init;                              // initialize function
    sensorReadFuncPtr read;                                 // read 3 axis data function
    uint16_t acc_1G;
    char revisionCode;                                      // a revision code for the sensor, if known
} acc_t;

extern gyro_t gyro;
extern acc_t acc;


