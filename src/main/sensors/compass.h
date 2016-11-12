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

// TODO: remove dependency on compass 
#include "drivers/compass.h"
#include "../common/axis.h"
#include "../config/sensors.h"
#include "../config/compass.h"

// Type of magnetometer used/detected
typedef enum {
    MAG_DEFAULT = 0,
    MAG_NONE = 1,
    MAG_HMC5883 = 2,
    MAG_AK8975 = 3,
    MAG_AK8963 = 4
} magSensor_e;

#define MAG_MAX  MAG_AK8963

void compassInit(void);
void updateCompass(flightDynamicsTrims_t *magZero);

void recalculateMagneticDeclination(void);

extern int32_t magADC[XYZ_AXIS_COUNT];

extern sensor_align_e magAlign;
extern mag_t mag;
extern float magneticDeclination;
