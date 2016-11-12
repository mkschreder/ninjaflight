/*
 * This file is part of Ninjaflight.
 *
 * Copyright: collective.
 * Cleanup: Martin Schröder <mkschreder.uk@gmail.com>
 * Original source from Cleanflight.
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

struct imu_config {
    // IMU configuration
    uint16_t looptime;                      // imu loop time in us
    uint8_t gyroSync;                       // Enable interrupt based loop
    uint8_t gyroSyncDenominator;            // Gyro sync Denominator
    uint16_t dcm_kp;                        // DCM filter proportional gain ( x 10000)
    uint16_t dcm_ki;                        // DCM filter integral gain ( x 10000)
    uint8_t small_angle;                    // Angle used for mag hold threshold.
    uint16_t max_angle_inclination;         // max inclination allowed in angle (level) mode. default 500 (50 degrees).
};


struct throttle_correction_config {
    uint16_t throttle_correction_angle;     // the angle when the throttle correction is maximal. in 0.1 degres, ex 225 = 22.5 ,30.0, 450 = 45.0 deg
    uint8_t throttle_correction_value;      // the correction that will be applied at throttle_correction_angle.
};

PG_DECLARE(struct imu_config, imuConfig);
PG_DECLARE_PROFILE(struct throttle_correction_config, throttleCorrectionConfig);


