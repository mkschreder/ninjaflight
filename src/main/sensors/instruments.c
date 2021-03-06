#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <platform.h>

#include "config/config.h"

#include "imu.h"
#include "barometer.h"
#include "instruments.h"

#include "system_calls.h"
/**
 * @defgroup Instruments
 * @{
 * @defgroup Instruments_Main
 * @{
 *
 * The purpose of this module is to maintain a filtered state of all flight
 * instruments that are available to the pilot. Instrumentation shall report
 * data in SI units or multiples of such (for example decidegrees for
 * represetnting fractions of degrees when not using floating point).
 *
 * Instruments object accepts raw sensor data as input and maintains estimated
 * state that can be directly used by other parts of the flight controller.
 * This module also handles sensor alignment.
 *
 * Currently supported input sensors include:
 * - Accelerometer
 * - Gyroscope
 * - Magnetometer
 *
 * Estimated quantities include:
 * - Orientation (quaternion and euler angles)
 * - Altitude
 * - Position
 * - Velocity
 */
enum {
	INS_USE_SENSOR_GYRO		= (1 << 0),
	INS_USE_SENSOR_ACC		= (1 << 1),
	INS_USE_SENSOR_MAG		= (1 << 2)
};

//! Initializes all instruments using specified configuration
void ins_init(struct instruments *self, const struct config *config){
	memset(self, 0, sizeof(struct instruments));
	
	self->config = config;
	self->sensors = INS_USE_SENSOR_ACC | INS_USE_SENSOR_GYRO | INS_USE_SENSOR_MAG;

	baro_init(&self->baro, config);

    board_alignment_init(&self->alignment, &self->config->alignment);

	ins_acc_init(&self->acc,
		&self->config->sensors.trims,
		&config_get_profile(self->config)->acc
	);

	ins_gyro_init(&self->gyro,
		&self->config->gyro
	);

	ins_mag_init(&self->mag,
		&config_get_profile(self->config)->mag,
		&self->config->sensors.trims
	);

    imu_init(&self->imu, self->config);
}

bool ins_is_calibrated(struct instruments *self){
	return ins_gyro_is_calibrated(&self->gyro) &&
		((self->sensors & INS_USE_SENSOR_ACC) && ins_acc_is_calibrated(&self->acc)) &&
		((self->sensors & INS_USE_SENSOR_MAG) && ins_mag_is_calibrated(&self->mag));
#if 0
#ifdef BARO
    if (sensors(SENSOR_BARO) && !isBaroCalibrationComplete()) {
        return true;
    }
#endif

    // Note: compass calibration is handled completely differently, outside of the main loop, see f.CALIBRATE_MAG

    return (!isAccelerationCalibrationComplete() && sensors(SENSOR_ACC)) || (!isGyroCalibrationComplete());
#endif
}

void ins_start_acc_calibration(struct instruments *self){
	ins_acc_calibrate(&self->acc);
}

void ins_start_gyro_calibration(struct instruments *self){
	ins_gyro_calibrate(&self->gyro);
}

void ins_start_mag_calibration(struct instruments *self){
	ins_mag_start_calibration(&self->mag);
}

void ins_process_gyro(struct instruments *self, int32_t x, int32_t y, int32_t z){
	int32_t raw[3] = {x, y, z};
	board_alignment_rotate_vector(&self->alignment, raw, raw, self->gyr_align);
	ins_gyro_process_sample(&self->gyro, raw[X], raw[Y], raw[Z]);
    imu_input_gyro(&self->imu, self->gyro.gyroADC[X], self->gyro.gyroADC[Y], self->gyro.gyroADC[Z]);
}

static bool _acc_magnitude_in_cent_g(struct instruments *self, int32_t raw[3], uint16_t min_cent, uint16_t max_cent){
	(void)self;
    int32_t axis;
    int32_t accMagnitude = 0;

	// scale the values a little so that they will not overflow
    for (axis = 0; axis < 3; axis++) {
        accMagnitude += (int32_t)(raw[axis] >> 3) * (raw[axis] >> 3); 
    }

	if(accMagnitude == 0) return false;

    accMagnitude = accMagnitude * 100 / (sq((int32_t)SYSTEM_ACCEL_1G >> 3));

    // Accept accel readings only in range 0.90g - 1.10g
    return (min_cent < accMagnitude) && (accMagnitude < max_cent);
}

void ins_process_acc(struct instruments *self, int32_t x, int32_t y, int32_t z){
	int32_t raw[3] = {x, y, z};
	board_alignment_rotate_vector(&self->alignment, raw, raw, self->acc_align);
	ins_acc_process_sample(&self->acc, raw[X], raw[Y], raw[Z]);
	
	raw[0] = ins_acc_get_x(&self->acc);
	raw[1] = ins_acc_get_y(&self->acc);
	raw[2] = ins_acc_get_z(&self->acc);
	// only update imu with samples that are in range 80%-120% of 1G
	// if we do not have this check then chances are that we would lose our attitude if quad is accelerating violently
	if(_acc_magnitude_in_cent_g(self, raw, 80, 120))
		imu_input_accelerometer(&self->imu, raw[0], raw[1], raw[2]);
}

void ins_process_mag(struct instruments *self, int32_t x, int32_t y, int32_t z){
	int32_t raw[3] = {x, y, z};
	board_alignment_rotate_vector(&self->alignment, raw, raw, self->mag_align);
	ins_mag_process_sample(&self->mag, raw[X], raw[Y], raw[Z]);
	imu_input_magnetometer(&self->imu, ins_mag_get_x(&self->mag), ins_mag_get_y(&self->mag), ins_mag_get_z(&self->mag));
}

void ins_process_pressure(struct instruments *self, uint32_t pressure){
	baro_process_pressure(&self->baro, pressure);
}

void ins_update(struct instruments *self, float dt){
	// for now, fast converge if we are not calibrated.
	if(!ins_is_calibrated(self))
		imu_enable_fast_dcm_convergence(&self->imu, true);
	else
		imu_enable_fast_dcm_convergence(&self->imu, false);
	imu_update(&self->imu, dt);
	baro_update(&self->baro);
}

//! returns estimated altitude above sea level in cm
uint32_t ins_get_altitude_cm(struct instruments *self){
	return baro_get_altitude(&self->baro);
}

//! returns vertical speed in cm/s
int16_t ins_get_vertical_speed_cms(struct instruments *self){
	(void)self;
	// TODO: calculate vertical speed
	return 0;
}

/**
 * @}
 * @}
 */

