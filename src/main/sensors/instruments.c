#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <platform.h>

#include "imu.h"
#include "instruments.h"

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
void ins_init(struct instruments *self,
	struct board_alignment_config *ba_conf,
	struct imu_config *imu_config,
	struct throttle_correction_config *thr_config,
	struct gyro_config *gyro_config,
	struct mag_config *mag_config,
	struct sensor_trims_config *sensor_trims,
	struct accelerometer_config *acc_config){
	memset(self, 0, sizeof(struct instruments));

	self->sensors = INS_USE_SENSOR_ACC | INS_USE_SENSOR_GYRO | INS_USE_SENSOR_MAG;

    board_alignment_init(&self->alignment, ba_conf);

	ins_acc_init(&self->acc,
		sensor_trims,
		acc_config
	);

	ins_gyro_init(&self->gyro,
		gyro_config
	);

	ins_mag_init(&self->mag,
		mag_config,
		sensor_trims
	);

    imu_init(&self->imu,
		imu_config,
		acc_config,
		thr_config
	);
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
    int32_t axis;
    int32_t accMagnitude = 0;

    for (axis = 0; axis < 3; axis++) {
        accMagnitude += (int32_t)raw[axis] * raw[axis];
    }

	if(accMagnitude == 0) return false;

    accMagnitude = accMagnitude * 100 / (sq((int32_t)self->acc.acc_1G));

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

void ins_update(struct instruments *self, float dt){
	// for now, fast converge if we are not calibrated.
	if(!ins_is_calibrated(self))
		imu_enable_fast_dcm_convergence(&self->imu, true);
	else
		imu_enable_fast_dcm_convergence(&self->imu, false);
	imu_update(&self->imu, dt);
}

void ins_set_gyro_scale(struct instruments *self, float scale){
	// TODO: both acc_1G and gyro scale need to be specified in only one place
	ins_gyro_set_scale(&self->gyro, scale);
	imu_set_gyro_scale(&self->imu, scale);
}

void ins_set_acc_scale(struct instruments *self, int16_t acc_1G){
	// TODO: both acc_1G and gyro scale need to be specified in only one place
	ins_acc_set_scale(&self->acc, acc_1G);
	imu_set_acc_scale(&self->imu, acc_1G);
	self->acc.acc_1G = acc_1G;
}

float ins_get_gyro_scale(struct instruments *self){
	return self->gyro.gyro_scale;
}
int16_t ins_get_acc_scale(struct instruments *self){
	return self->acc.acc_1G;
}

//! returns estimated altitude above sea level in cm
uint32_t ins_get_altitude_cm(struct instruments *self){
	(void)self;
	// TODO: calculate altitude here as we currently do in altitude hold code
	return 0;
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

