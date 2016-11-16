#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <platform.h>

#include "imu.h"
#include "instruments.h"

enum {
	INS_USE_SENSOR_GYRO		= (1 << 0),
	INS_USE_SENSOR_ACC		= (1 << 1)
};

void ins_init(struct instruments *self,
	struct board_alignment_config *ba_conf,
	struct imu_config *imu_config,
	struct throttle_correction_config *thr_config,
	struct gyro_config *gyro_config,
	struct accelerometer_config *acc_config,
	float gyro_scale,
	int32_t acc_1G){
	memset(self, 0, sizeof(struct instruments));

	self->sensors = INS_USE_SENSOR_ACC | INS_USE_SENSOR_GYRO;

    board_alignment_init(&self->alignment, ba_conf);

	ins_acc_init(&self->acc,
		acc_config,
		acc_1G
	);

	ins_gyro_init(&self->gyro,
		gyro_config,
		gyro_scale
	);

    imu_init(&self->imu,
		imu_config,
		acc_config,
		thr_config,
		gyro_scale,
		acc_1G
	);
}

bool ins_is_calibrated(struct instruments *self){
	return ins_gyro_is_calibrated(&self->gyro) && ((self->sensors & INS_USE_SENSOR_ACC) && ins_acc_is_calibrated(&self->acc));
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

void ins_process_gyro(struct instruments *self, int32_t x, int32_t y, int32_t z){
	int32_t raw[3] = {x, y, z};
	board_alignment_rotate_vector(&self->alignment, raw, raw, self->gyr_align);
	ins_gyro_process_sample(&self->gyro, raw[X], raw[Y], raw[Z]);
    imu_input_gyro(&self->imu, self->gyro.gyroADC[X], self->gyro.gyroADC[Y], self->gyro.gyroADC[Z]);
}

void ins_process_acc(struct instruments *self, int32_t x, int32_t y, int32_t z){
	int32_t raw[3] = {x, y, z};
	board_alignment_rotate_vector(&self->alignment, raw, raw, self->acc_align);
	ins_acc_process_sample(&self->acc, raw[X], raw[Y], raw[Z]);
	imu_input_accelerometer(&self->imu, self->acc.accADC[X], self->acc.accADC[Y], self->acc.accADC[Z]);
}

void ins_process_mag(struct instruments *self, int32_t x, int32_t y, int32_t z){
	int32_t raw[3] = {x, y, z};
	board_alignment_rotate_vector(&self->alignment, raw, raw, self->mag_align);
	ins_mag_process_sample(&self->mag, raw[X], raw[Y], raw[Z]);
}

void ins_update(struct instruments *self, float dt){
	// for now, fast converge if we are not calibrated.
	if(!ins_is_calibrated(self))
		imu_enable_fast_dcm_convergence(&self->imu, true);
	else
		imu_enable_fast_dcm_convergence(&self->imu, false);
	imu_update(&self->imu, dt);
}
