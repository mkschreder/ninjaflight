#pragma once

#include "../config/imu.h"
#include "imu.h"
#include "acceleration.h"
#include "gyro.h"
#include "compass.h"
#include "boardalignment.h"
#include "barometer.h"

struct instruments {
	struct ins_acc acc;
	struct ins_gyro gyro;
	struct ins_mag mag;
	struct baro baro;
	struct imu imu;

	struct board_alignment alignment;
	sensor_align_e mag_align;
	sensor_align_e gyr_align;
	sensor_align_e acc_align;

	uint8_t sensors;

	const struct config *config;
};

void ins_init(struct instruments *self, const struct config *config);

void ins_start_gyro_calibration(struct instruments *self);
void ins_start_acc_calibration(struct instruments *self);
void ins_start_mag_calibration(struct instruments *self);

bool ins_is_calibrated(struct instruments *self);
void ins_process_gyro(struct instruments *self, int32_t x, int32_t y, int32_t z);
void ins_process_acc(struct instruments *self, int32_t x, int32_t y, int32_t z);
void ins_process_mag(struct instruments *self, int32_t x, int32_t y, int32_t z);
void ins_process_pressure(struct instruments *self, uint32_t pressure);

void ins_update(struct instruments *self, float dt);

static inline void ins_reset_imu(struct instruments *self) { imu_reset(&self->imu); }

static inline int16_t ins_get_acc_x(struct instruments *self){ return ins_acc_get_x(&self->acc); }
static inline int16_t ins_get_acc_y(struct instruments *self){ return ins_acc_get_y(&self->acc); }
static inline int16_t ins_get_acc_z(struct instruments *self){ return ins_acc_get_z(&self->acc); }

static inline int32_t ins_get_gyro_x(struct instruments *self){ return ins_gyro_get_x(&self->gyro); }
static inline int32_t ins_get_gyro_y(struct instruments *self){ return ins_gyro_get_y(&self->gyro); }
static inline int32_t ins_get_gyro_z(struct instruments *self){ return ins_gyro_get_z(&self->gyro); }

static inline int32_t ins_get_mag_x(struct instruments *self){ return ins_mag_get_x(&self->mag); }
static inline int32_t ins_get_mag_y(struct instruments *self){ return ins_mag_get_y(&self->mag); }
static inline int32_t ins_get_mag_z(struct instruments *self){ return ins_mag_get_z(&self->mag); }

static inline int16_t ins_get_roll_dd(struct instruments *self){ return imu_get_roll_dd(&self->imu); }
static inline int16_t ins_get_pitch_dd(struct instruments *self){ return imu_get_pitch_dd(&self->imu); }
static inline int16_t ins_get_yaw_dd(struct instruments *self){ return imu_get_yaw_dd(&self->imu); }

//! returns estimated altitude above sea level in cm
uint32_t ins_get_altitude_cm(struct instruments *self);

//! returns vertical speed in cm/s
int16_t ins_get_vertical_speed_cms(struct instruments *self);

static inline void ins_set_gyro_alignment(struct instruments *self, sensor_align_e align) { self->gyr_align = align; }
static inline void ins_set_acc_alignment(struct instruments *self, sensor_align_e align) { self->acc_align = align; }
static inline void ins_set_mag_alignment(struct instruments *self, sensor_align_e align) { self->mag_align = align; }

static inline void ins_set_gyro_filter_hz(struct instruments *self, uint16_t hz) { ins_gyro_set_filter_hz(&self->gyro, hz); }
