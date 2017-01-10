/*
 * This file is part of Ninjaflight.
 *
 * Copyright 2016-2017, Martin Schröder <mkschreder.uk@gmail.com>
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

/**
 * "fastloop" task that runs at high priority and syncs to the gyro. This task does the following:
 * - wait for gyro data ready interrupt
 * - read the gyro
 * - read acc if it is time
 * - read compass if it is time
 * - update dcm and attitude rate/angle estimate
 * - read control inputs (if user sent some)
 * - run pid controller
 * - run the mixer and write pwm values back to system
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "fastloop.h"

#ifdef SITL
#define ACC_READ_TIMEOUT 20000
#else
#define ACC_READ_TIMEOUT 2000
#endif

static void _task(void *param){
	struct fastloop *self = (struct fastloop*)param;
	int16_t _acc[3];
	int16_t _gyro[3];

	sys_micros_t loop_time = 0;
	self->next_acc_read_time = sys_micros(self->system) + ACC_READ_TIMEOUT;

	uint8_t dt_mul = 1; // this is used for accounting for skipped beats when we do not do a dcm update.

	while(true){
		// this is for calculating full update time
		sys_micros_t t_start = sys_micros(self->system);
		// this function will put this thread to sleep until the gyro interrupt fires.
		// It is important that the gyro interrupt is both configured and fires on each gyro sample
		// If it does not work and we do not sleep here then this thread will consume all cpu cycles and we will
		// likely waste a lot of cycles reading gyro when we don't have to.
		if(sys_gyro_sync(self->system) < 0){
#ifdef SITL
			vTaskDelay(3); // if gyro sync is not supported, introduce artificial delay
#else
			vTaskDelay(1); // if gyro sync is not supported, introduce artificial delay
#endif
		}

		// record time when we start handling the interrupt
		sys_micros_t t = sys_micros(self->system);

		// below are two distinct code paths that interleave with eachother at each gyro interrupt
		// - whenever it is time to update the acc (at 1khz) we do just that and
		// then yield. This allows other tasks to run. We can not achieve 8k
		// update on an spracingf3 due to i2c clock constraint that takes
		// ~140us to do the 6 value transaction from each sensor. But we can do
		// 4k just fine - EVEN without interrupt driven i2c.
		// - Second path is taken whenever it is time to update the gyro. It
		// runs the whole closed loop and outputs an update to the motors.
		if(t > self->next_acc_read_time){
			dt_mul++;
			if (sys_acc_read(self->system, _acc) == 0) {
				ins_process_acc(&self->ins, _acc[0], _acc[1], _acc[2]);
			}
			self->next_acc_read_time = t + ACC_READ_TIMEOUT;

			// read user command
			if(self->in_queue){
				struct fastloop_input in;
				memset(&in, 0, sizeof(in));
				if(xQueuePeek(self->in_queue, &in, 0)){
					anglerate_set_level_percent(&self->ctrl, in.level_pc[0], in.level_pc[1]);
					anglerate_input_user(&self->ctrl, in.roll, in.pitch, in.yaw);

					// put pid into open loop if requested
					if(in.mode & FL_OPEN){
						anglerate_set_openloop(&self->ctrl, true);
					} else {
						anglerate_set_openloop(&self->ctrl, false);
					}
					// soften pids if throttle is near zero
					if(in.throttle < self->config->rx.mincheck){
						anglerate_set_pid_axis_weight(&self->ctrl, FD_ROLL, 50);
						anglerate_set_pid_axis_weight(&self->ctrl, FD_PITCH, 50);
					} else {
						anglerate_set_pid_axis_weight(&self->ctrl, FD_ROLL, 100);
						anglerate_set_pid_axis_weight(&self->ctrl, FD_PITCH, 100);
					}

					mixer_input_command(&self->mixer, MIXER_INPUT_G0_THROTTLE, in.throttle);
					// center the RC input value around the RC middle value
					// by subtracting the RC middle value from the RC input value, we get:
					// data - middle = input
					// 2000 - 1500 = +500
					// 1500 - 1500 = 0
					// 1000 - 1500 = -500
					for(int c = 0; c < 8; c++){
						mixer_input_command(&self->mixer, MIXER_INPUT_GROUP_RC + c, in.rc[c]);
					}

					if(in.mode & FL_ARMED){
						mixer_enable_armed(&self->mixer, true);
					} else {
						mixer_enable_armed(&self->mixer, false);
					}
				}
			}
		} else {
			// this is dt used for the gyro updates to the dcm.
			// It is extremely important that we use gyro sample rate and not our loop rate
			// doing it any other way will introduce small errors. Why let them be there if we can eliminate them?
			// also account for loops we skip when reading the acc
			float dt = GYRO_RATE_DT * dt_mul;
			dt_mul = 1;

			// read gyro (this MUST be done each time interrupt fires because dcm calculations rely on the GYRO update rate, NOT looptime)
			if(sys_gyro_read(self->system, _gyro) == 0){
				ins_process_gyro(&self->ins, _gyro[0], _gyro[1], _gyro[2]);
				ins_update(&self->ins, dt);
			}

			anglerate_input_body_rates(&self->ctrl, ins_get_gyro_x(&self->ins), ins_get_gyro_y(&self->ins), ins_get_gyro_z(&self->ins));
			anglerate_input_body_angles(&self->ctrl, ins_get_roll_dd(&self->ins), ins_get_pitch_dd(&self->ins), ins_get_yaw_dd(&self->ins));
			anglerate_update(&self->ctrl, dt);

			mixer_set_throttle_range(&self->mixer, 1500, self->config->pwm_out.minthrottle, self->config->pwm_out.maxthrottle);

			// TODO: passthrough mode
			mixer_input_command(&self->mixer, MIXER_INPUT_G0_ROLL, anglerate_get_roll(&self->ctrl));
			mixer_input_command(&self->mixer, MIXER_INPUT_G0_PITCH, anglerate_get_pitch(&self->ctrl));
			mixer_input_command(&self->mixer, MIXER_INPUT_G0_YAW, -anglerate_get_yaw(&self->ctrl));

			mixer_update(&self->mixer);

			// make data available to other applications
			if(self->out_queue){
				struct fastloop_output out;
				out.loop_time = loop_time;
				out.gyr[0] = ins_get_gyro_x(&self->ins);
				out.gyr[1] = ins_get_gyro_y(&self->ins);
				out.gyr[2] = ins_get_gyro_z(&self->ins);
				out.acc[0] = ins_get_acc_x(&self->ins);
				out.acc[1] = ins_get_acc_y(&self->ins);
				out.acc[2] = ins_get_acc_z(&self->ins);
				out.roll = ins_get_roll_dd(&self->ins);
				out.pitch = ins_get_pitch_dd(&self->ins);
				out.yaw = ins_get_yaw_dd(&self->ins);
				for(int c = 0; c < 8; c++){
					out.motors[c] = mixer_get_motor_value(&self->mixer, c);
					out.servos[c] = mixer_get_servo_value(&self->mixer, c);
				}
				xQueueOverwrite(self->out_queue, &out);
			}

			// store the looptime
			loop_time = sys_micros(self->system) - t_start;
		}
	}
}

// TODO: remove this crap
extern sensor_align_e gyroAlign;
extern sensor_align_e accAlign;
extern sensor_align_e magAlign;

void fastloop_init(struct fastloop *self, const struct system_calls *system, const struct config *config){
	self->system = system;
	self->config = config;

	self->in_queue = xQueueCreate(1, sizeof(struct fastloop_input));
	self->out_queue = xQueueCreate(1, sizeof(struct fastloop_output));

	// TODO: sensor scale and alignment should be completely handled by the driver!
	ins_set_gyro_alignment(&self->ins, config->sensors.alignment.gyro_align);
	ins_set_acc_alignment(&self->ins, config->sensors.alignment.acc_align);
	ins_set_mag_alignment(&self->ins, config->sensors.alignment.mag_align);

	mixer_init(&self->mixer, self->config, &system->pwm);
	ins_init(&self->ins, self->config);
	anglerate_init(&self->ctrl, &self->ins, self->config);
}

void fastloop_write_controls(struct fastloop *self, const struct fastloop_input *in){
	if(self->in_queue) xQueueOverwrite(self->in_queue, in);
}

void fastloop_read_outputs(struct fastloop *self, struct fastloop_output *out){
	if(self->out_queue) xQueuePeek(self->out_queue, out, 5000);
}

void fastloop_start(struct fastloop *self){
	xTaskCreate(_task, "gyro", 1224 / sizeof(StackType_t), self, 4, NULL);
}

