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

#define ACC_READ_TIMEOUT 2000

uint32_t fastloop_time = 0;
void mpu_set_rate(void);
static __attribute__((unused)) void _task(void *param){
	struct fastloop *self = (struct fastloop*)param;
	int16_t _acc[3];
	int16_t _gyro[3];

	sys_micros_t last_t = sys_micros(self->system);
	self->next_acc_read_time = last_t + ACC_READ_TIMEOUT;

	vTaskDelay(30);
	mpu_set_rate();

	while(true){
		sys_gyro_sync(self->system);

		sys_micros_t t = sys_micros(self->system);

		float dt = (t - last_t) * 1e-6f;
	
		// read acc
		if(t > self->next_acc_read_time){
			if (sys_acc_read(self->system, _acc) == 0) {
				ins_process_acc(&self->ins, _acc[0], _acc[1], _acc[2]);
			}
			self->next_acc_read_time = t + ACC_READ_TIMEOUT;
		} else {
			// read gyro
			if(sys_gyro_read(self->system, _gyro) == 0){
				ins_process_gyro(&self->ins, _gyro[0], _gyro[1], _gyro[2]);
				ins_update(&self->ins, dt);
			}
		}
		
		// read user command
		if(self->in_queue){
			struct fastloop_input in;
			memset(&in, 0, sizeof(in));
			if(xQueuePeek(self->in_queue, &in, 0)){
				anglerate_set_level_percent(&self->ctrl, in.level_pc[0], in.level_pc[1]);
				anglerate_input_user(&self->ctrl, in.roll, in.pitch, in.yaw);
				mixer_input_command(&self->mixer, MIXER_INPUT_G0_THROTTLE, in.throttle);
				// center the RC input value around the RC middle value
				// by subtracting the RC middle value from the RC input value, we get:
				// data - middle = input
				// 2000 - 1500 = +500
				// 1500 - 1500 = 0
				// 1000 - 1500 = -500
				for(int c = 0; c < 8; c++){
					mixer_input_command(&self->mixer, MIXER_INPUT_GROUP_RC, in.rc[c]);
				}
			}
		}

		anglerate_input_body_rates(&self->ctrl, ins_get_gyro_x(&self->ins), ins_get_gyro_y(&self->ins), ins_get_gyro_z(&self->ins));
		anglerate_input_body_angles(&self->ctrl, ins_get_roll_dd(&self->ins), ins_get_pitch_dd(&self->ins), ins_get_yaw_dd(&self->ins));
		anglerate_update(&self->ctrl, dt);

		mixer_set_throttle_range(&self->mixer, 1500, self->config->pwm_out.minthrottle, self->config->pwm_out.maxthrottle);

		// write output queue
		if(self->out_queue){
			struct fastloop_output out;
			out.loop_time = t - last_t;
			out.gyr[0] = ins_get_gyro_x(&self->ins);
			out.gyr[1] = ins_get_gyro_y(&self->ins);
			out.gyr[2] = ins_get_gyro_z(&self->ins);
			out.acc[0] = ins_get_acc_x(&self->ins);
			out.acc[1] = ins_get_acc_y(&self->ins);
			out.acc[2] = ins_get_acc_z(&self->ins);
			out.roll = ins_get_roll_dd(&self->ins);
			out.pitch = ins_get_pitch_dd(&self->ins);
			out.yaw = ins_get_yaw_dd(&self->ins);

			xQueueOverwrite(self->out_queue, &out);
		}

		// TODO: flight modes
		//if (FLIGHT_MODE(PASSTHRU_MODE)) {
			// Direct passthru from RX
			//mixer_input_command(&self->mixer, MIXER_INPUT_G0_ROLL, rcCommand[ROLL]);
			//mixer_input_command(&self->mixer, MIXER_INPUT_G0_PITCH, rcCommand[PITCH]);
			//mixer_input_command(&self->mixer, MIXER_INPUT_G0_YAW, rcCommand[YAW]);
		//} else {
			// Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
			mixer_input_command(&self->mixer, MIXER_INPUT_G0_ROLL, anglerate_get_roll(&self->ctrl));
			mixer_input_command(&self->mixer, MIXER_INPUT_G0_PITCH, anglerate_get_pitch(&self->ctrl));
			mixer_input_command(&self->mixer, MIXER_INPUT_G0_YAW, -anglerate_get_yaw(&self->ctrl));
		//}
		mixer_update(&self->mixer);

		fastloop_time = sys_micros(self->system) - t;
		last_t = t;
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
	ins_set_gyro_alignment(&self->ins, gyroAlign);
	ins_set_acc_alignment(&self->ins, accAlign);
	ins_set_mag_alignment(&self->ins, magAlign);

	mixer_init(&self->mixer, self->config, &system->pwm);
	ins_init(&self->ins, self->config);
	anglerate_init(&self->ctrl, &self->ins, self->config);

	anglerate_set_algo(&self->ctrl, config_get_profile(self->config)->pid.pidController);
}

void fastloop_write_controls(struct fastloop *self, const struct fastloop_input *in){
	if(self->in_queue) xQueueOverwrite(self->in_queue, in);
}

void fastloop_read_outputs(struct fastloop *self, struct fastloop_output *out){
	if(self->out_queue) xQueuePeek(self->out_queue, out, 10);
}

void fastloop_start(struct fastloop *self){
	xTaskCreate(_task, "gyro", 1224 / sizeof(StackType_t), self, 4, NULL);
}

