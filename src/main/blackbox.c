/*
 * New blackbox logger implementation for Ninjaflight
 *
 * Copyright (c) 2016 Martin Schröder <mkschreder.uk@gmail.com>
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
 * This is a self-contained blackbox application that implements functionality
 * for regularly sending system state to a variety of different destinations
 * (for example flash or serial port).
 *
 * Blackbox runs as a medium priority task that wakes up at a regular interval
 * (smallest wakeup interval is 1ms). It then polls various system services for
 * their published state and combines all state into one blackbox state. This
 * state is then delta-compressed and written to destination of choice.
 */
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <platform.h>
#include "config/config.h"

#include "common/packer.h"
#include "common/pt.h"
#include "common/ulink.h"

#include "fastloop.h"
#include "ninja.h"

#include "blackbox.h"

#include <FreeRTOS.h>
#include <task.h>

#define STATIC_ASSERT(COND,MSG) typedef char static_assertion_##MSG[(COND)?1:-1]

void blackbox_init(struct blackbox *self, const struct config * config, const struct system_calls *system){
	memset(self, 0, sizeof(*self));
	self->cur_frame = self->buffers[0];
	self->prev_frame = self->buffers[1];
	self->state_queue = xQueueCreate(1, sizeof(struct blackbox_frame));
	self->config = config;
	self->system = system;
}

void blackbox_write(struct blackbox *self, const struct blackbox_frame *state){
	xQueueOverwrite(self->state_queue, state);
}

void blackbox_flush(struct blackbox *self){
	if(xQueueReceive(self->state_queue, self->cur_frame, portMAX_DELAY) == pdFALSE) return;

	int16_t size = delta_encode(self->prev_frame, self->cur_frame, sizeof(struct blackbox_frame), self->delta_buffer, sizeof(self->delta_buffer));

	if(size > 0){
		struct ulink_frame frame;
		ulink_frame_init(&frame);
		ulink_pack_data(self->delta_buffer, size, &frame);

		// write all data until we are done
		size_t idx = 0;
		while(idx < ulink_frame_size(&frame)){
			int ret = sys_logger_write(self->system, (char*)ulink_frame_data(&frame) + idx, ulink_frame_size(&frame) - idx);
			if(ret <= 0){
				// we failed. Maybe set an error flag.
				break;
			}
			idx += ret;
		}
	}

	// swap buffers
	uint8_t *tmp = self->cur_frame;
	self->cur_frame = self->prev_frame;
	self->prev_frame = tmp;
}

int blackbox_parse(const void *data, size_t size, struct blackbox_frame *out, size_t *consumed){
	struct ulink_frame frame;
	ulink_frame_init(&frame);
	size_t fsize = ulink_parse_frame(data, size, &frame);
	*consumed = fsize;
	if(ulink_frame_size(&frame) == 0) return -1;
	if(delta_decode(ulink_frame_data(&frame), out, sizeof(*out)) > 0) return 1;
	return -1;
}

void _blackbox_task(void *param){
	struct blackbox *self = (struct blackbox*)param;
	(void)self;
	while(true){
		// flush must sleep on incoming queue in order for this task not to hang the system
		blackbox_flush(self);
	}
}

void blackbox_start(struct blackbox *self){
	(void)self;
	xTaskCreate(_blackbox_task, "bbox", 1224 / sizeof(StackType_t), self, 3, NULL);
}
