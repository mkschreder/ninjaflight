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

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <platform.h>
#include "config/config.h"

#include "common/packer.h"
#include "common/pt.h"

#include "blackbox.h"

#define STATIC_ASSERT(COND,MSG) typedef char static_assertion_##MSG[(COND)?1:-1]

void blackbox_init(struct blackbox *self, const struct config * config, const struct system_calls *system){
	memset(self, 0, sizeof(*self));
	self->cur_frame = self->buffers[0];
	self->prev_frame = self->buffers[1];
	self->config = config;
	self->system = system;
	PT_INIT(&self->flash_writer);
}
static PT_THREAD(_blackbox_stream_writer(struct blackbox *self)){
	PT_BEGIN(&self->flash_writer);

	while(true){
		PT_WAIT_UNTIL(&self->flash_writer, self->delta_ptr != NULL);
		int ret = sys_logger_write(self->system, self->delta_ptr, self->delta_size);
		if(ret > 0){
			self->delta_ptr += ret;
			self->delta_size -= ret;
		}
		if(self->delta_size == 0){
			self->delta_ptr = NULL;
		}
	}

	PT_END(&self->flash_writer);
}

void blackbox_write_frame(struct blackbox *self, const struct blackbox_frame *frame){
	// if previous frame still has not been written then we can not continue
	_blackbox_stream_writer(self);

	if(self->delta_ptr != NULL) return;

	// TODO: pack
	memcpy(self->cur_frame, frame, sizeof(*frame));

	int16_t size = delta_encode(self->prev_frame, self->cur_frame, sizeof(*frame), self->delta_buffer, sizeof(self->delta_buffer));

	if(size > 0) {
		self->delta_size = size;
		self->delta_ptr = self->delta_buffer;
	}

	// swap write buffers
	uint8_t *tmp = self->prev_frame;
	self->prev_frame = self->cur_frame;
	self->cur_frame = tmp;
}

bool blackbox_is_running(struct blackbox *self){
	(void)self;
	return true;
}


