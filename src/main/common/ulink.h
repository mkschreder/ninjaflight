/*
	Copyright (c) 2016 Martin Schröder <mkschreder.uk@gmail.com>

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <stdint.h>
#include <stdbool.h>

#define ULINK_MAX_FRAME_SIZE 512

struct ulink_frame {
	char buf[ULINK_MAX_FRAME_SIZE]; 
	size_t size; 
	uint8_t state; 
}; 

size_t ulink_pack_data(const void *data, size_t size, struct ulink_frame *frame); 
size_t ulink_parse_frame(const void *data, size_t size, struct ulink_frame *frame); 

void ulink_frame_init(struct ulink_frame *self); 
const void *ulink_frame_data(struct ulink_frame *self); 
size_t ulink_frame_size(struct ulink_frame *self); 
bool ulink_frame_valid(struct ulink_frame *self); 
size_t ulink_frame_to_buffer(struct ulink_frame *self, char *data, size_t max_size); 

#define ulink_stream_each_frame(buffer, buffer_size, frame) \
	for(int _rlen = 0; \
		(_rlen += ulink_parse_frame((buffer) + _rlen, (buffer_size) - _rlen, (frame))) && ulink_frame_valid(frame); \
		)
