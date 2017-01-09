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

#include <stdlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

extern "C" {
	#include "common/ulink.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"


static void print_buffer(const char *name, const void *_buf, size_t size){
	const char *buf = (const char*)_buf;
	printf("%s: ", name); 
	for(size_t c = 0; c < size; c++){
		printf("%02x ", (unsigned char)buf[c] & 0xff); 
	}
	printf("\n"); 
}

TEST(UlinkTest, UlinkBasicTest){
	int len = 0; 
	struct ulink_frame rx_frame, tx_frame; 

	ulink_frame_init(&rx_frame); 
	ulink_frame_init(&tx_frame); 

	char in_buffer[64]; 
	char tx_buffer[64]; 
	char rx_buffer[64]; 
	
	const char *data1 = "\x01\x00\x7d\x7e"; 
	const char *data2 = "\x7e\x7e\x01\x02"; 

	// pack two packets into a transmit buffer, test that it comes out as we expect and then try to parse out the data. 
	memcpy(in_buffer, data1, 4); 	
	EXPECT_EQ(ulink_pack_data(in_buffer, 4, &tx_frame), 4); 
	EXPECT_EQ(ulink_frame_size(&tx_frame), 9); 
	EXPECT_EQ(ulink_frame_to_buffer(&tx_frame, tx_buffer, sizeof(tx_buffer)), 9); 
	
	memcpy(in_buffer, data2, 4); 
	EXPECT_EQ(ulink_pack_data(in_buffer, 4, &tx_frame), 4); 
	EXPECT_EQ(ulink_frame_to_buffer(&tx_frame, tx_buffer + 9, sizeof(tx_buffer) - 9), 9); 

	print_buffer("packed: ", tx_buffer, 18); 

	memcpy(rx_buffer, "\x01\x00\x7d\x5d\x7d\x5e\x8c\x80\x7e\x7d\x5e\x7d\x5e\x01\x02\x61\xf8\x7e", 18); 
	EXPECT_EQ(memcmp(tx_buffer, rx_buffer, len), 0); 

	memset(rx_buffer, 0, sizeof(rx_buffer)); 

	int sizes[2] = { 4, 4 }; 
	const char *datas[2] = { data1, data2 }; 
	const char **data = datas; 
	int *size = sizes;  	
	int valid_frames = 0; 
	ulink_stream_each_frame(tx_buffer, 5, &rx_frame) {
		print_buffer("rx: ", ulink_frame_data(&rx_frame), *size); 
		EXPECT_EQ(*size, ulink_frame_size(&rx_frame)); 
		EXPECT_EQ(0, memcmp(ulink_frame_data(&rx_frame), *data++, *size)); 
		size++; 
		valid_frames++; 
	}

	ulink_stream_each_frame(tx_buffer + 5, 13, &rx_frame) {
		print_buffer("rx: ", ulink_frame_data(&rx_frame), *size); 
		EXPECT_EQ(*size, ulink_frame_size(&rx_frame)); 
		EXPECT_EQ(0, memcmp(ulink_frame_data(&rx_frame), *data++, *size)); 
		size++; 
		valid_frames++; 
	}
	
	EXPECT_EQ(valid_frames, 2); 

	// try to corrupt the frames
	valid_frames = 0; 
	uint8_t tmp = tx_buffer[4]; 
	tx_buffer[4] = 0x7e; 
	ulink_stream_each_frame(tx_buffer, 18, &rx_frame) {
		print_buffer("rx: ", ulink_frame_data(&rx_frame), ulink_frame_size(&rx_frame)); 
		valid_frames++; 
	}
	tx_buffer[4] = tmp; 
	EXPECT_EQ(1, valid_frames); 

	tx_buffer[6] = 0; 
	valid_frames = 0; 
	ulink_stream_each_frame(tx_buffer, 18, &rx_frame) {
		print_buffer("rx: ", ulink_frame_data(&rx_frame), ulink_frame_size(&rx_frame)); 
		valid_frames++; 
	}
	EXPECT_EQ(1, valid_frames); 
}
