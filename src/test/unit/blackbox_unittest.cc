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
	#include "config/config.h"
	#include "common/ulink.h"
	#include "ninja.h"

	#include "blackbox.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

class BlackBoxTest : public ::testing::Test {
protected:
    virtual void SetUp() {
		mock_system_reset();
		config_reset(&config);
    }
	struct config_store config;
};

TEST_F(BlackBoxTest, PackingTest){
	struct blackbox blackbox;

	printf("blackbox state size (bytes): %lu\n", sizeof(struct blackbox_frame));
	blackbox_init(&blackbox, &config.data, mock_syscalls());
	
	struct blackbox_frame frame;
	struct blackbox_frame frame2;
	memset(&frame, 0, sizeof(frame));
	memset(&frame2, 0, sizeof(frame));

	frame.gyr[0] = 1;
	frame.gyr[1] = 2;
	frame.gyr[2] = 3;

	printf("gyro offset: %lu\n", offsetof(struct blackbox_frame, gyr));
	blackbox_write(&blackbox, &frame);

	EXPECT_EQ(0, mock_logger_pos);

	blackbox_flush(&blackbox);

	frame2.acc[0] = 4;
	frame2.acc[1] = 5;
	frame2.acc[2] = 6;

	blackbox_write(&blackbox, &frame2);
	blackbox_flush(&blackbox);

	size_t logger_pos = mock_logger_pos;
	blackbox_write(&blackbox, &frame2);
	blackbox_flush(&blackbox);

	// writing the same data twice should not result in any output
	EXPECT_EQ(logger_pos, mock_logger_pos);

	printf("logger %d: ", mock_logger_pos);
	fflush(stdout);
	for(size_t c = 0; c < mock_logger_pos; c++){
		printf("%02x ", mock_logger_data[c] & 0xff);
	}
	printf("\n");

	// parse back the data
	struct blackbox_frame parsed;
	size_t consumed = 0;
	memset(&parsed, 0, sizeof(parsed));

	EXPECT_EQ(1, blackbox_parse(mock_logger_data, mock_logger_pos, &parsed, &consumed));

	EXPECT_EQ(0, memcmp(&frame, &parsed, sizeof(frame)));

	EXPECT_EQ(1, blackbox_parse(mock_logger_data + consumed, mock_logger_pos - consumed, &parsed, &consumed));

	EXPECT_EQ(0, memcmp(&frame2, &parsed, sizeof(frame2)));

	// try to corrup some of the blackbox data
	mock_logger_data[2] = 0xfe;

	memset(&parsed, 0, sizeof(parsed));

	// parsing should succeed, but we should have second frame parsed now and not the first
	EXPECT_EQ(1, blackbox_parse(mock_logger_data, mock_logger_pos, &parsed, &consumed));
	EXPECT_NE(0, memcmp(&parsed, &frame, sizeof(frame)));
	EXPECT_EQ(0, memcmp(&parsed, &frame2, sizeof(frame2)));
	// parsing should fail since no more frames are available
	EXPECT_EQ(-1, blackbox_parse(mock_logger_data + consumed, mock_logger_pos - consumed, &parsed, &consumed));
	
}

