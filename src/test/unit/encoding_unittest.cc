/*
 * This file is part of Ninjaflight.
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
#include <stdint.h>

extern "C" {
    #include "common/encoding.h"
	#include "common/packer.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"
/*
typedef struct zigzagEncodingExpectation_s {
    int32_t input;
    uint32_t expected;
} zigzagEncodingExpectation_t;

typedef struct floatToIntEncodingExpectation_s {
    float input;
    uint32_t expected;
} floatToIntEncodingExpectation_t;

TEST(EncodingTest, ZigzagEncodingTest)
{
    // given
    zigzagEncodingExpectation_t expectations[] = {
        { 0, 0},
        {-1, 1},
        { 1, 2},
        {-2, 3},
        { 2, 4},

        { 2147483646, 4294967292},
        {-2147483647, 4294967293},
        { 2147483647, 4294967294},
        {-2147483648, 4294967295},
    };
    int expectationCount = sizeof(expectations) / sizeof(expectations[0]);

    // expect

    for (int i = 0; i < expectationCount; i++) {
        zigzagEncodingExpectation_t *expectation = &expectations[i];

        EXPECT_EQ(expectation->expected, zigzagEncode(expectation->input));
    }
}

TEST(EncodingTest, FloatToIntEncodingTest)
{
    // given
    floatToIntEncodingExpectation_t expectations[] = {
        {0.0, 0x00000000},
        {2.0, 0x40000000}, // Exponent should be in the top bits
        {4.5, 0x40900000}
    };
    int expectationCount = sizeof(expectations) / sizeof(expectations[0]);

    // expect

    for (int i = 0; i < expectationCount; i++) {
        floatToIntEncodingExpectation_t *expectation = &expectations[i];

        EXPECT_EQ(expectation->expected, castFloatBytesToInt(expectation->input));
    }
}
*/
class PackerTest : public ::testing::Test {
protected:
    virtual void SetUp() {
		packer_init(&packer, buffer, sizeof(buffer));
		srand(time(NULL));
    }
	struct packer packer;
	char buffer[32];
};


TEST_F(PackerTest, BytePack){
	char buf[2];
	struct packer pack;
	packer_init(&pack, buf, sizeof(buf));
	EXPECT_EQ(1, pack_byte(&pack, 0x12));
	EXPECT_EQ(1, pack_byte(&pack, 0x34));
	EXPECT_EQ(0, pack_byte(&pack, 0x56));

	struct unpacker unpack;
	unpacker_init(&unpack, buf, sizeof(buf));
	EXPECT_EQ(0x12, unpack_byte(&unpack));
	EXPECT_EQ(0x34, unpack_byte(&unpack));
	EXPECT_EQ(-1, unpack_byte(&unpack));
}
/*
TEST_F(PackerTest, PackU32){
	char buf[64];
	struct packer pack;
	packer_init(&pack, buf, sizeof(buf));
	EXPECT_EQ(1, pack_u32(&pack, 0x71));
	EXPECT_EQ(2, pack_u32(&pack, 0x82));
	EXPECT_EQ(3, pack_u32(&pack, 0x7283));
	EXPECT_EQ(3, pack_u32(&pack, 0x8283));
	EXPECT_EQ(4, pack_u32(&pack, 0x848283));
	EXPECT_EQ(5, pack_u32(&pack, 0x86848283));
	EXPECT_EQ(5, pack_u32(&pack, 0xffffffff));

	printf("packed u32: ");
	for(uint8_t c = 0; c < pack.tail; c++){
		printf("%02x", pack.buffer[c]);
	} printf("\n");

	struct unpacker unpack;
	unpacker_init(&unpack, buf, sizeof(buf));
	EXPECT_EQ(0x71, unpack_u32(&unpack));
	EXPECT_EQ(0x82, unpack_u32(&unpack));
	EXPECT_EQ(0x7283, unpack_u32(&unpack));
	EXPECT_EQ(0x8283, unpack_u32(&unpack));
	EXPECT_EQ(0x848283, unpack_u32(&unpack));
	EXPECT_EQ(0x86848283, unpack_u32(&unpack));
	EXPECT_EQ(0xffffffff, unpack_u32(&unpack));
}

TEST_F(PackerTest, PackS32){
	char buf[64];
	struct packer pack;
	packer_init(&pack, buf, sizeof(buf));
	EXPECT_EQ(2, pack_s32(&pack, 0x71));
	EXPECT_EQ(2, pack_s32(&pack, 0x82));
	EXPECT_EQ(3, pack_s32(&pack, 0x7283));
	EXPECT_EQ(3, pack_s32(&pack, 0x8283));
	EXPECT_EQ(4, pack_s32(&pack, 0x848283));
	EXPECT_EQ(5, pack_s32(&pack, 0x86848283));
	EXPECT_EQ(1, pack_s32(&pack, 0xffffffff));

	printf("packed s32: ");
	for(uint8_t c = 0; c < pack.tail; c++){
		printf("%02x", pack.buffer[c]);
	} printf("\n");

	struct unpacker unpack;
	unpacker_init(&unpack, buf, sizeof(buf));
	EXPECT_EQ(0x71, unpack_s32(&unpack));
	EXPECT_EQ(0x82, unpack_s32(&unpack));
	EXPECT_EQ(0x7283, unpack_s32(&unpack));
	EXPECT_EQ(0x8283, unpack_s32(&unpack));
	EXPECT_EQ(0x848283, unpack_s32(&unpack));
	EXPECT_EQ(0x86848283, unpack_s32(&unpack));
	EXPECT_EQ(0xffffffff, unpack_s32(&unpack));
}
TEST_F(PackerTest, PackS32Array){
	char buf[64];
	struct packer pack;
	packer_init(&pack, buf, sizeof(buf));
	int32_t values[] = {0x71, 0x82, 0x7283, 0x8283, 0x848283, (int32_t)0x86848283, -1};
	EXPECT_EQ(20, pack_s32_array(&pack, values, sizeof(values) / sizeof(values[0])));

	struct unpacker unpack;
	unpacker_init(&unpack, buf, sizeof(buf));
	int32_t result[sizeof(values)/sizeof(values[0])];
	unpack_s32_array(&unpack, result, sizeof(result)/sizeof(result[0]));
	for(unsigned c = 0; c < sizeof(result)/sizeof(result[0]); c++){
		EXPECT_EQ(values[c], result[c]);
	}
}
*/

TEST_F(PackerTest, Pack4s16){
	char buf[64];
	struct packer pack;
	packer_init(&pack, buf, sizeof(buf));
	struct {
		uint8_t count;
		int16_t val[4];
	} expect[] = {
		{ .count = 1, .val = {0, 0, 0, 0} },
		{ .count = 3, .val = {128, 0, 0, 0} },
		{ .count = 5, .val = {128, 128, 0, 0} },
		{ .count = 5, .val = {0, 0, 500, 500} },
		{ .count = 7, .val = {128, -500, 500, 0} },
		{ .count = 9, .val = {128, 128, 500, -500} },
		{ .count = 9, .val = {30000, 128, 512, -28000} },
	};

	for(unsigned c = 0; c < sizeof(expect) / sizeof(expect[0]); c++){
		EXPECT_EQ(expect[c].count, pack_tag8_4s16(&pack, expect[c].val));
	}
}
/*
TEST_F(PackerTest, Pack8s32delta){
	char buf[64];
	struct packer pack;
	packer_init(&pack, buf, sizeof(buf));
	struct {
		uint8_t count;
		int32_t val[8];
	} expect[] = {
		{ .count = 1, .val = {0, 0, 0, 0, 0, 0, 0, 0} },
		{ .count = 3, .val = {0, 0, 0, 200, 0, 0, 0, 0} },
		{ .count = 5, .val = {200, 0, 0, 200, 0, 0, 0, 0} },
		{ .count = 7, .val = {200, 0, 0, 200, 0, 0, 0, 200} },
		{ .count = 25, .val = {30000, 30000, 30000, 30000, -30000, 30000, 30000, -30000} },
	};

	for(unsigned c = 0; c < sizeof(expect) / sizeof(expect[0]); c++){
		EXPECT_EQ(expect[c].count, pack_tag8_8svb(&pack, expect[c].val, sizeof(expect[0].val) / sizeof(expect[0].val[0])));
	}
}
*/
static void print_array(const char *name, const void *_arr, size_t size){
	uint8_t *arr = (uint8_t*)_arr;
	printf("%s: ", name);
	for(size_t c = 0; c < size; c++){
		printf("%02x ", (arr[c] & 0xff));
	}
	printf("\n");
}

TEST_F(PackerTest, DeltaEncode){
	uint8_t prev_state[16], cur_state[sizeof(prev_state)];
	uint8_t output[sizeof(prev_state) * 2];
	memset(prev_state, 0, sizeof(prev_state));
	memset(cur_state, 0, sizeof(cur_state));
	memset(output, 0, sizeof(output));

	// both buffers are equal so output is a single null byte
	EXPECT_EQ(0, delta_encode(prev_state, cur_state, sizeof(prev_state), output, sizeof(output)));

	// make a small change
	cur_state[1] = 0x12;
	// we will have output of 0x01,0x01,0x02,0x12
	EXPECT_EQ(4, delta_encode(prev_state, cur_state, sizeof(prev_state), output, sizeof(output)));
	// first block of the 64 byte blocks
	EXPECT_EQ((1 << 0), output[0]);
	EXPECT_EQ((1 << 0), output[1]);
	// sencond byte of the first 8 byte block
	EXPECT_EQ((1 << 1), output[2]);
	// newly changed value
	EXPECT_EQ(cur_state[1], output[3]);
}

/**
 * Test decoding of a block
 */
TEST_F(PackerTest, DeltaDecode){
	uint8_t delta[] = { 0x01, 0x03, 0x02, 0x44, 0x03, 0xaa, 0xbb };
	uint8_t expect[10];
	uint8_t output[10];
	memset(expect, 0, sizeof(expect));
	memset(output, 0, sizeof(output));
	expect[1] = 0x44;
	expect[8] = 0xaa;
	expect[9] = 0xbb;

	delta_decode(delta, output, sizeof(output));
	for(size_t c = 0; c < sizeof(output); c++){
		EXPECT_EQ(expect[c], output[c]);
	}
}


/**
 * Test different size buffers using the delta packer
 */
TEST_F(PackerTest, PackSizes){
	for(int c = 1; c < 512; c++){
		//printf("testing size %d\n", c);
		//fflush(stdout);
		uint8_t *zero = (uint8_t*)malloc(c);
		uint8_t *buf = (uint8_t*)malloc(c);
		uint8_t *delta = (uint8_t*)malloc(c * 2);
		if(!zero || !buf || !delta) {
			perror("Memory allocation failed!");
			return;
		}
		// test different fill retios
		memset(zero, 0, c);
		memset(buf, 0, c);
		memset(delta, 0, c);
		for(int j = 0; j < c; j++){
			// modify every second byte
			if((j & 1) == 1) buf[j] = j;
		}
		// pack the delta
		int16_t size = delta_encode(zero, buf, c, delta, c * 2);
		//print_array("buffer: ", buf, c);
		//print_array("delta: ", delta, size);
		// double check
		EXPECT_EQ(size, delta_decode(delta, zero, c));
		for(int j = 0; j < c; j++){
			EXPECT_EQ(zero[j], buf[j]);
		}
		free(zero);
		free(buf);
		free(delta);
	}
}

/**
 * Test compression rates for different fill ratios for the delta packer
 */
TEST_F(PackerTest, FillRatios){
	for(int c = 8; c < 512; c+=8){
		//printf("testing size %d\n", c);
		//fflush(stdout);
		uint8_t *zero = (uint8_t*)malloc(c);
		uint8_t *buf = (uint8_t*)malloc(c);
		uint8_t *delta = (uint8_t*)malloc(c * 2);
		if(!zero || !buf || !delta) {
			perror("Memory allocation failed!");
			return;
		}
		// test different fill retios
		for(uint8_t ratio = 0; ratio <= 100; ratio += 10){
			memset(zero, 0, c);
			memset(buf, 0, c);
			memset(delta, 0, c);
			for(int j = 0; j < c; j++){
				if((rand() % 100) < ratio) buf[j] = j;
			}
			// pack the delta
			int16_t size = delta_encode(zero, buf, c, delta, c * 2);
			printf("size: %d, fill: %d, compression: %.2f\n", c, ratio, 100.0f * (double)size / c);
			//print_array("org: ", buf, c);
			//print_array("del: ", delta, size);
			// double check
			EXPECT_EQ(size, delta_decode(delta, zero, c));
			//print_array("dec: ", zero, c);
			for(int j = 0; j < c; j++){
				EXPECT_EQ(zero[j], buf[j]);
			}
		}
		free(zero);
		free(buf);
		free(delta);
	}
}

/*
TEST_F(PackerTest, Pack8x8delta){
	char buf[255];
	struct packer pack;
	packer_init(&pack, buf, sizeof(buf));
	char input[255];
	memset(input, 0, sizeof(input));
	EXPECT_EQ(4, pack_blob(&pack, input, sizeof(input)));
	packer_init(&pack, buf, sizeof(buf));
	for(unsigned c = 0; c < sizeof(input); c++){
		input[c] = rand();
	}
	EXPECT_EQ(255, pack_blob(&pack, input, sizeof(input)));

	memset(input, 0, sizeof(input));
	packer_init(&pack, buf, sizeof(buf));
	for(unsigned c = 0; c < sizeof(input); c+=2){
		input[c] = c + 1;
	}
	EXPECT_EQ(175, pack_blob(&pack, input, sizeof(input)));

	memset(input, 0, sizeof(input));
	packer_init(&pack, buf, sizeof(buf));
	for(unsigned c = 0; c < sizeof(input) / 2; c++){
		input[c] = c + 1;
	}
	EXPECT_EQ(163, pack_blob(&pack, input, sizeof(input)));
}
*/
