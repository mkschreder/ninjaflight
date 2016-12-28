//#include <memory.h>
#include <stdlib.h>
#include <string.h>

#include "packer.h"

static uint8_t __attribute__((__unused__)) rle_encode (const void *_in, uint8_t insize, void *_out, uint8_t outsize){
	uint8_t *in = (uint8_t*)_in;
	uint8_t *out = (uint8_t*)_out;
	int block_len, block_char, c;
	uint8_t inpos = 0;
	uint8_t outpos = 0;
	block_len = 0;
	block_char = -1;
	for(;;){
		c = in[inpos++];
		if (c == block_char && block_len < 255){
			block_len++;
		} else {
			if (block_len) {
				out[outpos++] = block_len & 0xff;
				out[outpos++] = block_char & 0xff;
				if(outpos > (outsize - 2)) break;
			}
			block_len = 1;
			block_char = c;
		}
		if(inpos == insize) break;
	}
	if (block_len && outpos < (outsize - 2)){
		out[outpos++] = block_len & 0xff;
		out[outpos++] = block_char & 0xff;
	}
	return outpos;
}

static __attribute__((__unused__)) uint8_t rle_decode (const void *_in, uint8_t insize, void *_out, uint8_t outsize){
	int i, len, c;
	int inpos = 0, outpos = 0;
	uint8_t *in = (uint8_t*)_in;
	uint8_t *out = (uint8_t*)_out;
	for(;;){
		len = in[inpos++];
		if (inpos == insize) return outpos;
		c = in[inpos++];
		if (inpos == insize) return outpos;
		for(i = 0; i < len; i++) {
			out[outpos++] = c;
			if(outpos == outsize) return outpos;
		}
	}
}

void delta_init(struct delta *self, uint8_t size){
	memset(self, 0, sizeof(*self));
	self->size = size;
}

#include <stdio.h>

/**
 * Recursively pack changes between two equal size blocks into a delta buffer.
 */
static int16_t _delta_encode_block(const void *_prev_state, const void *_cur_state, size_t data_size, void *_output, size_t out_size, size_t block_size){
	// get char pointers to the data
	const uint8_t *prev = (const uint8_t*)_prev_state;
	const uint8_t *cur = (const uint8_t*)_cur_state;
	uint8_t *output = (uint8_t*)_output;
	// store header so that we can update it after we have encoded a block
	uint8_t *header = output++;
	// data size may be zero for whatever reason. In that case we do not write any delta data
	if(!data_size) return 0;
	// it is necessary to make sure we reset the delta bits
	*header = 0;
	// last iteration will have block size one
	if(block_size > 1){
		uint8_t bit = 0;
		//printf("encode %lu bytes with blocksize %lu\n", data_size, block_size);
		for(size_t c = 0; c < data_size; c += block_size){
			size_t len = data_size;
			if(block_size < data_size) len = block_size;
			// last block can be less than a full block size so we need to account for that
			if((c + block_size) > data_size) len = data_size - c;
			//printf("do block recursively at %lu of len %lu at output %p\n", c, len, output);
			// descend down into successively smaller blocks sizes while keeping track of which blocks have changes in them
			int16_t r = _delta_encode_block(
				prev + c,
				cur + c,
				len,
				output,
				out_size - 1, // -1 is for the header
				block_size / 8 // blocks size is always subdivided such that we can full in 8 bits
			);
			// if the encoding has failed we just return -1
			if(r < 0) return -1;
			else if(r > 0) {
				*header |= (1 << bit);
				out_size -= r;
				output += r;
			}
			bit++;
		}
	} else {
		//printf("encode %lu bytes at %p\n", data_size, cur);
		// this is the deepest iteration where we simply encode 8 bytes (or less)
		for(size_t c = 0; c < data_size; c++){
			if(cur[c] != prev[c]){
				*output++ = cur[c];
				*header |= (1 << c);
			}
		}
	}
	// if no blocks are dirty here then we don't write anything
	if(*header == 0) return 0;
	// return number of bytes written into the output
	return output - (uint8_t*)_output;
}

int16_t _delta_decode_block(const void *_delta, void *_output, size_t out_size, size_t block_size){
	const uint8_t *delta = (const uint8_t*)_delta;
	const uint8_t *header = delta++;
	if(block_size == 0) return 0;
	if(*header == 0) return 0;
	for(uint8_t bit = 0; bit < 8; bit++){
		if(((*header) & (1 << bit)) == 0) continue;
		size_t offset = (bit * block_size);
		uint8_t *output = ((uint8_t*)_output) + offset;
		//printf("decoding block %d of size %lu to offset %lu\n", bit, block_size, offset);
		if(block_size > 1){
			size_t len = block_size / 8;
			int16_t r = _delta_decode_block(delta, output, out_size - offset, len);
			if(r > 0){
				delta += r;
			}
		} else {
			if(offset > out_size){
				//printf("ERROR: trying to place byte at %lu block size %lu\n", offset, block_size);
				return 0;
			}
			*output = *delta++;
		}
	}
	return delta - header;
}

/**
 * Pack a delta buffer with changes between two blocks of memory. Input size must be at most 64 * 8 = 512 bytes long.
 */
int16_t delta_encode(const void *_prev_state, const void *_cur_state, size_t data_size, void *_output, size_t out_size){
	return _delta_encode_block(_prev_state, _cur_state, data_size, _output, out_size, 64);
}

int16_t delta_decode(const void *_delta, void *_output, size_t out_size){
	return _delta_decode_block(_delta, _output, out_size, 64);
}

/**
 * ZigZag encoding maps all values of a signed integer into those of an unsigned integer in such
 * a way that numbers of small absolute value correspond to small integers in the result.
 *
 * (Compared to just casting a signed to an unsigned which creates huge resulting numbers for
 * small negative integers).
 */
static uint32_t zigzag_encode(int32_t value){
	return (uint32_t)((value << 1) ^ (value >> 31));
}

static int32_t zigzag_decode(uint32_t n){
	return (n >> 1) ^ (-(n & 1));
}

/**
 * Cast the in-memory representation of the given float directly to an int.
 *
 * This is useful for printing the hex representation of a float number (which is considerably cheaper
 * than a full decimal float formatter, in both code size and output length).
 */
static uint32_t castFloatBytesToInt(float f){
    union floatConvert_t {
        float f;
        uint32_t u;
    } floatConvert;

    floatConvert.f = f;

    return floatConvert.u;
}

void packer_init(struct packer *self, void *buffer, uint8_t size){
	memset(self, 0, sizeof(*self));
	self->buffer = (uint8_t*)buffer;
	self->size = size;
}

void packer_reset(struct packer *self){
	self->tail = 0;
}

/**
 * Pack a byte into the packer buffer
 */
uint8_t pack_byte(struct packer *self, uint8_t byte){
	if(self->tail > (self->size - 1)) return 0;
	self->buffer[self->tail++] = byte;
	return 1;
}

/**
 * Write an unsigned integer to the buffer using variable byte encoding.
 */
uint8_t pack_u32_vb(struct packer *self, uint32_t value){
	uint8_t res = 0;
	//While this isn't the final byte (we can only write 7 bits at a time)
	while (value > 127) {
		res += pack_byte(self, (uint8_t) (value | 0x80)); // Set the high bit to mean "more bytes follow"
		value >>= 7;
	}
	res += pack_byte(self, value);
	return res;
}

/**
 * Write a signed integer to the blackbox serial port using ZigZig and variable byte encoding.
 */
uint8_t pack_s32_vb(struct packer *self, int32_t value){
	//ZigZag encode to make the value always positive
	return pack_u32_vb(self, zigzag_encode(value));
}

uint8_t pack_s32_array(struct packer *self, const int32_t *array, uint8_t count){
	uint8_t res = 0;
	for (int i = 0; i < count; i++) {
		res += pack_s32(self, array[i]);
	}
	return res;
}

uint8_t pack_s16_array(struct packer *self, const int16_t *array, uint8_t count){
	uint8_t res = 0;
	for (int i = 0; i < count; i++) {
		res += pack_s32(self, array[i]);
	}
	return res;
}

uint8_t pack_8u8(struct packer *self, const uint8_t *bytes, uint8_t size){
	uint8_t res = 0;
	uint8_t nz = 0;
	if(size > 8) size = 8;
	for(uint8_t c = 0; c < size; c++){
		if(bytes[c])
			nz |= (1 << c);
	}
	res += pack_byte(self, nz);
	for(uint8_t c = 0; c < size; c++)
		if(bytes[c])
			res += pack_byte(self, bytes[c]);
	return res;
}

uint8_t pack_blob(struct packer *self, const void *_in, uint8_t insize){
	struct packer pack;
	uint8_t buffer[0xff];
	uint8_t *in = (uint8_t*)_in;
	uint8_t res = 0;
	const uint8_t MAXINPUT = 8 * 24;
	if(insize > MAXINPUT) insize = MAXINPUT;
	packer_init(&pack, buffer, sizeof(buffer));
	for(unsigned c = 0; c < insize; c+=8){
		uint8_t len = ((insize - c) > 8)?8:(insize - c);
		res += pack_8u8(&pack, in + c, len);
	}
	insize = res;
	res = 0;
	for(unsigned c = 0; c < insize; c+=8){
		uint8_t len = ((insize - c) > 8)?8:(insize - c);
		res += pack_8u8(self, buffer + c, len);
	}
	return res;
}

uint8_t pack_8s16(struct packer *self, const int16_t *values){
	// pack as two 8u8 spans
	uint8_t res = 0;
	res += pack_8u8(self, (const uint8_t*)values, 8);
	res += pack_8u8(self, (const uint8_t*)(values + 4), 8);
	return res;
}

/**
 * Write a 2 bit tag followed by 3 signed fields of 2, 4, 6 or 32 bits
 */
uint8_t pack_tag2_3s32(struct packer *self, const int32_t values[3]){
	static const int NUM_FIELDS = 3;
	uint8_t res = 0;
	//Need to be enums rather than const ints if we want to switch on them (due to being C)
	enum {
		BITS_2  = 0,
		BITS_4  = 1,
		BITS_6  = 2,
		BITS_32 = 3
	};

	enum {
		BYTES_1  = 0,
		BYTES_2  = 1,
		BYTES_3  = 2,
		BYTES_4  = 3
	};

	int x;
	int selector = BITS_2, selector2;

	/*
	 * Find out how many bits the largest value requires to encode, and use it to choose one of the packing schemes
	 * below:
	 *
	 * Selector possibilities
	 *
	 * 2 bits per field  ss11 2233,
	 * 4 bits per field  ss00 1111 2222 3333
	 * 6 bits per field  ss11 1111 0022 2222 0033 3333
	 * 32 bits per field sstt tttt followed by fields of various byte counts
	 */
	for (x = 0; x < NUM_FIELDS; x++) {
		//Require more than 6 bits?
		if (values[x] >= 32 || values[x] < -32) {
			selector = BITS_32;
			break;
		}

		//Require more than 4 bits?
		if (values[x] >= 8 || values[x] < -8) {
			 if (selector < BITS_6) {
				 selector = BITS_6;
			 }
		} else if (values[x] >= 2 || values[x] < -2) { //Require more than 2 bits?
			if (selector < BITS_4) {
				selector = BITS_4;
			}
		}
	}

	switch (selector) {
		case BITS_2:
			res += pack_byte(self, (selector << 6) | ((values[0] & 0x03) << 4) | ((values[1] & 0x03) << 2) | (values[2] & 0x03));
		break;
		case BITS_4:
			res += pack_byte(self, (selector << 6) | (values[0] & 0x0F));
			res += pack_byte(self, (values[1] << 4) | (values[2] & 0x0F));
		break;
		case BITS_6:
			res += pack_byte(self, (selector << 6) | (values[0] & 0x3F));
			res += pack_byte(self, (uint8_t)values[1]);
			res += pack_byte(self, (uint8_t)values[2]);
		break;
		case BITS_32:
			/*
			 * Do another round to compute a selector for each field, assuming that they are at least 8 bits each
			 *
			 * Selector2 field possibilities
			 * 0 - 8 bits
			 * 1 - 16 bits
			 * 2 - 24 bits
			 * 3 - 32 bits
			 */
			selector2 = 0;

			//Encode in reverse order so the first field is in the low bits:
			for (x = NUM_FIELDS - 1; x >= 0; x--) {
				selector2 <<= 2;

				if (values[x] < 128 && values[x] >= -128) {
					selector2 |= BYTES_1;
				} else if (values[x] < 32768 && values[x] >= -32768) {
					selector2 |= BYTES_2;
				} else if (values[x] < 8388608 && values[x] >= -8388608) {
					selector2 |= BYTES_3;
				} else {
					selector2 |= BYTES_4;
				}
			}

			//Write the selectors
			res += pack_byte(self, (selector << 6) | selector2);

			//And now the values according to the selectors we picked for them
			for (x = 0; x < NUM_FIELDS; x++, selector2 >>= 2) {
				switch (selector2 & 0x03) {
					case BYTES_1:
						res += pack_byte(self, values[x]);
					break;
					case BYTES_2:
						res += pack_byte(self, values[x]);
						res += pack_byte(self, values[x] >> 8);
					break;
					case BYTES_3:
						res += pack_byte(self, values[x]);
						res += pack_byte(self, values[x] >> 8);
						res += pack_byte(self, values[x] >> 16);
					break;
					case BYTES_4:
						res += pack_byte(self, values[x]);
						res += pack_byte(self, values[x] >> 8);
						res += pack_byte(self, values[x] >> 16);
						res += pack_byte(self, values[x] >> 24);
					break;
				}
			}
		break;
	}
	return res;
}

/**
 * Write an 8-bit selector followed by four signed fields of size 0, 4, 8 or 16 bits.
 */
uint8_t pack_tag8_4s16(struct packer *self, const int16_t values[4]){
	//Need to be enums rather than const ints if we want to switch on them (due to being C)
	enum {
		FIELD_ZERO  = 0,
		FIELD_4BIT  = 1,
		FIELD_8BIT  = 2,
		FIELD_16BIT = 3
	};

	uint8_t selector, buffer;
	int nibbleIndex;
	int x;
	uint8_t res = 0;

	selector = 0;
	//Encode in reverse order so the first field is in the low bits:
	for (x = 3; x >= 0; x--) {
		selector <<= 2;

		if (values[x] == 0) {
			selector |= FIELD_ZERO;
		} else if (values[x] < 8 && values[x] >= -8) {
			selector |= FIELD_4BIT;
		} else if (values[x] < 128 && values[x] >= -128) {
			selector |= FIELD_8BIT;
		} else {
			selector |= FIELD_16BIT;
		}
	}

	res += pack_byte(self, selector);

	nibbleIndex = 0;
	buffer = 0;
	for (x = 0; x < 4; x++, selector >>= 2) {
		switch (selector & 0x03) {
			case FIELD_ZERO:
				//No-op
			break;
			case FIELD_4BIT:
				if (nibbleIndex == 0) {
					//We fill high-bits first
					buffer = values[x] << 4;
					nibbleIndex = 1;
				} else {
					res += pack_byte(self, buffer | (values[x] & 0x0F));
					nibbleIndex = 0;
				}
			break;
			case FIELD_8BIT:
				if (nibbleIndex == 0) {
					res += pack_byte(self, values[x]);
				} else {
					//Write the high bits of the value first (mask to avoid sign extension)
					res += pack_byte(self, buffer | ((values[x] >> 4) & 0x0F));
					//Now put the leftover low bits into the top of the next buffer entry
					buffer = values[x] << 4;
				}
			break;
			case FIELD_16BIT:
				if (nibbleIndex == 0) {
					//Write high byte first
					res += pack_byte(self, values[x] >> 8);
					res += pack_byte(self, values[x]);
				} else {
					//First write the highest 4 bits
					res += pack_byte(self, buffer | ((values[x] >> 12) & 0x0F));
					// Then the middle 8
					res += pack_byte(self, values[x] >> 4);
					//Only the smallest 4 bits are still left to write
					buffer = values[x] << 4;
				}
			break;
		}
	}
	//Anything left over to write?
	if (nibbleIndex == 1) {
		res += pack_byte(self, buffer);
	}
	return res;
}

/**
 * Write `valueCount` fields from `values` to the Blackbox using signed variable byte encoding. A 1-byte header is
 * written first which specifies which fields are non-zero (so this encoding is compact when most fields are zero).
 *
 * valueCount must be 8 or less.
 */
uint8_t pack_tag8_8svb(struct packer *self, const int32_t *values, uint8_t valueCount){
	uint8_t header;
	uint8_t res = 0;
	int i;

	if (valueCount > 0) {
		//If we're only writing one field then we can skip the header
		if (valueCount == 1) {
			res += pack_s32(self, values[0]);
		} else {
			//First write a one-byte header that marks which fields are non-zero
			header = 0;

			// First field should be in low bits of header
			for (i = valueCount - 1; i >= 0; i--) {
				header <<= 1;

				if (values[i] != 0) {
					header |= 0x01;
				}
			}

			res += pack_byte(self, header);

			for (i = 0; i < valueCount; i++) {
				if (values[i] != 0) {
					res += pack_s32(self, values[i]);
				}
			}
		}
	}
	return res;
}

/** Write unsigned integer **/
uint8_t pack_s32(struct packer *self, int32_t value){
	pack_byte(self, value & 0xFF);
	pack_byte(self, (value >> 8) & 0xFF);
	pack_byte(self, (value >> 16) & 0xFF);
	pack_byte(self, (value >> 24) & 0xFF);
	return 4;
}

uint8_t pack_s16(struct packer *self, int16_t value){
	pack_byte(self, value & 0xFF);
	pack_byte(self, (value >> 8) & 0xFF);
	return 2;
}

/** Write float value in the integer form **/
uint8_t pack_f32(struct packer *self, float value){
	return pack_s32(self, castFloatBytesToInt(value));
}

void unpacker_init(struct unpacker *self, const void *buffer, uint8_t size){
	memset(self, 0, sizeof(*self));
	self->buffer = (const uint8_t*)buffer;
	self->size = size;
}

int16_t unpack_byte(struct unpacker *self){
	if(self->head >= self->size) return -1;
	return self->buffer[self->head++];
}

uint32_t unpack_u32(struct unpacker *self){
	uint32_t res = 0;
	uint8_t bits = 0;
	do {
		uint32_t val = self->buffer[self->head++];
		res |= (val & 0x7f) << bits;
		// if high bit not set then no more bytes
		if((val & 0x80) == 0)
			break;
		bits += 7;
	} while(self->head < self->size);
	return res;
}

int32_t unpack_s32(struct unpacker *self){
	return zigzag_decode(unpack_u32(self));
}

void unpack_s32_array(struct unpacker *self, int32_t *array, uint8_t count){
	for(unsigned c = 0; c < count; c++){
		array[c] = unpack_s32(self);
	}
}
/*
uint8_t unpack_blob(struct unpacker *self, void *_out, uint8_t outsize){
	struct packer pack;
	uint8_t buffer[255];
	uint8_t *in = (uint8_t*)_in;
	uint8_t res = 0;
	unpacker_init(&unpack, buffer, sizeof(buffer));
	for(uint8_t c = 0; c < insize; c+=8){
		uint8_t len = ((insize - c) > 8)?8:(insize - c);
		res += pack_8u8(&pack, in + c, len);
	}
	insize = res;
	res = 0;
	for(uint8_t c = 0; c < insize; c+=8){
		uint8_t len = ((insize - c) > 8)?8:(insize - c);
		res += pack_8u8(self, buffer + c, len);
	}
	return res;
}
*/


