#pragma once

#include <stdint.h>
#include <stddef.h>

#define MAX_DELTA_SIZE (8 * 24)

struct delta {
	int8_t level0[255];
	int8_t level1[255];
	int8_t level2[255];
	uint8_t size;
};

void delta_init(struct delta *self, uint8_t size);
int16_t delta_encode(const void *_prev_state, const void *_cur_state, size_t data_size, void *_output, size_t out_size);
int16_t delta_decode(const void *_delta, void *_output, size_t out_size);
int16_t delta_pack(struct delta *self, void *dst, const void *bytes, uint8_t size);
void delta_unpack(struct delta *self, void *dst, const void *bytes, uint8_t size);

struct packer {
	uint8_t *buffer;
	uint8_t size;
	uint8_t tail;
};

void packer_init(struct packer *self, void *buffer, uint8_t size);
void packer_reset(struct packer *self);
uint8_t pack_byte(struct packer *self, uint8_t byte);
uint8_t pack_u32(struct packer *self, uint32_t value);
uint8_t pack_s32(struct packer *self, int32_t value);
uint8_t pack_s32_array(struct packer *self, const int32_t *array, uint8_t count);
uint8_t pack_s16_array(struct packer *self, const int16_t *array, uint8_t count);

uint8_t pack_8u8(struct packer *self, const uint8_t *u8, uint8_t size);
uint8_t pack_8s16(struct packer *self, const int16_t *s16);

uint8_t pack_blob(struct packer *self, const void *_in, uint8_t insize);

uint8_t pack_s16(struct packer *self, int16_t value);
uint8_t pack_tag2_3s32(struct packer *self, const int32_t values[3]);
uint8_t pack_tag8_4s16(struct packer *self, const int16_t values[4]);
uint8_t pack_tag8_8svb(struct packer *self, const int32_t values[8], uint8_t valueCount);
uint8_t pack_u32_raw(struct packer *self, int32_t value);
uint8_t pack_f32(struct packer *self, float value);

struct unpacker {
	const uint8_t *buffer;
	uint8_t size;
	uint8_t head;
};

void unpacker_init(struct unpacker *self, const void *buffer, uint8_t size);
int16_t unpack_byte(struct unpacker *self);
uint32_t unpack_u32(struct unpacker *self);
int32_t unpack_s32(struct unpacker *self);
void unpack_s32_array(struct unpacker *self, int32_t *array, uint8_t count);
