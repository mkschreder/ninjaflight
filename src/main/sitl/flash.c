#include <stdint.h>
#include <stdbool.h>
#include <memory.h>

#include "platform.h"

uint8_t __config_start[8192] __attribute__((aligned(1024)));
uint8_t __config_end[1]; // TODO

/*
static const int Capacity = 8192;
static const int PageSize = 1024;

int64_t wroteTo;
bool unlocked;

int erases;
int writes;

static uint8_t *data(void) { return __config_start; }
static void init(void)
{
	memset(data(), 0xAE, Capacity);
}
static int64_t toOffset(uint32_t address) 
{
	int32_t offset = address - (uint32_t)(uintptr_t)__config_start;
	return offset;
}
static bool inBounds(uint32_t address, int count) 
{
	uint32_t offset = toOffset(address);
	return offset >= 0 && (offset + count) < Capacity;
}

static FLASH_Status erase(uint32_t address)
{
	memset(data() + toOffset(address), 0xFF, PageSize);
	erases++;

	return FLASH_COMPLETE;
}

static FLASH_Status program(uint32_t address, uint32_t value)
{
	uint32_t offset = toOffset(address);

	for (uint i = 0; i < sizeof(value); i++) {
		data()[offset + i] = (uint8_t)value;
		value >>= 8;
	}
	wroteTo = std::max(wroteTo, (int64_t)(offset + sizeof(value)));
	writes++;

	return FLASH_COMPLETE;
}

void FLASH_Unlock(void)
{
}

void FLASH_Lock(void)
{
}

FLASH_Status FLASH_ErasePage(uint32_t address)
{
    return mockFlash.erase(address);
}

FLASH_Status FLASH_ProgramWord(uint32_t address, uint32_t data)
{
    return mockFlash.program(address, data);
}
*/
