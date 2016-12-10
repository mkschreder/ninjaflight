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

#include "drivers/config_flash.h"

#include "platform.h"

#include <string.h>
#include <errno.h>

extern uint8_t *__config_start;
extern uint8_t *__config_end;

#if !defined(FLASH_PAGE_SIZE)
# if defined(STM32F303xC)
#  define FLASH_PAGE_SIZE                 (0x800)
# elif defined(STM32F10X_MD)
#  define FLASH_PAGE_SIZE                 (0x400)
# elif defined(STM32F10X_HD)
#  define FLASH_PAGE_SIZE                 (0x800)
# elif defined(UNIT_TEST)
#  define FLASH_PAGE_SIZE                 (0x400)
# else
#  error "Flash page size not defined for target."
# endif
#endif

#define CONFIG_FLASH_SIZE (__config_end - __config_start)

void _clear_flags(void){
#if defined(STM32F303)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
#elif defined(STM32F10X)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
#elif defined(UNIT_TEST)
    // NOP
#else
# error "Unsupported CPU"
#endif
}

int flash_write(uintptr_t base, const void *data, size_t size){
	// refuse to write unaligned data
	if((void*)base >= (void*)(__config_start + CONFIG_FLASH_SIZE)) return -EINVAL;
	if((void*)(base + size) >= (void*)__config_end)
		size = (size_t)(__config_end - base);

	FLASH_Unlock();
	for(size_t c = 0; c < size >> 2; c++){
		uint32_t value = ((uint32_t*)data)[c];
		if(FLASH_ProgramWord(base + (c << 2), value) != FLASH_COMPLETE){
			FLASH_Lock();
			return c << 2;
		}
	}
	FLASH_Lock();
	return size;
}

int flash_erase_page(uintptr_t base){
	if((base % FLASH_PAGE_SIZE != 0) || (void*)base >= (void*)(__config_start + CONFIG_FLASH_SIZE)) return -EINVAL;

	FLASH_Unlock();
	if(FLASH_ErasePage(base) != FLASH_COMPLETE){
		FLASH_Lock();
		return -EIO;
	}
	FLASH_Lock();
	return 0;
}

int flash_read(uintptr_t base, void *data, size_t size){
	if((void*)base >= (void*)(__config_start + CONFIG_FLASH_SIZE)) return -EINVAL;
	if((void*)(base + size) >= (void*)__config_end)
		size = (size_t)(__config_end - base);
	memcpy(data, (void*)base, size);
	return size;
}

size_t flash_get_page_size(void){
	return FLASH_PAGE_SIZE;
}

size_t flash_get_num_pages(void){
	return CONFIG_FLASH_SIZE;
}

