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
#include <stddef.h>

extern "C" {
    #include "platform.h"
    #include "build_config.h"
   
   #include "config/config.h"
    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/color.h"
    #include "common/utils.h"

    #include "flight/anglerate.h"

    #include "drivers/sensor.h"
    #include "drivers/timer.h"
    #include "drivers/accgyro.h"
    #include "drivers/compass.h"
    #include "drivers/pwm_rx.h"
    #include "drivers/serial.h"
	#include "drivers/config_flash.h"

    #include "flight/rate_profile.h"
    #include "io/rc_adjustments.h"
    #include "io/serial.h"
    #include "io/ledstrip.h"
    #include "io/transponder_ir.h"

    #include "sensors/acceleration.h"
    #include "sensors/barometer.h"
    #include "sensors/compass.h"
    #include "sensors/gyro.h"
    #include "sensors/battery.h"
    #include "sensors/boardalignment.h"

    #include "flight/mixer.h"
    #include "flight/navigation.h"
    #include "flight/failsafe.h"
    #include "flight/altitudehold.h"

    #include "telemetry/telemetry.h"
    #include "telemetry/frsky.h"
    #include "telemetry/hott.h"

    #include "config/config.h"
    #include "config/feature.h"
    #include "config/profile.h"
	#include "config/frsky.h"

    #include "platform.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

/*
 * For testing config we will actually use the low level flash eeprom writer driver just to make sure that it works as expected.
 */

// this one is defined in mock_flash.cc
//extern uint8_t *__config_start;

static int _eeprom_read(const struct system_calls_bdev *self, void *dst, uint16_t addr, size_t size){
	(void)self;
	flash_read(addr, dst, size);
	//printf("EEPROM read from %04x, size %lu\n", addr, size);
	fflush(stdout);
	return size;
}

static int _eeprom_erase_page(const struct system_calls_bdev *self, uint16_t addr){
	(void)self;
	flash_erase_page(addr);
	//printf("EEPROM erase at %04x\n", addr);
	fflush(stdout);
	return 0;
}

static int _eeprom_write(const struct system_calls_bdev *self, uint16_t addr, const void *data, size_t size){
	(void)self;
	flash_write(addr, data, size);
	//printf("EEPROM write to %04x, size %lu\n", addr, size);
	fflush(stdout);
	return size;
}

static void _eeprom_get_info(const struct system_calls_bdev *self, struct system_bdev_info *info){
	(void)self;
	info->page_size = 1024;
	info->num_pages = 8;
}

class ConfigTest : public ::testing::Test {
protected:
    virtual void SetUp() {
		mock_system_reset();
		config_reset(&config);
    }
	struct config_store config;
};

TEST_F(ConfigTest, Dummy){
	struct config_store store;
	printf("config size: %lu (0x%04x) bytes, %lu words\n", sizeof(struct config), (unsigned int)sizeof(struct config), sizeof(struct config)/2);
	// some code assumes config comes first followed by checksum
	EXPECT_EQ(0, offsetof(struct config_store, data));
	EXPECT_EQ(offsetof(struct config_store, crc), sizeof(struct config));
	// config must be word aligned
	EXPECT_EQ(0, sizeof(struct config) % sizeof(uint16_t));
	EXPECT_EQ(sizeof(store.crc), sizeof(uint16_t));
}

/**
 * @ingroup CONFIG
 * @page CONFIG
 *
 * - Config reset must set all fields in the config to appropriate defaults. If
 * some fields do not have defaults then this is an error!
 */
TEST_F(ConfigTest, TestConfigCoverage){
	uint8_t magic[] = {0xAA, 0xFF, 0x00, 0x77};
	uint8_t good[sizeof(struct config)];

	memset(good, 0, sizeof(good));

	uint8_t *buf = (uint8_t*)&config;
	for(unsigned int c = 0; c < sizeof(magic); c++){
		printf("testing magic %02x\n", magic[c]);
		// fill config with magic values
		memset(&config, magic[c], sizeof(config));
		// reset
		config_reset(&config);
		// check
		for(unsigned int j = 0; j < sizeof(config); j++){
			if(buf[j] != magic[c]){
				good[j] = 1;
			}
		}
	}
	int failed = 0;
	for(unsigned int j = 0; j < sizeof(config); j++){
		if(!good[j]){
			failed = 1;
			break;
		}
	}
	if(failed){
		printf("Not all values in the config have preset defaults:\n");
		printf("0x%04x: ", (int)0);
		int fail = 0;
		for(unsigned int j = 0; j < sizeof(config); j++){
			if(!good[j]){
				printf("%02x", buf[j]);
				fail++;
			} else {
				printf(".");
			}
			if((j % 64) == 63){
				printf("\n0x%04x: ", j);
			}
		}
		printf("\n");
		EXPECT_EQ(0, fail);
	}
}

/**
 * @ingroup CONFIG
 * @page CONFIG
 *
 * - Make sure that initial load after reset does not modify the config.
 */
TEST_F(ConfigTest, TestFirstLoad){
	struct config_store config;
	struct config_store zero;
	memset(&config, 0, sizeof(struct config));
	memset(&zero, 0, sizeof(struct config));
	EXPECT_TRUE(config_load(&config, mock_syscalls()) < 0);
	// this now fails because we actually do a reset before loading the config
	//EXPECT_EQ(0, memcmp(&config.data, &zero.data, sizeof(struct config)));
}

/**
 * @ingroup CONFIG
 * @page CONFIG
 *
 * - Test that config saving and loading works. Must make sure that only deltas
 * are being written. At the very least checksum will be written as a delta
 * since it is not part of the data.
 */
TEST_F(ConfigTest, TestSaveLoad){
	struct config_store a, b;
	memset(&a, 0, sizeof(a));
	memset(&b, 0, sizeof(b));
	config_reset(&a);
	EXPECT_EQ(0, config_save(&a, mock_syscalls()));
	// since we are using default config, we should not have any writes besides crc (one size of a delta)
	EXPECT_EQ(4, mock_eeprom_written);

	EXPECT_EQ(0, config_load(&b, mock_syscalls()));
	EXPECT_EQ(0, memcmp(&a, &b, sizeof(struct config)));
}

/**
 * @ingroup CONFIG
 * @page CONFIG
 *
 * - Test that when eeprom content is corrupted, loading of config fails.
 */
TEST_F(ConfigTest, TestBadEEPROMData){
	struct config_store config;
	struct config_store res;
	config_reset(&res);
	// modify some parameter
	config_get_rate_profile_rw(&res.data)->rcRate8 = 8;

	EXPECT_TRUE(config_save(&res, mock_syscalls()) == 0);
	// we should have written two deltas (1 for checksum and one for data)
	EXPECT_EQ(8, mock_eeprom_written);
	// loading will be successful
	EXPECT_TRUE(config_load(&config, mock_syscalls()) == 0);
	memset(&config, 0, sizeof(config));
	memset(&res, 0, sizeof(config));
	// corrupt a few bytes
	mock_eeprom_data[2] = 0x66;
	// loading should fail
	EXPECT_TRUE(config_load(&config, mock_syscalls()) < 0);
	// make sure that the data was not modified
	// this is no longer true because we want to save stack so config is modified
	//EXPECT_EQ(0, memcmp(&config.data, &res.data, sizeof(struct config)));
}


/**
 * @ingroup CONFIG
 * @page CONFIG
 *
 * - Test how the delta packing code behaves when there is not enough memory.
 * It should correctly return error when config can not be saved.
 */

TEST_F(ConfigTest, TestPageBoundarySaveLoad){
	struct config_store a, b;
	memset(&a, 0, sizeof(a));
	memset(&b, 0, sizeof(b));

	uint8_t *data = (uint8_t*)&a;
	// fill with random data
	for(size_t c = 0; c < sizeof(a); c++){
		data[c] = c & 0xff;
	}

	// small eeprom
	mock_eeprom_page_size = 256;
	mock_eeprom_pages = 2;

	// this should fail
	EXPECT_TRUE(config_save(&a, mock_syscalls()) < 0);

	mock_eeprom_page_size = 512;
	mock_eeprom_pages = 8;
	mock_eeprom_written = 0;

	// this should be ok
	EXPECT_EQ(0, config_save(&a, mock_syscalls()));
/*
	for(size_t c = 0; c < (mock_eeprom_page_size * mock_eeprom_pages); c++){
		printf("%02x ", (int)mock_eeprom_data[c] & 0xff);
		if(c > 0 && c % 64 == 63) printf("\n");
	}
	printf("\n");
	*/
	EXPECT_EQ(0, config_load(&b, mock_syscalls()));
/*
	for(size_t c = 0; c < sizeof(a); c++){
		int aa = (int)((char*)&a)[c] & 0xff;
		int bb = (int)((char*)&b)[c] & 0xff;
		if(aa != bb) printf("\nfail at %02x (%02x != %02x)\n", (unsigned int)c, aa, bb);
		printf("%02x%02x", aa, bb);
		if(c > 0 && c % 64 == 63) printf("\n");
	}
	printf("\n");
*/
	EXPECT_EQ(0, memcmp(&a, &b, sizeof(struct config)));
}

/**
 * @ingroup CONFIG
 * @page CONFIG
 *
 * - Test that system works the same way on flash based memory (where erased
 * value is 0xff) and on normal eeprom where eraced value is 0x00.
 */
TEST_F(ConfigTest, TestFlashvsNormalSaveLoad){
	struct config_store a, b;
	memset(&a, 0, sizeof(a));
	memset(&b, 0, sizeof(b));

	uint8_t *data = (uint8_t*)&a;
	// fill with random data
	for(size_t c = 0; c < sizeof(a); c++){
		data[c] = rand();
	}

	mock_eeprom_erase_byte = 0x00;
	EXPECT_EQ(0, config_save(&a, mock_syscalls()));
	EXPECT_EQ(0, config_load(&b, mock_syscalls()));
	mock_eeprom_erase_byte = 0xff;
	memset(&b, 0, sizeof(b));

	EXPECT_EQ(0, config_save(&a, mock_syscalls()));
	EXPECT_EQ(0, config_load(&b, mock_syscalls()));

	EXPECT_EQ(0, memcmp(&a, &b, sizeof(struct config)));
}

/**
 * @ingroup CONFIG
 * @page CONFIG
 *
 * - Test that loading of configs works as expected when eeprom reads fail
 */
TEST_F(ConfigTest, TestEEPROMFail){
	struct config_store a;
	memset(&a, 0, sizeof(a));

	uint8_t *data = (uint8_t*)&a;
	// fill with random data
	for(size_t c = 0; c < sizeof(a); c++){
		data[c] = c & 0xff;
	}

	// must be ok
	EXPECT_TRUE(config_save(&a, mock_syscalls()) == 0);

	// clamp the eeprom so we try to read past the end and get an EIO
	mock_eeprom_page_size = 512;
	mock_eeprom_pages = 1;

	// this should fail
	EXPECT_TRUE(config_save(&a, mock_syscalls()) < 0);
	EXPECT_TRUE(config_load(&a, mock_syscalls()) < 0);
}

/**
 * @ingroup CONFIG
 * @page CONFIG
 *
 * - Test various fixups.
 */

TEST_F(ConfigTest, TestFixups){
	struct config_store a;
	config_reset(&a);
	// if fixups modify default values then something is seriosly wrong
	EXPECT_FALSE(config_fixup(&a));

	// we know that blackbox fixup will change rate_num
	a.data.blackbox.rate_num = 0;
	EXPECT_TRUE(config_fixup(&a));

	// check that balckbox fixup computes the correct gcd
	a.data.blackbox.rate_num = 6;
	a.data.blackbox.rate_denom = 8;

	EXPECT_TRUE(config_fixup(&a));
	EXPECT_EQ(3, a.data.blackbox.rate_num);
	EXPECT_EQ(4, a.data.blackbox.rate_denom);
}

//static uint8_t flash_data[512];
extern uint8_t __config_start;
//uint8_t *__config_start = flash_data;
//uint8_t *__config_end = flash_data + sizeof(flash_data);

TEST_F(ConfigTest, TestLowLevel){
	struct system_calls *sys = mock_syscalls();
	sys->eeprom.read = _eeprom_read;
	sys->eeprom.write = _eeprom_write;
	sys->eeprom.erase_page = _eeprom_erase_page;
	sys->eeprom.get_info = _eeprom_get_info;

	// assume random data in the eeprom
	// this means no space left in the eeprom so it should result in a full reset upon first boot.
	for(size_t c = 0; c < (512 * 8); c++){
		*(&__config_start + c) = 0xAA;
	}

	struct config_store config;
	config_reset(&config);
	if(config_load(&config, sys) < 0){
		printf("erase config\n");
		config_erase(&config, sys);
	}
	config.data.blackbox.rate_num = 12;
	printf("save\n");
	EXPECT_EQ(0, config_save(&config, sys));
	config.data.blackbox.rate_num = 11;
	printf("load\n");
	EXPECT_EQ(0, config_load(&config, sys));
	EXPECT_EQ(12, config.data.blackbox.rate_num);

	printf("random\n");
	for(size_t c = 0; c < sizeof(config); c++){
		*((uint8_t*)&config + c) = rand();
	}
	EXPECT_EQ(0, config_save(&config, sys));
	struct config_store nc;
	config_reset(&nc);
	EXPECT_EQ(0, config_load(&nc, sys));
	EXPECT_EQ(0, memcmp(&nc, &config, sizeof(config)));
}
