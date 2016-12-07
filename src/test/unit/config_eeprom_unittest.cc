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

class ConfigTest : public ::testing::Test {
protected:
    virtual void SetUp() {
		mock_system_reset();

		config_reset(&config);
    }
	struct config config;
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

TEST_F(ConfigTest, TestSaveLoad){
	struct config a, b;
	memset(&a, 0, sizeof(a));
	memset(&b, 0, sizeof(b));
	config_reset(&a);
	EXPECT_EQ(0, config_save(&a, mock_syscalls()));
	// since we are using default config, we should not have any writes besides crc (one size of a delta)
	EXPECT_EQ(4, mock_eeprom_written);

	EXPECT_EQ(0, config_load(&b, mock_syscalls()));
	EXPECT_EQ(0, memcmp(&a, &b, sizeof(struct config)));
}

TEST_F(ConfigTest, TestPageBoundarySaveLoad){
	struct config a, b;
	memset(&a, 0, sizeof(a));
	memset(&b, 0, sizeof(b));

	uint8_t *data = (uint8_t*)&a;
	// fill with random data
	for(size_t c = 0; c < sizeof(a); c++){
		data[c] = rand();
	}

	// small eeprom
	mock_eeprom_page_size = 256;
	mock_eeprom_pages = 2;

	// this should fail
	EXPECT_TRUE(config_save(&a, mock_syscalls()) < 0);

	mock_eeprom_page_size = 512;
	mock_eeprom_pages = 8;

	// this should be ok
	EXPECT_EQ(0, config_save(&a, mock_syscalls()));

	EXPECT_EQ(0, config_load(&b, mock_syscalls()));
	EXPECT_EQ(0, memcmp(&a, &b, sizeof(struct config)));
}

TEST_F(ConfigTest, TestFlashvsNormalSaveLoad){
	struct config a, b;
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


