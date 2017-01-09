/*
 * This file is part of Ninjaflight.
 *
 * Unit testing and docs: Martin Schröder <mkschreder.uk@gmail.com>
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
#include <stdbool.h>

#include <limits.h>

extern "C" {
    #include <platform.h>

	#include "common/utils.h"
	#include "common/pt.h"
    #include "config/config.h"

	#include <FreeRTOS.h>
	#include <task.h>
	#include <queue.h>

    #include "rx/rx.h"
	#include "flight/failsafe.h"
    #include "common/maths.h"

	#include "ninja.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"


class FastloopTest : public ::testing::Test {
protected:
    virtual void SetUp() {
		mock_system_reset();
		config_reset(&config);
    }
public:
	struct config_store config;
	struct fastloop loop;
};

TEST_F(FastloopTest, TestFastloop){
	struct tester {
		static void _test_task(void *param){
			FastloopTest *self = (FastloopTest*)param;
			struct fastloop *loop = &self->loop;
			struct fastloop_input in;
			struct fastloop_output out;

			memset(&in, 0, sizeof(in));
			
			// write default controls
			in.throttle = -500;
			fastloop_write_controls(loop, &in);

			mock_acc[0] = 0;
			mock_acc[1] = 0;
			mock_acc[2] = SYSTEM_ACCEL_1G;

			vTaskDelay(5);

			fastloop_read_outputs(loop, &out);
			printf("looptime: %d\n", out.loop_time);

			EXPECT_EQ(SYSTEM_ACCEL_1G, out.acc[2]);

			// arm, throttle up and check outputs
			in.throttle = -250;
			in.mode |= FLARM;

			fastloop_write_controls(loop, &in); vTaskDelay(5);

			// all motors will have the given throttle
			EXPECT_EQ(1250, mock_motor_pwm[0]);
			EXPECT_EQ(1250, mock_motor_pwm[1]);
			EXPECT_EQ(1250, mock_motor_pwm[2]);
			EXPECT_EQ(1250, mock_motor_pwm[3]);

			// try rolling
			in.roll = -500;
			in.throttle = 0;
			fastloop_write_controls(loop, &in);

			self->config.data.pwm_out.mincommand = 1000;
			vTaskDelay(100);
			
			EXPECT_EQ(1000, mock_motor_pwm[0]);
			EXPECT_EQ(1000, mock_motor_pwm[1]);
			EXPECT_EQ(1000, mock_motor_pwm[2]);
			EXPECT_EQ(1000, mock_motor_pwm[3]);
			vTaskEndScheduler();
		}
	};

	fastloop_init(&loop, mock_syscalls(), &config.data);
	fastloop_start(&loop);

	xTaskCreate(tester::_test_task, "test", 128, this, 1, NULL);

	vTaskStartScheduler();
}

