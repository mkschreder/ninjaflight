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
#include <sched.h>
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


class TasksTest : public ::testing::Test {
protected:
    virtual void SetUp() {
		mock_system_reset();
    }
};

struct pchar {
	xQueueHandle queue;
	TaskHandle_t task_handle;
};

static void _pchar_task(void *param){
	struct pchar *self = (struct pchar*)param;
	while(true){
		char ch = 0;
		if(xQueueReceive(self->queue, &ch, portMAX_DELAY)){
			printf("%c", ch);
			fflush(stdout);
		}
	}
}

void pchar_init(struct pchar *self){
	self->queue = xQueueCreate(16, sizeof(char));
	xTaskCreate(_pchar_task, "pchar", 128, self, 0, &self->task_handle);
}

void pchar_write(struct pchar *self, const char ch){
	xQueueSend(self->queue, &ch, portMAX_DELAY);
}

void pchar_stop(struct pchar *self){
	vQueueDelete(self->queue);
	vTaskDelete(self->task_handle);
}

struct printer_request {
	uint8_t size;
	char data[32];
};

struct printer {
	xQueueHandle out_queue, req_queue;
	struct printer_request requests[2];
	struct pchar *pchar;
	StackType_t stack[512];
	StaticTask_t task;
	TaskHandle_t task_handle;
};

void _printer_task(void *param){
	struct printer *self = (struct printer*)param;
	while(true){
		struct printer_request *req = NULL;
		if(xQueueReceive(self->out_queue, &req, portMAX_DELAY)){
			for(uint8_t c = 0; c < req->size; c++){
				pchar_write(self->pchar, req->data[c]);
			}
			xQueueSend(self->req_queue, &req, portMAX_DELAY);
		}
	}
}

void printer_init(struct printer *self, struct pchar *pchar){
	self->out_queue = xQueueCreate(2, sizeof(struct printer_request*));
	self->req_queue = xQueueCreate(2, sizeof(struct printer_request*));
	struct printer_request *r = NULL;
	for(int c = 0; c < 2; c++){
		r = &self->requests[c];
		xQueueSend(self->req_queue, &r, portMAX_DELAY);
	}
	self->pchar = pchar;
	//self->task_handle = xTaskCreateStatic(_printer_task, "printer", 512, self, 0, self->stack, &self->task);
	xTaskCreate(_printer_task, "printer", 128, self, 0, &self->task_handle);
}

static void printer_write(struct printer *self, const char *data, size_t size){
	while(size){
		struct printer_request *req = NULL;
		if(xQueueReceive(self->req_queue, &req, portMAX_DELAY) == pdTRUE){
			size_t s = (size > sizeof(req->data))?sizeof(req->data):size;
			memcpy(req->data, data, s);
			req->size = s;
			data += s;
			size -= s;
			xQueueSend(self->out_queue, &req, portMAX_DELAY);
		}
	}
}

void printer_stop(struct printer *self){
	vQueueDelete(self->out_queue);
	vQueueDelete(self->req_queue);
	vTaskDelete(self->task_handle);
}

struct pchar pchar;
struct printer printer;

static void _main_task(void *ptr){
	(void)ptr;
	//memset(data, 0, sizeof(data));
	for(int c = 0; c < 2; c++){
		const char *str = "Hello World!\n";
		printer_write(&printer, str, strlen(str));
		vTaskDelay(100);
	}
	printer_stop(&printer);
	pchar_stop(&pchar);
	printf("stopping scheduler!\n"); fflush(stdout);
	vTaskEndScheduler();
}

TEST_F(TasksTest, TestScheduler){
	cpu_set_t set;
	CPU_SET(0, &set);
	sched_setaffinity(getpid(), sizeof(set), &set);

	printf("pthread minimum stack size: %d\n", PTHREAD_STACK_MIN);
	pchar_init(&pchar);
	printer_init(&printer, &pchar);
	TaskHandle_t task;
	xTaskCreate(_main_task, "main", 128, NULL, 0, &task);
	vTaskStartScheduler();
	vTaskDelete(task);
	printf("tasks done!\n");
	fflush(stdout);
}

