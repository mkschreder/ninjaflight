#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>

#include <platform.h>

#include "build_config.h"
#include "debug.h"

#include "common/axis.h"
#include "common/color.h"
//#include "common/atomic.h"
#include "common/maths.h"
#include "common/streambuf.h"
#include "common/utils.h"

#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/light_led.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "drivers/serial_uart.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/inverter.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "io/serial.h"
#include "io/ledstrip.h"
#include "io/display.h"
#include "io/transponder_ir.h"
#include "io/serial_msp.h"

#include "sensors/sonar.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/initialisation.h"

#include "telemetry/telemetry.h"

#include "flight/anglerate.h"
#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"

#include "config/config.h"
#include "config/feature.h"

#include "system_calls.h"

#include <FreeRTOS.h>
#include <task.h>

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#include "sitl.h"
#include "ninja.h"
#include "fastloop.h"
#include "blackbox.h"

struct application {
	struct ninja ninja;
	struct config_store config;

	struct fastloop fastloop;

	// this is provided by the sitl
	struct system_calls *system;
	//struct fc_sitl_server_interface *sitl;
	pthread_t thread;

	//struct system_calls syscalls;
};

static void _app_task(void *param){
	struct application *self = (struct application*)param;

	while (true) {
		ninja_heartbeat(&self->ninja);
		vTaskDelay(5);
    }
}

static void _io_task(void *param){
	struct application *self = (struct application*)param;

	while (true) {
		(void)self;
		// FIXME: call io main loop from this task
		vTaskDelay(500);
		//usleep(900);
    }

}

// main thread for the application that reads user inputs and runs the flight controller
static void *_application_thread(void *param){
	struct application *self = (struct application*)param;
	
	int ret = 0;
	if((ret = config_load(&self->config, self->system)) < 0){
		printf("ERROR loading config (%d)\n", ret);
	}

	// default sitl smaple rate (1000 looptime)
	self->config.data.imu.gyro_sample_div = 8;

	fastloop_init(&self->fastloop, self->system, &self->config.data);
	ninja_init(&self->ninja, &self->fastloop, self->system, &self->config);

	fastloop_start(&self->fastloop);

	// simulate as closely as possible what the real system would do
	xTaskCreate(_app_task, "app", 4096, self, 3, NULL);
	xTaskCreate(_io_task, "io", 4096, self, 2, NULL);

	vTaskStartScheduler();
	return NULL;
}

#include <sched.h>

static void application_init(struct application *self, struct system_calls *system){
	self->system = system;
	pthread_create(&self->thread, NULL, _application_thread, self);
	//struct sched_param param;
	//param.sched_priority = sched_get_priority_max(SCHED_FIFO);
	//pthread_setschedparam(self->thread, SCHED_FIFO, &param);
}

#include <fcntl.h>
#include <termio.h>
#include <sys/stat.h>

struct sitl_serial_port {
	serialPort_t dev;
	int fd;
};

// shared library entry point used by client to instantiate a flight controller
// client is allocated by the client and passed to us as a pointer so we safe it within the server object
void *fc_sitl_create_aircraft(struct system_calls *cl);
void *fc_sitl_create_aircraft(struct system_calls *cl){
	UNUSED(cl);

	// start a flight controller application for this client
	struct application *app = malloc(sizeof(struct application));
	application_init(app, cl);

	// return app as handle to our sitl for now
	return app;
}

// TODO: these should be part of a struct (defined in flight controller)
uint32_t gyro_sync_get_looptime(void){ return 2000; }
int16_t adcGetChannel(uint8_t chan) { (void)chan; return 0; }

serialPort_t *usbVcpOpen(uint8_t id, serialReceiveCallbackPtr callback, uint32_t baudRate, portMode_t mode, portOptions_t options){ 
	(void)id;
	(void) callback;
	(void)baudRate;
	(void)mode;
	(void)options;
	return NULL; 
}

static void _serial_write_char(serialPort_t *port, uint8_t ch){
	struct sitl_serial_port *self = container_of(port, struct sitl_serial_port, dev);
	write(self->fd, &ch, 1);
}

static uint8_t _serial_rx_waiting(serialPort_t *port){
	struct sitl_serial_port *self = container_of(port, struct sitl_serial_port, dev);
	int bytes;
	ioctl(self->fd, FIONREAD, &bytes);
	return bytes;
}

static uint8_t _serial_tx_free(serialPort_t *port){
	(void)port;
	return 255;
}

static uint8_t _serial_read_char(serialPort_t *port){
	struct sitl_serial_port *self = container_of(port, struct sitl_serial_port, dev);
	uint8_t ch = 0;
	read(self->fd, &ch, 1);
	return ch;
}

static void _serial_set_baud_rate(serialPort_t *port, uint32_t baud){
	printf("UART%d: baud rate %d\n", port->identifier, baud);
}

static bool _serial_tx_empty(serialPort_t *port){
	(void)port;
	return true;
}

static void _serial_set_mode(serialPort_t *port, portMode_t mode){
	(void)port; (void)mode;
}

static void _serial_write(serialPort_t *port, void *data, int count){
	struct sitl_serial_port *self = container_of(port, struct sitl_serial_port, dev);
	uint8_t *ptr = (uint8_t*)data;
	while(count > 0){
		int r = write(self->fd, ptr, count);
		if(r < 0) {
			perror("UART write");
			break;
		}
		count -= r;
	}
}

static void _serial_begin_write(serialPort_t *port){
	(void)port;
}

static void _serial_end_write(serialPort_t *port){
	(void)port;
}

static struct serial_port_ops _serial_port_ops = {
	.put = _serial_write_char,
	.serialTotalRxWaiting = _serial_rx_waiting,
	.serialTotalTxFree = _serial_tx_free,
	.serialRead = _serial_read_char,
	.serialSetBaudRate = _serial_set_baud_rate,
	.isSerialTransmitBufferEmpty = _serial_tx_empty,
	.setMode = _serial_set_mode,
	.writeBuf = _serial_write,
	.beginWrite = _serial_begin_write,
	.endWrite = _serial_end_write
};

serialPort_t *uartOpen(uint8_t id, serialReceiveCallbackPtr callback, uint32_t baudRate, portMode_t mode, portOptions_t options) { 
	(void)id;
	(void) callback;
	(void)baudRate;
	(void)mode;
	(void)options;
	int fd = open("/dev/ptmx", O_RDWR);
	if(fd <= 0){
		perror("opening serial terminal");
		return NULL;
	}
	int nr;
	if(ioctl(fd, TIOCGPTN, &nr) != 0){
		return NULL;
	}
	char ptsname[32]; 
	sprintf(ptsname, "/dev/pts/%d", nr);
	printf("UART%d, baud %d, created at '%s'\n", id, baudRate, ptsname);
	grantpt(fd);
	unlockpt(fd);

	struct sitl_serial_port *port = calloc(1, sizeof(struct sitl_serial_port));
	port->dev.vTable = &_serial_port_ops;
	port->dev.baudRate = baudRate;
	port->fd = fd;

	sleep(1);
	return &port->dev;
}
serialPort_t *openSoftSerial(softSerialPortIndex_e id, serialReceiveCallbackPtr callback, uint32_t baudRate, portOptions_t options) { 
	(void)id;
	(void) callback;
	(void)baudRate;
	(void)options;
	return NULL; 
}

void systemResetToBootloader(void){}
void systemReset(void){}

int16_t debug[DEBUG16_VALUE_COUNT];

void setLedHsv(void){}
int getLedHsv(void){ return 0; }
bool isWS2811LedStripReady(void){ return false; }
void ws2811LedStripInit(void){}
void ws2811UpdateStrip(void){}
#include "common/color.h"
void setStripColor(const hsvColor_t *color){ (void) color; }

void vApplicationStackOverflowHook(void){
	printf("STACK OVERFLOW DETECTED!\n");
	fflush(stdout);
	void (*ptr)() = NULL;
	ptr(); // crash
}

void vApplicationMallocFailedHook(void){
	printf("MALLOC FAILURE DETECTED\n");
	fflush(stdout);
	void (*ptr)() = NULL;
	ptr(); // crash
}

void vApplicationIdleHook(void){

}

void vApplicationTickHook(void){

}

#include <FreeRTOS.h>
void vApplicationGetIdleTaskMemory(StaticTask_t **task, StackType_t **stack, uint32_t *size){
	static StaticTask_t _task;
	static StackType_t _stack[128];
	*task = &_task;
	*stack = &_stack[0];
	*size = sizeof(_stack);
}

void vApplicationGetTimerTaskMemory(StaticTask_t **task, StackType_t **stack, uint32_t *size){
	static StaticTask_t _task;
	static StackType_t _stack[128];
	*task = &_task;
	*stack = _stack;
	*size = sizeof(_stack);
}

