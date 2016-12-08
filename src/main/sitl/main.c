#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>

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
#include "drivers/gyro_sync.h"

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
#include "blackbox/blackbox.h"

#include "flight/anglerate.h"
#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"

#include "config/config.h"
#include "config/feature.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#include "sitl.h"
#include "ninja.h"

#include <unistd.h>
#include <time.h>
// since _XOPEN_SOURCE (or posix 2008) usleep is deprecated and nanosleep should be used instead.
#if _XOPEN_SOURCE > 500
int usleep(uint32_t us){
	struct timespec req = {
		.tv_sec = (__time_t)(us / 1000000UL),
		.tv_nsec = (__time_t)((us % 1000000UL) * 1000)
	};
	struct timespec rem;
	return nanosleep(&req, &rem);
}
#endif

struct application {
	struct ninja ninja;
	struct config config;

	struct fc_sitl_server_interface *sitl;
	pthread_t thread;

	struct system_calls syscalls;
};

static void _application_send_state(struct application *self){
	struct fc_sitl_client_interface *cl = self->sitl->client;
	cl->update_euler_angles(cl, ins_get_roll_dd(&self->ninja.ins), ins_get_pitch_dd(&self->ninja.ins), ins_get_yaw_dd(&self->ninja.ins));
}

static void _application_fc_run(struct application *self){
	ninja_heartbeat(&self->ninja);
}

static void application_run(struct application *self){
	_application_fc_run(self);
	_application_send_state(self);
}

// main thread for the application that reads user inputs and runs the flight controller
static void *_application_thread(void *param){
	struct application *app = (struct application*)param;
	while (true) {
		application_run(app);
		usleep(900);
    }
	return NULL;
}

#include <string.h>
static int32_t _micros(const struct system_calls_time *time){
	(void)time;
	struct timespec ts;
	static struct timespec start_ts = {0, 0};
	clock_gettime(CLOCK_MONOTONIC, &ts);
	if(start_ts.tv_sec == 0) memcpy(&start_ts, &ts, sizeof(start_ts));
	int32_t t = (ts.tv_sec - start_ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;
	//printf("read time: %d\n", t);
	return t;
}

static void _write_motor(const struct system_calls_pwm *pwm, uint8_t id, uint16_t value){
	struct application *self = container_of(container_of(pwm, struct system_calls, pwm), struct application, syscalls);
	struct fc_sitl_client_interface *cl = self->sitl->client;
	cl->write_pwm(cl, id, value);
}

static void _write_servo(const struct system_calls_pwm *pwm, uint8_t id, uint16_t value){
	struct application *self = container_of(container_of(pwm, struct system_calls, pwm), struct application, syscalls);
	struct fc_sitl_client_interface *cl = self->sitl->client;
	cl->write_pwm(cl, 8 + id, value);
}

static uint16_t _read_pwm(const struct system_calls_pwm *pwm, uint8_t id){
	struct application *self = container_of(container_of(pwm, struct system_calls, pwm), struct application, syscalls);
	struct fc_sitl_client_interface *cl = self->sitl->client;
	return cl->read_rc(cl, id);
}

static uint16_t _read_ppm(const struct system_calls_pwm *pwm, uint8_t id){
	struct application *self = container_of(container_of(pwm, struct system_calls, pwm), struct application, syscalls);
	struct fc_sitl_client_interface *cl = self->sitl->client;
	return cl->read_rc(cl, id);
}

static int _read_gyro(const struct system_calls_imu *imu, int16_t output[3]){
	struct application *self = container_of(container_of(imu, struct system_calls, imu), struct application, syscalls);
	struct fc_sitl_client_interface *cl = self->sitl->client;
	float gyr[3];
	cl->read_gyro(cl, gyr);

	// scale gyro to int16 range. 35.0f is 2000 deg/s max gyro range in radians.
	// incoming gyro data from sim is in rad/s
	output[0] = (gyr[0] / 35.0f) * 32768;
	output[1] = (gyr[1] / 35.0f) * 32768;
	output[2] = (gyr[2] / 35.0f) * 32768;

	return 0;
}

static int _read_acc(const struct system_calls_imu *imu, int16_t output[3]){
	struct application *self = container_of(container_of(imu, struct system_calls, imu), struct application, syscalls);
	struct fc_sitl_client_interface *cl = self->sitl->client;
	float accel[3];

	cl->read_accel(cl, accel);

	// accelerometer is scaled to 512 (acc_1G)
	output[0] = (accel[0] / 9.82f) * 512;
	output[1] = (accel[1] / 9.82f) * 512;
	output[2] = (accel[2] / 9.82f) * 512;

	return 0;
}

static void _led_on(const struct system_calls_leds *leds, uint8_t id, bool on){
	struct application *self = container_of(container_of(leds, struct system_calls, leds), struct application, syscalls);
	struct fc_sitl_client_interface *cl = self->sitl->client;

	cl->led_on(cl, id, on);
}

static void _led_toggle(const struct system_calls_leds *leds, uint8_t id){
	struct application *self = container_of(container_of(leds, struct system_calls, leds), struct application, syscalls);
	struct fc_sitl_client_interface *cl = self->sitl->client;

	cl->led_toggle(cl, id);
}

static void _beeper_on(const struct system_calls_beeper *calls, bool on){
	(void)calls;
	//struct application *self = container_of(container_of(calls, struct system_calls, beeper), struct application, syscalls);
	//struct fc_sitl_client_interface *cl = self->sitl->client;
	
	static bool last = false;
	if(!last && on)
		printf("BEEP\n");
	last = on;
	fflush(stdout);
}

#define SITL_EEPROM_PAGE_SIZE 4096
#define SITL_EEPROM_NUM_PAGES 1

char _flash[SITL_EEPROM_PAGE_SIZE * SITL_EEPROM_NUM_PAGES];

static int _eeprom_read(const struct system_calls_eeprom *self, void *dst, uint16_t addr, size_t size){
	(void)self;
	(void)dst;
	printf("EEPROM read from %04x, size %lu\n", addr, size);
	if((addr + size) > sizeof(_flash)){
		size = sizeof(_flash) - addr;
	}
	memcpy(dst, _flash + addr, size);
	return 0;
}

static int _eeprom_write(const struct system_calls_eeprom *self, uint16_t addr, const void *data, size_t size){
	(void)self;
	(void)data;
	printf("EEPROM write to %04x, size %lu\n", addr, size);
	memcpy(_flash + addr, data, size);
	return 0;
}

static int _eeprom_erase_page(const struct system_calls_eeprom *self, uint16_t addr){
	(void)self;
	(void)addr;
	// we don't use pages
	return 0;
}

static void _eeprom_get_info(const struct system_calls_eeprom *self, struct system_eeprom_info *info){
	(void)self;
	info->page_size = SITL_EEPROM_PAGE_SIZE;
	info->num_pages = SITL_EEPROM_NUM_PAGES;
}

static void application_init(struct application *self, struct fc_sitl_server_interface *server){
	self->sitl = server;

	self->syscalls = (struct system_calls){
		.pwm = {
			.write_motor = _write_motor,
			.write_servo = _write_servo,
			.read_ppm = _read_ppm,
			.read_pwm = _read_pwm
		},
		.imu = {
			.read_gyro = _read_gyro,
			.read_acc = _read_acc
		},
		.leds = {
			.on = _led_on,
			.toggle = _led_toggle
		},
		.beeper = {
			.on = _beeper_on
		},
		.time = {
			.micros = _micros
		},
		.eeprom = {
			.read = _eeprom_read,
			.write = _eeprom_write,
			.erase_page = _eeprom_erase_page,
			.get_info = _eeprom_get_info
		}
	};

	config_load(&self->config, &self->syscalls);

	ninja_init(&self->ninja, &self->syscalls, &self->config);

	struct config *conf = &self->config;
	struct pid_config *pid = &config_get_profile_rw(conf)->pid;
	struct rate_profile *rates = config_get_rate_profile_rw(conf);

	pid->P8[PIDROLL] = 40;
	pid->I8[PIDROLL] = 0;
	pid->D8[PIDROLL] = 20;
	
	pid->P8[PIDPITCH] = 40;
	pid->I8[PIDPITCH] = 0;
	pid->D8[PIDPITCH] = 20;

	pid->P8[PIDYAW] = 50;
	pid->I8[PIDYAW] = 5;
	pid->D8[PIDYAW] = 30;

	pid->P8[PIDLEVEL] = 20;
	pid->I8[PIDLEVEL] = 10;
	pid->D8[PIDLEVEL] = 100;

	rates->rates[ROLL] = 100;
    rates->rates[PITCH] = 100;
    rates->rates[YAW] = 100;

	conf->mixer.mixerMode = MIXER_QUADX;

	pthread_create(&self->thread, NULL, _application_thread, self);
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
struct fc_sitl_server_interface *fc_sitl_create_aircraft(struct fc_sitl_client_interface *cl);
struct fc_sitl_server_interface *fc_sitl_create_aircraft(struct fc_sitl_client_interface *cl){
	UNUSED(cl);

	struct fc_sitl_server_interface *server = calloc(1, sizeof(struct fc_sitl_server_interface));

	// save client interface pointer so we can send pwm to it and read rc inputs
	server->client = cl;

	// start a flight controller application for this client
	struct application *app = malloc(sizeof(struct application));
	application_init(app, server);
	return server;
}

// TODO: these should be part of a struct (defined in flight controller)
uint8_t stateFlags;
uint16_t flightModeFlags;
uint32_t rcModeActivationMask = 0;
float magneticDeclination = 0;
uint32_t gyro_sync_get_looptime(void){ return 2000; }

uint8_t cliMode;
void validateAndFixConfig(void);
void validateAndFixConfig(void){}
void setAccelerationTrims(flightDynamicsTrims_t *trims);
void setAccelerationTrims(flightDynamicsTrims_t *trims){(void)trims;}
void recalculateMagneticDeclination(void);
void recalculateMagneticDeclination(void){}
//void beeperSilence(void) {}
//void beeperConfirmationBeeps(void);
//void beeperConfirmationBeeps(void){}
//void beeper(uint8_t type) { (void)type; }
void failureMode(uint8_t mode){UNUSED(mode);}
int16_t adcGetChannel(uint8_t chan) { (void)chan; return 0; }
bool isAccelerationCalibrationComplete(void){ return true; }
bool isGyroCalibrationComplete(void){ return true; }

// for rx reception stuff
/*bool isPPMDataBeingReceived(void){ return true; }
bool resetPPMDataReceivedState(void){ return true; }
bool isPWMDataBeingReceived(void){ return true; }
*/
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
