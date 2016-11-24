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

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"
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
#include "io/flashfs.h"
#include "io/gps.h"
#include "io/rc_controls.h"
#include "io/ledstrip.h"
#include "io/display.h"
#include "io/asyncfatfs/asyncfatfs.h"
#include "io/transponder_ir.h"
#include "io/msp.h"
#include "io/serial_msp.h"
#include "io/serial_cli.h"

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

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_system.h"
#include "config/feature.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#include "sitl.h"
#include "ninja.h"

struct application {
	struct ninja ninja;
	struct fc_sitl_server_interface *sitl;
	pthread_t thread;

	struct system_calls syscalls;
};

static void _application_send_state(struct application *self){
	struct fc_sitl_client_interface *cl = self->sitl->client;
	cl->update_euler_angles(cl, ins_get_roll_dd(&self->ninja.ins), ins_get_pitch_dd(&self->ninja.ins), ins_get_yaw_dd(&self->ninja.ins));
#if 0
	struct sitl_server_packet pkt;
	memset(&pkt, 0, sizeof(pkt));
	pkt.mode = SITL_MODE_PHYSICS_ON_CLIENT;
	pkt.frame = (uint8_t)SITL_FRAME_QUAD_X;
	union attitude_euler_angles att;
	imu_get_attitude_dd(&self->imu, &att);
	//pkt.euler[0] = att.values.roll * 0.1f; pkt.euler[1] = att.values.pitch * 0.1f; pkt.euler[2] = att.values.yaw * 0.1f;
	for(int c = 0; c < 4; c++){
		self->sitl
	pkt.servo[0] = mixer_get_motor_value(&self->ninja.mixer, 1);
	pkt.servo[1] = mixer_get_motor_value(&self->ninja.mixer, 2);
	pkt.servo[2] = mixer_get_motor_value(&self->ninja.mixer, 3);
	pkt.servo[3] = mixer_get_motor_value(&self->ninja.mixer, 0);
	for(int c = 4; c < 8; c++){
		pkt.servo[c] = 1500;
	}
	printf("motors: %d %d %d %d\n",
		mixer_get_motor_value(&self->ninja.mixer,0),
		mixer_get_motor_value(&self->ninja.mixer,1),
		mixer_get_motor_value(&self->ninja.mixer,2),
		mixer_get_motor_value(&self->ninja.mixer,3)
	);
	sitl_send_state(&pkt);
#endif
}
static void _application_recv_state(struct application *self){
	struct fc_sitl_client_interface *cl = self->sitl->client;
	//printf("rc: ");
	for(int c = 0; c < 8; c++){
		uint16_t pwm = cl->read_rc(cl, c);
		rc_set_channel_value(c, pwm);
		//printf("%d ", pwm);
	}
	//printf("\n");

#if 0
	struct sitl_client_packet pkt;
	sitl_recv_state(&pkt);
	//imu_input_accelerometer(&self->imu, pkt.accel[0], pkt.accel[1], pkt.accel[2]);
	#endif
}
static void _application_fc_run(struct application *self){
	ninja_heartbeat(&self->ninja);
	/*
	int16_t roll = (rc_get_channel_value(0) - 1500);
	int16_t pitch = (rc_get_channel_value(1) - 1500);
	int16_t yaw = (rc_get_channel_value(3) - 1500);
	int16_t throttle = rc_get_channel_value(2) - 1000;
	anglerate_input_user(&self->ninja.ctrl, roll, pitch, yaw);
	anglerate_input_body_rates(&self->ninja.ctrl, ins_get_gyro_x(&self->ninja.ins), ins_get_gyro_y(&self->ninja.ins), ins_get_gyro_z(&self->ninja.ins));
	anglerate_input_body_angles(&self->ninja.ctrl, ins_get_roll_dd(&self->ninja.ins), ins_get_pitch_dd(&self->ninja.ins), ins_get_yaw_dd(&self->ninja.ins));
	anglerate_set_level_percent(&self->ninja.ctrl, 100, 100);
	anglerate_update(&self->ninja.ctrl, 0.001);
	printf("rcCommand: %d %d %d %d\n", roll, pitch, yaw, throttle);
	printf("pid output: %d %d %d\n", anglerate_get_roll(&self->ninja.ctrl), anglerate_get_pitch(&self->ninja.ctrl), anglerate_get_yaw(&self->ninja.ctrl));
	//printf("acc: %d %d %d\n", ins_get_acc_x(&self->ninja.ins), ins_get_acc_y(&self->ninja.ins), ins_get_acc_z(&self->ninja.ins));
	printf("gyro: %d %d %d\n", ins_get_gyro_x(&self->ninja.ins), ins_get_gyro_y(&self->ninja.ins), ins_get_gyro_z(&self->ninja.ins));
	printf("roll: %d, pitch: %d, yaw: %d\n", ins_get_roll_dd(&self->ninja.ins), ins_get_pitch_dd(&self->ninja.ins), ins_get_yaw_dd(&self->ninja.ins));

	mixer_enable_armed(&self->ninja.mixer, true);
	mixer_input_command(&self->ninja.mixer, MIXER_INPUT_G0_ROLL, anglerate_get_roll(&self->ninja.ctrl));
	mixer_input_command(&self->ninja.mixer, MIXER_INPUT_G0_PITCH, anglerate_get_pitch(&self->ninja.ctrl));
	mixer_input_command(&self->ninja.mixer, MIXER_INPUT_G0_YAW, anglerate_get_yaw(&self->ninja.ctrl));
	mixer_input_command(&self->ninja.mixer, MIXER_INPUT_G0_THROTTLE, throttle - 500);
	mixer_update(&self->ninja.mixer);
	*/
}

static void application_run(struct application *self){
	_application_recv_state(self);
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
	fflush(stdout);
}

static void _led_toggle(const struct system_calls_leds *leds, uint8_t id){
	struct application *self = container_of(container_of(leds, struct system_calls, leds), struct application, syscalls);
	struct fc_sitl_client_interface *cl = self->sitl->client;

	cl->led_toggle(cl, id);
	fflush(stdout);
}


static void application_init(struct application *self, struct fc_sitl_server_interface *server){
	resetEEPROM();
	self->sitl = server;

	pidProfile()->P8[PIDROLL] = 90;
	pidProfile()->I8[PIDROLL] = 10;
	pidProfile()->D8[PIDROLL] = 30;
	
	pidProfile()->P8[PIDPITCH] = 90;
	pidProfile()->I8[PIDPITCH] = 10;
	pidProfile()->D8[PIDPITCH] = 30;

	pidProfile()->P8[PIDYAW] = 98;
	pidProfile()->I8[PIDYAW] = 5;
	pidProfile()->D8[PIDYAW] = 60;

	pidProfile()->P8[PIDLEVEL] = 20;
	pidProfile()->I8[PIDLEVEL] = 10;
	pidProfile()->D8[PIDLEVEL] = 100;

	controlRateProfiles(0)->rates[ROLL] = 173;
    controlRateProfiles(0)->rates[PITCH] = 173;
    controlRateProfiles(0)->rates[YAW] = 173;

	mixerConfig()->mixerMode = MIXER_QUADX;

	ninja_init(&self->ninja, &self->syscalls);

	pthread_create(&self->thread, NULL, _application_thread, self);

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
		.time = {
			.micros = _micros
		}
	};
}

#include <fcntl.h>
#include <termio.h>
#include <sys/stat.h>
// shared library entry point used by client to instantiate a flight controller
// client is allocated by the client and passed to us as a pointer so we safe it within the server object
struct fc_sitl_server_interface *fc_sitl_create_aircraft(struct fc_sitl_client_interface *cl);
struct fc_sitl_server_interface *fc_sitl_create_aircraft(struct fc_sitl_client_interface *cl){
	UNUSED(cl);
/*	
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
	printf("opened serial device %s\n", ptsname);
	grantpt(fd);
	unlockpt(fd);
	int sfd = open(ptsname, O_RDWR);
	if(sfd <= 0) perror("open");
	//if(write(sfd, "Hello", 6) < 0) perror("write");
	if(read(fd, ptsname, 6) < 0) perror("read");

	printf("Got: %s\n", ptsname);
	exit(0);
*/
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
//int32_t gyroADC[3];
//int32_t magADC[3];
uint32_t rcModeActivationMask = 0;
float magneticDeclination = 0;
//void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims) {rollAndPitchTrims->values.roll = 0;rollAndPitchTrims->values.pitch = 0;};
uint32_t gyro_sync_get_looptime(void){ return 2000; }

#include "config/config_streamer.h"
void config_streamer_init(config_streamer_t *c){UNUSED(c); }

void config_streamer_start(config_streamer_t *c, uintptr_t base, int size){
	UNUSED(c); UNUSED(base); UNUSED(size);
}

int config_streamer_write(config_streamer_t *c, const uint8_t *p, uint32_t size){
	UNUSED(c); UNUSED(p); UNUSED(size);
	return 0;
}

int config_streamer_flush(config_streamer_t *c){
	UNUSED(c);
	return 0;
}

int config_streamer_finish(config_streamer_t *c){
	UNUSED(c);
	return 0;
}

uint8_t cliMode;
void scanEEPROM(void);
void scanEEPROM(void){}
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
void writeConfigToEEPROM(void);
void writeConfigToEEPROM(void){}
void failureMode(uint8_t mode){UNUSED(mode);}
bool isEEPROMContentValid(void);
bool isEEPROMContentValid(void){ return true; }
int16_t adcGetChannel(uint8_t chan) { (void)chan; return 0; }
bool isAccelerationCalibrationComplete(void){ return true; }
bool isGyroCalibrationComplete(void){ return true; }
bool isPPMDataBeingReceived(void){ return true; }
bool resetPPMDataReceivedState(void){ return true; }
bool isPWMDataBeingReceived(void){ return true; }

serialPort_t *usbVcpOpen(uint8_t id, serialReceiveCallbackPtr callback, uint32_t baudRate, portMode_t mode, portOptions_t options){ 
	(void)id;
	(void) callback;
	(void)baudRate;
	(void)mode;
	(void)options;
	return NULL; 
}
serialPort_t *uartOpen(uint8_t id, serialReceiveCallbackPtr callback, uint32_t baudRate, portMode_t mode, portOptions_t options) { 
	(void)id;
	(void) callback;
	(void)baudRate;
	(void)mode;
	(void)options;
	return NULL; 
}
serialPort_t *openSoftSerial(softSerialPortIndex_e id, serialReceiveCallbackPtr callback, uint32_t baudRate, portOptions_t options) { 
	(void)id;
	(void) callback;
	(void)baudRate;
	(void)options;
	return NULL; 
}
void serialSetMode(serialPort_t* port,portMode_t mode){
	(void)port;
	(void)mode;
}
bool isSerialTransmitBufferEmpty(serialPort_t* port){
	(void)port;
	return true;
}
void systemResetToBootloader(void){}
void serialBeginWrite(serialPort_t* port){ (void)port; }
void serialWriteBuf(serialPort_t* port,uint8_t * data,int count){ (void)port; (void)data; (void)count;}
void serialWrite(serialPort_t* port,uint8_t data){ (void)port; (void)data; }
void serialEndWrite(serialPort_t* port){ (void)port;}
uint8_t serialRead(serialPort_t* port){ (void)port; return 0;}
uint8_t serialRxBytesWaiting(serialPort_t* port){ (void)port; return 0;}
void serialSetBaudRate(serialPort_t *port, uint32_t baud) { (void)port; (void)baud; }
uint32_t serialGetBaudRate(serialPort_t *port){ (void)port; return 9600; }
void serialPrint(serialPort_t *port, const char *str){ (void)port; (void)str; }
void systemReset(void){}

// TODO: fix these
const char *buildDate = "JAN 01 1970";
const char *buildTime = "1234567890";
const char *shortGitRevision = "123123123123123";

int16_t debug[DEBUG16_VALUE_COUNT];

void setLedHsv(void){}
int getLedHsv(void){ return 0; }
bool isWS2811LedStripReady(void){ return false; }
void ws2811LedStripInit(void){}
void ws2811UpdateStrip(void){}
#include "common/color.h"
void setStripColor(const hsvColor_t *color){ (void) color; }
