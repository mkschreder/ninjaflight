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

#include "scheduler.h"
#include "sitl.h"

struct application {
	struct instruments ins;
	struct mixer mixer;
	struct anglerate controller;
	struct fc_sitl_server_interface *sitl;
	pthread_t thread;
};

static void _application_send_state(struct application *self){
	struct fc_sitl_client_interface *cl = self->sitl->client;
	// TODO: motor mapping is different in sim compared to ninjaflight. Need to standardize it.
	cl->write_pwm(cl, 0, mixer_get_motor_value(&self->mixer, 1));
	cl->write_pwm(cl, 1, mixer_get_motor_value(&self->mixer, 2));
	cl->write_pwm(cl, 2, mixer_get_motor_value(&self->mixer, 3));
	cl->write_pwm(cl, 3, mixer_get_motor_value(&self->mixer, 0));
	for(int c = 4; c < FC_SITL_PWM_CHANNELS; c++){
		cl->write_pwm(cl, c, 1500);
	}

	cl->update_euler_angles(cl, ins_get_roll_dd(&self->ins), ins_get_pitch_dd(&self->ins), ins_get_yaw_dd(&self->ins));
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
	pkt.servo[0] = mixer_get_motor_value(&self->mixer, 1);
	pkt.servo[1] = mixer_get_motor_value(&self->mixer, 2);
	pkt.servo[2] = mixer_get_motor_value(&self->mixer, 3);
	pkt.servo[3] = mixer_get_motor_value(&self->mixer, 0);
	for(int c = 4; c < 8; c++){
		pkt.servo[c] = 1500;
	}
	printf("motors: %d %d %d %d\n",
		mixer_get_motor_value(&self->mixer,0),
		mixer_get_motor_value(&self->mixer,1),
		mixer_get_motor_value(&self->mixer,2),
		mixer_get_motor_value(&self->mixer,3)
	);
	sitl_send_state(&pkt);
#endif
}
static void _application_recv_state(struct application *self){
	struct fc_sitl_client_interface *cl = self->sitl->client;
	printf("rc: ");
	for(int c = 0; c < 8; c++){
		uint16_t pwm = cl->read_rc(cl, c);
		rc_set_channel_value(c, pwm);
		printf("%d ", pwm);
	}
	printf("\n");

	float accel[3], gyr[3];

	cl->read_accel(cl, accel);
	cl->read_gyro(cl, gyr);

	int16_t w[3];
	// scale gyro to int16 range. 35.0f is 2000 deg/s max gyro range in radians.
	// incoming gyro data from sim is in rad/s
	w[0] = (gyr[0] / 35.0f) * 32768;
	w[1] = (gyr[1] / 35.0f) * 32768;
	w[2] = (gyr[2] / 35.0f) * 32768;

	// accelerometer is scaled to 512 (acc_1G)
	accel[0] = (accel[0] / 9.82f) * 512;
	accel[1] = (accel[1] / 9.82f) * 512;
	accel[2] = (accel[2] / 9.82f) * 512;

	ins_process_acc(&self->ins, accel[0], accel[1], accel[2]);

	printf("acc: %d %d %d\n", (int)accel[0], (int)accel[1], (int)accel[2]);
	ins_process_gyro(&self->ins, w[0], w[1], w[2]);

	ins_update(&self->ins, 0.001);

#if 0
	struct sitl_client_packet pkt;
	sitl_recv_state(&pkt);
	//imu_input_accelerometer(&self->imu, pkt.accel[0], pkt.accel[1], pkt.accel[2]);
	#endif
}
static void _application_fc_run(struct application *self){
	// TODO: rc commands need to be passed directly into anglerate controller instead of being in global state
	if(!ins_is_calibrated(&self->ins)) return;
	int16_t roll = (rc_get_channel_value(0) - 1500);
	int16_t pitch = (rc_get_channel_value(1) - 1500);
	int16_t yaw = (rc_get_channel_value(3) - 1500);
	int16_t throttle = rc_get_channel_value(2) - 1000;
	anglerate_input_user(&self->controller, roll, pitch, yaw);
	anglerate_input_body_rates(&self->controller, ins_get_gyro_x(&self->ins), ins_get_gyro_y(&self->ins), ins_get_gyro_z(&self->ins));
	anglerate_input_body_angles(&self->controller, ins_get_roll_dd(&self->ins), ins_get_pitch_dd(&self->ins), ins_get_yaw_dd(&self->ins));
	anglerate_set_level_percent(&self->controller, 100, 100);
	anglerate_update(&self->controller, 0.001);
	printf("rcCommand: %d %d %d %d\n", roll, pitch, yaw, throttle);
	printf("pid output: %d %d %d\n", anglerate_get_roll(&self->controller), anglerate_get_pitch(&self->controller), anglerate_get_yaw(&self->controller));
	//printf("acc: %d %d %d\n", ins_get_acc_x(&self->ins), ins_get_acc_y(&self->ins), ins_get_acc_z(&self->ins));
	printf("gyro: %d %d %d\n", ins_get_gyro_x(&self->ins), ins_get_gyro_y(&self->ins), ins_get_gyro_z(&self->ins));
	printf("roll: %d, pitch: %d, yaw: %d\n", ins_get_roll_dd(&self->ins), ins_get_pitch_dd(&self->ins), ins_get_yaw_dd(&self->ins));

	mixer_enable_armed(&self->mixer, true);
	mixer_input_command(&self->mixer, MIXER_INPUT_G0_ROLL, anglerate_get_roll(&self->controller));
	mixer_input_command(&self->mixer, MIXER_INPUT_G0_PITCH, anglerate_get_pitch(&self->controller));
	mixer_input_command(&self->mixer, MIXER_INPUT_G0_YAW, anglerate_get_yaw(&self->controller));
	mixer_input_command(&self->mixer, MIXER_INPUT_G0_THROTTLE, throttle - 500);
	mixer_update(&self->mixer);
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
		usleep(1000);
    }
	return NULL;
}

static void application_init(struct application *self, struct fc_sitl_server_interface *server){
	resetEEPROM();
	self->sitl = server;
    mixer_init(&self->mixer,
		mixerConfig(),
		motor3DConfig(),
		motorAndServoConfig(),
		rxConfig(),
		rcControlsConfig(),
		servoProfile()->servoConf,
		customMotorMixer(0), MAX_SUPPORTED_MOTORS);

	static struct rate_config rateConfig;
	memset(&rateConfig, 0, sizeof(struct rate_config));

	pidProfile()->P8[PIDROLL] = 40;
	pidProfile()->I8[PIDROLL] = 30;
	pidProfile()->D8[PIDROLL] = 23;
	
	pidProfile()->P8[PIDPITCH] = 40;
	pidProfile()->I8[PIDPITCH] = 30;
	pidProfile()->D8[PIDPITCH] = 23;

	pidProfile()->P8[PIDYAW] = 60;
	pidProfile()->I8[PIDYAW] = 45;
	pidProfile()->D8[PIDYAW] = 0;

	pidProfile()->P8[PIDLEVEL] = 20;
	pidProfile()->I8[PIDLEVEL] = 10;
	pidProfile()->D8[PIDLEVEL] = 100;

	rateConfig.rates[ROLL] = 173;
    rateConfig.rates[PITCH] = 173;
    rateConfig.rates[YAW] = 173;

	imuConfig()->dcm_kp = 2500;
	imuConfig()->dcm_ki = 2500;
    imuConfig()->looptime = 1000;
    imuConfig()->gyroSync = 1;
    imuConfig()->gyroSyncDenominator = 1;
    imuConfig()->small_angle = 25;
    imuConfig()->max_angle_inclination = 450;

	anglerate_init(&self->controller,
		&self->ins,
		pidProfile(),
		&rateConfig,
		imuConfig()->max_angle_inclination,
		&accelerometerConfig()->trims,
		rxConfig()
	);
	anglerate_set_algo(&self->controller, PID_CONTROLLER_LUX_FLOAT);

	for(int c = 0; c < 3; c++){
		anglerate_set_pid_axis_scale(&self->controller, c, 100);
		anglerate_set_pid_axis_weight(&self->controller, c, 100);
	}

	ins_init(&self->ins,
		boardAlignment(),
		imuConfig(),
		throttleCorrectionConfig(),
		gyroConfig(),
		compassConfig(),
		sensorTrims(),
		accelerometerConfig(),
		1.0f/16.4f,
		512
	);

	pthread_create(&self->thread, NULL, _application_thread, self);
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
uint8_t armingFlags = 0xff;
//int32_t gyroADC[3];
//int32_t magADC[3];
uint32_t rcModeActivationMask = 0;
float magneticDeclination = 0;
//void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims) {rollAndPitchTrims->values.roll = 0;rollAndPitchTrims->values.pitch = 0;};
bool rcModeIsActive(boxId_e modeId) { return rcModeActivationMask & (1 << modeId); }
uint32_t gyro_sync_get_looptime(void){ return 2000; }
bool sensors(uint32_t mask) { UNUSED(mask); return true; }
int32_t getRcStickDeflection(int32_t axis, uint16_t midrc) {return MIN(ABS(rc_get_channel_value(axis) - midrc), 500);}

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

void scanEEPROM(void);
void scanEEPROM(void){}
void validateAndFixConfig(void);
void validateAndFixConfig(void){}
bool isSerialConfigValid(serialConfig_t *c){(void)c; return true;}
void resetAdjustmentStates(void);
void resetAdjustmentStates(void){}
void setAccelerationTrims(flightDynamicsTrims_t *trims);
void setAccelerationTrims(flightDynamicsTrims_t *trims){(void)trims;}
void recalculateMagneticDeclination(void);
void recalculateMagneticDeclination(void){}
void beeperConfirmationBeeps(void);
void beeperConfirmationBeeps(void){}
void writeConfigToEEPROM(void);
void writeConfigToEEPROM(void){}
void useRcControlsConfig(modeActivationCondition_t *c){(void)c;}
void parseRcChannels(const char *input, rxConfig_t *rxConfig){UNUSED(input);UNUSED(rxConfig);}
void suspendRxSignal(void){}
void failureMode(uint8_t mode){UNUSED(mode);}
void resumeRxSignal(void){}
void failsafeReset(void){}
bool isEEPROMContentValid(void);
bool isEEPROMContentValid(void){ return true; }

bool isAccelerationCalibrationComplete(void){ return true; }
bool isGyroCalibrationComplete(void){ return true; }
void calculateRxChannelsAndUpdateFailsafe(uint32_t t){ (void)t;}
