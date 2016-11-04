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
#include "io/motor_and_servo.h"
#include "io/rc_controls.h"
#include "io/gimbal.h"
#include "io/ledstrip.h"
#include "io/display.h"
#include "io/asyncfatfs/asyncfatfs.h"
#include "io/transponder_ir.h"
#include "io/msp.h"
#include "io/serial_msp.h"
#include "io/serial_cli.h"

#include "sensors/sensors.h"
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
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/servos.h"
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
	struct imu imu;
	struct mixer mixer;
	struct anglerate controller;
	struct fc_sitl_server_interface *sitl;
	pthread_t thread;
};

static void _application_send_state(struct application *self){
	struct fc_sitl_client_interface *cl = self->sitl->client;
	printf("pwm: ");
	for(int c = 0; c < 4; c++){
		printf("%d ", mixer_get_motor_value(&self->mixer, c));
		cl->write_pwm(cl, c, mixer_get_motor_value(&self->mixer, c));
	}
	for(int c = 4; c < FC_SITL_PWM_CHANNELS; c++){
		cl->write_pwm(cl, c, 1500);
	}
	printf("\n");
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

	float accel[3], gyro[3];

	cl->read_accel(cl, accel);
	cl->read_gyro(cl, gyro);
	imu_input_accelerometer(&self->imu,
		(accel[0] / 9.82f) * 512,
		(accel[1] / 9.82f) * 512,
		(accel[2] / 9.82f) * 512);

	gyroADC[0] = gyro[0] * 4096;
	gyroADC[1] = -gyro[1] * 4096;
	gyroADC[2] = -gyro[2] * 4096;

	imu_input_gyro(&self->imu, gyroADC[0], gyroADC[1], gyroADC[2]);
	imu_update(&self->imu, 0.001);

#if 0
	struct sitl_client_packet pkt;
	sitl_recv_state(&pkt);
	//imu_input_accelerometer(&self->imu, pkt.accel[0], pkt.accel[1], pkt.accel[2]);
	#endif
}
static void _application_fc_run(struct application *self){
	union attitude_euler_angles att;
	imu_get_attitude_dd(&self->imu, &att);
	rcCommand[ROLL] = (rc_get_channel_value(0) - 1500) >> 2;
	rcCommand[PITCH] = (rc_get_channel_value(1) - 1500) >> 2;
	rcCommand[THROTTLE] = rc_get_channel_value(2);
	rcCommand[YAW] = (rc_get_channel_value(3) - 1500) >> 2;
	anglerate_update(&self->controller, &att, 0.001);
	const struct pid_controller_output *out = anglerate_get_output_ptr(&self->controller);
	printf("pid output: %d %d %d\n", out->axis[0], out->axis[1], out->axis[2]);

	mixer_enable_motor_outputs(&self->mixer, true);
	mixer_update(&self->mixer, anglerate_get_output_ptr(&self->controller));
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
		customMotorMixer(0), MAX_SUPPORTED_MOTORS);

	mixer_set_motor_disarmed_pwm(&self->mixer, 0, 1000);
	mixer_set_motor_disarmed_pwm(&self->mixer, 1, 1000);
	mixer_set_motor_disarmed_pwm(&self->mixer, 2, 1000);
	mixer_set_motor_disarmed_pwm(&self->mixer, 3, 1000);

	static struct rate_config rateConfig;
	memset(&rateConfig, 0, sizeof(struct rate_config));
	pidProfile()->pidController = 1;
	pidProfile()->P8[PIDROLL] = 10;
	pidProfile()->P8[PIDPITCH] = 10;
	pidProfile()->P8[PIDYAW] = 60;

	pidProfile()->I8[PIDROLL] = 5;
	pidProfile()->I8[PIDPITCH] = 5;
	pidProfile()->I8[PIDYAW] = 10;

	pidProfile()->D8[PIDROLL] = 20;
	pidProfile()->D8[PIDPITCH] = 20;
	pidProfile()->D8[PIDYAW] = 5;

	anglerate_init(&self->controller,
		pidProfile(),
		&rateConfig,
		imuConfig()->max_angle_inclination,
		&accelerometerConfig()->accelerometerTrims,
		rxConfig()
	);

	for(int c = 0; c < 3; c++){
		anglerate_set_pid_axis_scale(&self->controller, c, 100);
		anglerate_set_pid_axis_weight(&self->controller, c, 100);
	}

	imu_init(&self->imu,
		imuConfig(),
		accelerometerConfig(),
		throttleCorrectionConfig(),
		1.0f/16.4f,
		512
	);

	pthread_create(&self->thread, NULL, _application_thread, self);
}

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
uint8_t armingFlags = 0xff;
int32_t gyroADC[3];
int32_t magADC[3];
uint32_t rcModeActivationMask = 0;
float magneticDeclination = 0;
//void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims) {rollAndPitchTrims->values.roll = 0;rollAndPitchTrims->values.pitch = 0;};
bool rcModeIsActive(boxId_e modeId) { return rcModeActivationMask & (1 << modeId); }
uint32_t gyro_sync_get_looptime(void){ return 2000; }
bool sensors(uint32_t mask) { UNUSED(mask); return true; }
int32_t getRcStickDeflection(int32_t axis, uint16_t midrc) {return MIN(ABS(rc_get_channel_value(axis) - midrc), 500);}
void parseRcChannels(const char *input, rxConfig_t *rxConfig);

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
bool isSerialConfigValid(serialConfig_t *);
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
void useRcControlsConfig(modeActivationCondition_t *);
void useRcControlsConfig(modeActivationCondition_t *c){(void)c;}
void parseRcChannels(const char *input, rxConfig_t *rxConfig){UNUSED(input);UNUSED(rxConfig);}
void suspendRxSignal(void){}
void failureMode(uint8_t mode){UNUSED(mode);}
void resumeRxSignal(void){}
void failsafeReset(void);
void failsafeReset(void){}
bool isEEPROMContentValid(void);
bool isEEPROMContentValid(void){ return true; }

