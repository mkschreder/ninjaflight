#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include "ninja.h"
#include "common/packer.h"
#include "common/ulink.h"
#include "blackbox.h"

int main(int argc, char **argv){
	(void)argc;

	int fd = open(argv[1], O_RDONLY);
	if(fd < 0){
		perror("opening input file");
		exit(1);
	}

	char buffer[4096];
	struct blackbox_frame state;
	memset(&state, 0, sizeof(state));
	int rd = 0;
	struct blackbox_frame frame;
	memset(&frame, 0, sizeof(frame));

	printf("time = [];\n");
	printf("rc_roll = [];\n");
	printf("rc_pitch = [];\n");
	printf("rc_yaw = [];\n");
	printf("rc_throttle = [];\n");
	printf("gyr_x = [];\n");
	printf("gyr_y = [];\n");
	printf("gyr_z = [];\n");
	printf("acc_x = [];\n");
	printf("acc_y = [];\n");
	printf("acc_z = [];\n");
	for(int c = 0; c < 8; c++){
		printf("motor_%d = [];\n", c);
		printf("servo_%d = [];\n", c);
	}
	while((rd = read(fd, buffer, sizeof(buffer))) > 0){
		size_t total = 0;
		while(total < rd){
			size_t parsed = 0;
			int ret = blackbox_parse(buffer + total, rd - total, &frame, &parsed);
			if(ret < 0) break;
			total += parsed;
			printf("time = [time %d];\n", frame.time);
			printf("rc_roll = [rc_roll %d];\n", frame.command[0]);
			printf("rc_pitch = [rc_pitch %d];\n", frame.command[1]);
			printf("rc_yaw = [rc_yaw %d];\n", frame.command[2]);
			printf("rc_throttle = [rc_throttle %d];\n", frame.command[3]);
			printf("gyr_x = [gyr_x %d];\n", frame.gyr[0] >> 4);
			printf("gyr_y = [gyr_y %d];\n", frame.gyr[1] >> 4);
			printf("gyr_z = [gyr_z %d];\n", frame.gyr[2] >> 4);
			printf("acc_x = [acc_x %d];\n", frame.acc[0] >> 4);
			printf("acc_y = [acc_y %d];\n", frame.acc[1] >> 4);
			printf("acc_z = [acc_z %d];\n", frame.acc[2] >> 4);
			for(int c = 0; c < 8; c++){
				printf("motor_%d = [motor_%d %d];\n", c, c, frame.motors[c]);
				printf("servo_%d = [servo_%d %d];\n", c, c, frame.servos[c]);
			}
		}
	}
	printf("plot(time, gyr_x, time, gyr_y, time, gyr_z, "
		"time, acc_x, time, acc_y, time, acc_z,");
	for(int c = 0; c < 8; c++){
		printf("time, motor_%d, ", c);
		printf("time, servo_%d, ", c);
	}
	printf("time, acc_x, time, acc_y, time, acc_z,"
		"time, rc_roll, time, rc_pitch, time, rc_yaw, time, rc_throttle);\n");
	printf("print -dpng plot_all.png\n");
	printf("plot(time, gyr_x, time, gyr_y, time, gyr_z);\n");
	printf("print -dpng plot_gyro.png\n");

	printf("plot(");
	for(int c = 0; c < 8; c++){
		printf("time, motor_%d", c);
		if(c != 7) printf(",");
	}
	printf(");\n");
	printf("print -dpng plot_motor.png\n");

	printf("plot(");
	for(int c = 0; c < 8; c++){
		printf("time, servo_%d", c);
		if(c != 7) printf(",");
	}
	printf(");\n");
	printf("print -dpng plot_servo.png\n");

	printf("plot(time, acc_x, time, acc_y, time, acc_z);\n");
	printf("plot(time, acc_x, time, acc_y, time, acc_z);\n");
	printf("print -dpng plot_acc.png\n");
	printf("plot(time, rc_roll, time, rc_pitch, time, rc_yaw, time, rc_throttle);\n");
	printf("print -dpng plot_rc.png\n");
	printf("input(\"press any key\")");
}
