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

	printf("time = []\n");
	printf("rc_roll = []\n");
	printf("rc_pitch = []\n");
	printf("rc_yaw = []\n");
	printf("rc_throttle = []\n");
	printf("gyr_x = []\n");
	printf("gyr_y = []\n");
	printf("gyr_z = []\n");
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
			printf("gyr_x = [gyr_x %d];\n", frame.gyr[0]);
			printf("gyr_y = [gyr_y %d];\n", frame.gyr[1]);
			printf("gyr_z = [gyr_z %d];\n", frame.gyr[2]);
			/*
			printf("%d: rc[%d %d %d %d], gyr [%d %d %d], acc[%d %d %d]\n", 
				frame.time,
				frame.command[0],
				frame.command[1],
				frame.command[2],
				frame.command[3],
				frame.gyr[0],
				frame.gyr[1],
				frame.gyr[2],
				frame.acc[0],
				frame.acc[1],
				frame.acc[2]);
			*/
		}
	}
	printf("plot(time, gyr_x, time, gyr_y, time, gyr_z);\n");
	printf("print -dpng plot_gyro.png\n");
	printf("plot(time, rc_roll, time, rc_pitch, time, rc_yaw, time, rc_throttle);\n");
	printf("print -dpng plot_rc.png\n");
	printf("input(\"press any key\")");
}
