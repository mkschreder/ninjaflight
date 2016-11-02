#pragma once

#include <stdint.h>

typedef enum {
	SITL_FRAME_UNKNOWN = 0,
	SITL_FRAME_QUAD_PLUS,
	SITL_FRAME_QUAD_X, 
	SITL_FRAME_RANGER_TILT_X
} sitl_frame_type_t; 

enum {
	SITL_MODE_PHYSICS_ON_CLIENT = 0, 
	SITL_MODE_PHYSICS_ON_SERVER = 1
}; 

// packets sent from sitl to simulator
struct sitl_server_packet {
	unsigned long long time; 
	uint8_t mode; 
	uint8_t frame; 
	int16_t servo[8]; 
	float pos[3]; 
	float vel[3]; 
	float euler[3]; 
	float acc[3]; 
	float mag[3]; 
};

// packets sent from simulator to us
struct sitl_client_packet {
	uint32_t id; 
	float timestamp;
	float gyro[3]; 
	float accel[3]; 
	float euler[3]; 
	float pos[3]; 
	float vel[3]; 
	float rcin[8]; 
	int32_t loc[3]; 
	float mag[3]; 
	float range[6]; 
};

int sitl_init(void); 
int sitl_start(void); 
int sitl_send_state(const struct sitl_server_packet *packet); 
int sitl_recv_state(struct sitl_client_packet *packet); 

