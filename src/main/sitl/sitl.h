#pragma once

#include <stdint.h>
#include "../../ninjarace/fc_sitl.h"

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

/*
int sitl_init(void); 
int sitl_start(void); 
int sitl_send_state(const struct sitl_server_packet *packet); 
int sitl_recv_state(struct sitl_client_packet *packet); 
*/
