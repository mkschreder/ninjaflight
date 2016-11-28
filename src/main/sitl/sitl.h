#pragma once

#include <stdint.h>
#include "../../ninjasitl/fc_sitl.h"

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
