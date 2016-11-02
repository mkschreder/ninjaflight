/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
	Copyright (c) Martin Schröder <mkschreder.uk@gmail.com>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
	SITL interface
*/

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <stdbool.h>
#include <errno.h>
#include <memory.h>

#include "sitl.h"
struct sitl {
	char *_shmout;  
	char *_shmin; 
}; 

static struct sitl sitl; 

int sitl_init(void){ 
	struct sitl *self = &sitl; 
	// setup shared memory!

	int out = shmget(9003, sizeof(struct sitl_server_packet), 0666); 
	if(out < 0) perror("shmget"); 
	int in = shmget(9005, sizeof(struct sitl_client_packet), 0666); 
	if(in < 0) perror("shmget"); 

	self->_shmin = (char*)shmat(in, NULL, 0); 
	self->_shmout = (char*)shmat(out, NULL, 0); 

	if(!self->_shmin || !self->_shmout) return -ENOENT; 

	memset(self->_shmin, 0, sizeof(struct sitl_client_packet)); 

	return 0;
}

int sitl_send_state(const struct sitl_server_packet *pkt){
	struct sitl *self = &sitl; 
	if(!self->_shmout) return -EINVAL; 
	memcpy(self->_shmout, pkt, sizeof(struct sitl_server_packet)); 
	return 0; 
}

int sitl_recv_state(struct sitl_client_packet *pkt){
	struct sitl *self = &sitl; 
	if(!self->_shmin) return -EINVAL; 
	memcpy(pkt, self->_shmin, sizeof(struct sitl_client_packet)); 
	return 0; 
}
