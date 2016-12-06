/*
 * This file is part of Ninjaflight.
 *
 * Ninjaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Ninjaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Ninjaflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "common/maths.h"
#include "common/axis.h"

#include "config/config.h"
#include "config/sensors.h"

#include "boardalignment.h"

void board_alignment_init(struct board_alignment *self, const struct board_alignment_config *config){
	memset(self, 0, sizeof(struct board_alignment));
	self->config = config;
	self->standardBoardAlignment = true;
	matrix_set_identity(self->rMat);

	// check if standard alignment
    if(!config || (!config->rollDegrees && !config->pitchDegrees && !config->yawDegrees)){
        return;
    }

    self->standardBoardAlignment = false;

    fp_angles_t rotationAngles;
    rotationAngles.angles.roll = degreesToRadians(config->rollDegrees);
    rotationAngles.angles.pitch = degreesToRadians(config->pitchDegrees);
    rotationAngles.angles.yaw = degreesToRadians(config->yawDegrees);

    buildRotationMatrix(&rotationAngles, self->rMat);
}

static void _calc_alignment(struct board_alignment *self, int32_t *vec){
    int32_t x = vec[X];
    int32_t y = vec[Y];
    int32_t z = vec[Z];

	// matrix multiplication
    vec[X] = lrintf(self->rMat[0][X] * x + self->rMat[1][X] * y + self->rMat[2][X] * z);
    vec[Y] = lrintf(self->rMat[0][Y] * x + self->rMat[1][Y] * y + self->rMat[2][Y] * z);
    vec[Z] = lrintf(self->rMat[0][Z] * x + self->rMat[1][Z] * y + self->rMat[2][Z] * z);
}

void board_alignment_rotate_vector(struct board_alignment *self, int32_t *src, int32_t *dest, uint8_t rotation){
    static uint32_t swap[3];
    memcpy(swap, src, sizeof(swap));

    switch (rotation) {
        default:
        case CW0_DEG:
            dest[X] = swap[X];
            dest[Y] = swap[Y];
            dest[Z] = swap[Z];
            break;
        case CW90_DEG:
            dest[X] = swap[Y];
            dest[Y] = -swap[X];
            dest[Z] = swap[Z];
            break;
        case CW180_DEG:
            dest[X] = -swap[X];
            dest[Y] = -swap[Y];
            dest[Z] = swap[Z];
            break;
        case CW270_DEG:
            dest[X] = -swap[Y];
            dest[Y] = swap[X];
            dest[Z] = swap[Z];
            break;
        case CW0_DEG_FLIP:
            dest[X] = -swap[X];
            dest[Y] = swap[Y];
            dest[Z] = -swap[Z];
            break;
        case CW90_DEG_FLIP:
            dest[X] = swap[Y];
            dest[Y] = swap[X];
            dest[Z] = -swap[Z];
            break;
        case CW180_DEG_FLIP:
            dest[X] = swap[X];
            dest[Y] = -swap[Y];
            dest[Z] = -swap[Z];
            break;
        case CW270_DEG_FLIP:
            dest[X] = -swap[Y];
            dest[Y] = -swap[X];
            dest[Z] = -swap[Z];
            break;
    }

    if (!self->standardBoardAlignment)
        _calc_alignment(self, dest);
}
