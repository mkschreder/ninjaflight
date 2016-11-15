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

#pragma once

#include "../config/boardalignment.h"

struct board_alignment {
	bool standardBoardAlignment;
	float rMat[3][3];              // matrix
	struct board_alignment_config *config;
};

void board_alignment_init(struct board_alignment *self, struct board_alignment_config *config); 
void board_alignment_rotate_vector(struct board_alignment *self, int32_t *src, int32_t *dest, uint8_t rotation);
