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

#define SONAR_OUT_OF_RANGE (-1)

struct sonar_hardware; 
struct sonar; 

struct sonar_ops {
	void (*start_reading)(struct sonar *self); 
}; 

struct sonar {
	int16_t distance; 
	int16_t altitude; 
	uint16_t max_range_cm; 
	uint16_t detection_cone_deci_degrees; 
	uint16_t detection_cone_extended_deci_degrees; 
	int16_t max_alt_with_tilt_cm; 
	int16_t cf_alt_cm; 
	int16_t max_tilt_deci_degrees; 
	float max_tilt_cos; 
	int16_t max_alt_with_tilt; 
	struct sonar_ops *ops; 
	const struct sonar_hardware *hw; 
}; 

// TODO: remove after refactoring
extern struct sonar default_sonar; 
/*
extern int16_t sonarMaxRangeCm;
extern int16_t sonarCfAltCm;
extern int16_t sonarMaxAltWithTiltCm;
*/

void sonar_init(struct sonar *self);
void sonar_update(struct sonar *self);
int32_t sonar_read(struct sonar *self);
int32_t sonar_calc_altitude(struct sonar *self, float cosTiltAngle);
int32_t sonar_get_altitude(struct sonar *self);

