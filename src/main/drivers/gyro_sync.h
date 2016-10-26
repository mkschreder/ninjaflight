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

#define INTERRUPT_WAIT_TIME 10

uint32_t gyro_sync_get_looptime(void); 
bool gyroSyncCheckUpdate(void);
uint8_t gyroMPU6xxxCalculateDivider(void);
void gyroSetSampleRate(uint32_t looptime, uint8_t lpf, uint8_t gyroSync, uint8_t gyroSyncDenominator);
