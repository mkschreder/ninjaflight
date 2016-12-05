/*
 * This file is part of Ninjaflight.
 *
 * Copyright: collective.
 * Cleanup: Martin Schröder <mkschreder.uk@gmail.com>
 * Original source from Cleanflight.
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

typedef enum BlackboxDevice {
    BLACKBOX_DEVICE_SERIAL = 0,

#ifdef USE_FLASHFS
    BLACKBOX_DEVICE_FLASH = 1,
#endif
#ifdef USE_SDCARD
    BLACKBOX_DEVICE_SDCARD = 2,
#endif

    BLACKBOX_DEVICE_END
} BlackboxDevice;


struct blackbox_config {
    uint8_t rate_num;
    uint8_t rate_denom;
    uint8_t device;
};

