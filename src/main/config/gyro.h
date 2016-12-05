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

struct gyro_config {
    uint8_t move_threshold;			//!< people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
    uint8_t gyro_lpf;                           //!< gyro LPF setting - values are driver specific, in case of invalid number, a reasonable default ~30-40HZ is chosen.
    uint16_t soft_gyro_lpf_hz;                  //!< Software based gyro filter in hz
};

