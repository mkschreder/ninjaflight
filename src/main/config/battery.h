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

typedef enum {
    CURRENT_SENSOR_NONE = 0,
    CURRENT_SENSOR_ADC,
    CURRENT_SENSOR_VIRTUAL,
    CURRENT_SENSOR_MAX = CURRENT_SENSOR_VIRTUAL
} current_sensor_type_t;

struct battery_config {
    uint8_t vbatscale;                      // adjust this to match battery voltage to reported value
    uint8_t vbatresdivval;                  // resistor divider R2 (default NAZE 10(K))
    uint8_t vbatresdivmultiplier;           // multiplier for scale (e.g. 2.5:1 ratio with multiplier of 4 can use '100' instead of '25' in ratio) to get better precision
    uint8_t vbatmaxcellvoltage;             // maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, default is 43 (4.3V)
    uint8_t vbatmincellvoltage;             // minimum voltage per cell, this triggers battery critical alarm, in 0.1V units, default is 33 (3.3V)
    uint8_t vbatwarningcellvoltage;         // warning voltage per cell, this triggers battery warning alarm, in 0.1V units, default is 35 (3.5V)

    int16_t currentMeterScale;             // scale the current sensor output voltage to milliamps. Value in 1/10th mV/A
    uint16_t currentMeterOffset;            // offset of the current sensor in millivolt steps
    current_sensor_type_t  currentMeterType;      // type of current meter used, either ADC or virtual

    // FIXME this doesn't belong in here since it's a concern of MSP, not of the battery code.
    uint8_t multiwiiCurrentMeterOutput;     // if set to 1 output the amperage in milliamp steps instead of 0.01A steps via msp
    uint16_t batteryCapacity;               // mAh
};

PG_DECLARE(struct battery_config, batteryConfig);