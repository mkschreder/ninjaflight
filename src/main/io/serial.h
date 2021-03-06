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

// TODO: remove dependency on serial driver
#include "drivers/serial.h"
#include "target.h"
#include "../config/serial.h"

extern const uint32_t baudRates[];

extern const serialPortIdentifier_e serialPortIdentifiers[SERIAL_PORT_COUNT];

void serialInit(bool softserialEnabled);

//
// runtime
//

typedef struct serialPortUsage_s {
    serialPortIdentifier_e identifier;
    serialPort_t *serialPort;
    serialPortFunction_e function;
} serialPortUsage_t;

serialPort_t *findSharedSerialPort(const struct serial_config *self, uint16_t functionMask, serialPortFunction_e sharedWithFunction);
serialPort_t *findNextSharedSerialPort(const struct serial_config *self, uint16_t functionMask, serialPortFunction_e sharedWithFunction);

void serialRemovePort(serialPortIdentifier_e identifier);
uint8_t serialGetAvailablePortCount(void);
bool serialIsPortAvailable(serialPortIdentifier_e identifier);
bool isSerialConfigValid(const struct serial_config *serialConfig);
const struct serial_port_config *serialFindPortConfiguration(const struct serial_config *self, serialPortIdentifier_e identifier);
bool doesConfigurationUsePort(const struct serial_config *self, serialPortIdentifier_e portIdentifier);

const struct serial_port_config *findSerialPortConfig(const struct serial_config *self, serialPortFunction_e function);
const struct serial_port_config *findNextSerialPortConfig(const struct serial_config *self, serialPortFunction_e function);
portSharing_e determinePortSharing(const struct serial_port_config *portConfig, serialPortFunction_e function);
bool isSerialPortShared(const struct serial_port_config *portConfig, uint16_t functionMask, serialPortFunction_e sharedWithFunction);
bool isSerialPortOpen(const struct serial_port_config *portConfig);


//
// runtime
//
serialPort_t *openSerialPort(
    serialPortIdentifier_e identifier,
    serialPortFunction_e function,
    serialReceiveCallbackPtr callback,
    uint32_t baudrate,
    portMode_t mode,
    portOptions_t options
);
void closeSerialPort(serialPort_t *serialPort);

void waitForSerialPortToFinishTransmitting(serialPort_t *serialPort);

baudRate_e lookupBaudRateIndex(uint32_t baudRate);


//
// msp/cli/bootloader
//
void evaluateOtherData(serialPort_t *serialPort, uint8_t receivedChar);
void handleSerial(void);
