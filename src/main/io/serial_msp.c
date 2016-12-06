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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "build_config.h"
#include <platform.h>
#include "target.h"

#include "common/streambuf.h"
#include "common/utils.h"

#include "config/feature.h"
#include "config/config.h"

#include "drivers/serial.h"
#include "drivers/system.h"
#include "drivers/pwm_output.h"

#include "io/serial.h"
#include "io/serial_msp.h"
#include "io/serial_4way.h"
#include "cli.h"
#include "msp.h"
#include "ninja.h"


// assign serialPort to mspPort
// free mspPort when serialPort is NULL
static void resetMspPort(struct serial_msp_port *mspPortToReset, serialPort_t *serialPort)
{
    memset(mspPortToReset, 0, sizeof(struct serial_msp_port));

    mspPortToReset->port = serialPort;
}

static struct serial_msp_port* mspPortFindFree(struct serial_msp *self)
{
    for(int i = 0; i < MAX_MSP_PORT_COUNT; i++)
        if(self->ports[i].port == NULL)
            return &self->ports[i];
    return NULL;
}

static void serial_msp_alloc_ports(struct serial_msp *self){
    for(const struct serial_port_config *portConfig = findSerialPortConfig(&self->config->serial, FUNCTION_MSP);
        portConfig != NULL;
        portConfig = findNextSerialPortConfig(&self->config->serial, FUNCTION_MSP)) {
        if(isSerialPortOpen(portConfig))
            continue; // port is already open

        // find unused mspPort for this serial
        struct serial_msp_port *mspPort = mspPortFindFree(self);
        if(mspPort == NULL) {
            // no mspPort available, give up
            // this error should be signalized to user (invalid configuration)
            return;
        }
        serialPort_t *serialPort = openSerialPort(portConfig->identifier, FUNCTION_MSP, NULL,
                                                  baudRates[portConfig->msp_baudrateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);
        if (serialPort) {
            resetMspPort(mspPort, serialPort);
        } else {
            // unexpected error, inform user
        }
    }
}

void serial_msp_release_port(struct serial_msp *self, serialPort_t *serialPort)
{
    for (int i = 0; i < MAX_MSP_PORT_COUNT; i++) {
        struct serial_msp_port *mspPort = &self->ports[i];
        if (mspPort->port == serialPort) {
            closeSerialPort(mspPort->port);
            resetMspPort(mspPort, NULL);
        }
    }
}

void serial_msp_init(struct serial_msp *self, const struct config *config, struct msp *msp){
	memset(self, 0, sizeof(*self));
	self->config = config;
	self->msp = msp;
    for(int i = 0; i < MAX_MSP_PORT_COUNT; i++)
        resetMspPort(&self->ports[i], NULL);
    serial_msp_alloc_ports(self);
}

static uint8_t mspSerialChecksum(uint8_t checksum, uint8_t byte)
{
    return checksum ^ byte;
}

static uint8_t mspSerialChecksumBuf(uint8_t checksum, uint8_t *data, int len)
{
    while(len-- > 0)
        checksum = mspSerialChecksum(checksum, *data++);
    return checksum;
}

static void mspSerialResponse(struct serial_msp_port *msp, mspPacket_t *reply)
{
    serialBeginWrite(msp->port);
    int len = sbufBytesRemaining(&reply->buf);
    uint8_t hdr[] = {'$', 'M', reply->result < 0 ? '!' : '>', len, reply->cmd};
    uint8_t csum = 0;                                       // initial checksum value
    serialWriteBuf(msp->port, hdr, sizeof(hdr));
    csum = mspSerialChecksumBuf(csum, hdr + 3, 2);          // checksum starts from len field
    if(len > 0) {
        serialWriteBuf(msp->port, sbufPtr(&reply->buf), len);
        csum = mspSerialChecksumBuf(csum, sbufPtr(&reply->buf), len);
    }
    serialWrite(msp->port, csum);
    serialEndWrite(msp->port);
}

static void mspSerialProcessReceivedCommand(struct serial_msp_port *msp, struct msp *processor){
    mspPacket_t command = {
        .buf = {
            .ptr = msp->inBuf,
            .end = msp->inBuf + msp->dataSize,
        },
        .cmd = msp->cmdMSP,
        .result = 0,
    };

    static uint8_t outBuf[MSP_PORT_OUTBUF_SIZE];
    mspPacket_t reply = {
        .buf = {
            .ptr = outBuf,
            .end = ARRAYEND(outBuf),
        },
        .cmd = -1,
        .result = 0,
    };
    if(msp_process(processor, &command, &reply)) {
        // reply should be sent back
        sbufSwitchToReader(&reply.buf, outBuf);     // change streambuf direction
        mspSerialResponse(msp, &reply);
    }
    msp->c_state = IDLE;
}

static bool mspSerialProcessReceivedByte(struct serial_msp_port *msp, uint8_t c)
{
    switch(msp->c_state) {
        default:                 // be conservative with unexpected state
        case IDLE:
            if (c != '$')        // wait for '$' to start MSP message
                return false;
            msp->c_state = HEADER_M;
            break;
        case HEADER_M:
            msp->c_state = (c == 'M') ? HEADER_ARROW : IDLE;
            break;
        case HEADER_ARROW:
            msp->c_state = (c == '<') ? HEADER_SIZE : IDLE;
            break;
        case HEADER_SIZE:
            if (c > MSP_PORT_INBUF_SIZE) {
                msp->c_state = IDLE;
            } else {
                msp->dataSize = c;
                msp->offset = 0;
                msp->c_state = HEADER_CMD;
            }
            break;
        case HEADER_CMD:
            msp->cmdMSP = c;
            msp->c_state = HEADER_DATA;
            break;
        case HEADER_DATA:
            if(msp->offset < msp->dataSize) {
                msp->inBuf[msp->offset++] = c;
            } else {
                uint8_t checksum = 0;
                checksum = mspSerialChecksum(checksum, msp->dataSize);
                checksum = mspSerialChecksum(checksum, msp->cmdMSP);
                checksum = mspSerialChecksumBuf(checksum, msp->inBuf, msp->dataSize);
                if(c == checksum)
                    msp->c_state = COMMAND_RECEIVED;
                else
                    msp->c_state = IDLE;
            }
            break;
		case COMMAND_RECEIVED:
			break;
    }
    return true;
}

void serial_msp_process(struct serial_msp *self, struct ninja *ninja){
    for (int i = 0; i < MAX_MSP_PORT_COUNT; i++) {
        struct serial_msp_port *msp = &self->ports[i];
        if (!msp->port) {
            continue;
        }

        while (serialRxBytesWaiting(msp->port)) {
            uint8_t c = serialRead(msp->port);
            bool consumed = mspSerialProcessReceivedByte(msp, c);

            if (!consumed) {
				// TODO: this is madness. There should be an msp message for entering cli
				if (c == '#') {
					cli_start(&ninja->cli, msp->port);
				}
				if (c == self->config->serial.reboot_character) {
					systemResetToBootloader();
				}
            }

            if (msp->c_state == COMMAND_RECEIVED) {
                mspSerialProcessReceivedCommand(msp, self->msp);
                break; // process one command at a time so as not to block and handle modal command immediately
            }
        }
#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
		// TODO: 4way interface
		/*
        if(self->mspEnterEsc4way) {
            self->mspEnterEsc4way = false;
            waitForSerialPortToFinishTransmitting(msp->port);
            // esc4wayInit() was called in msp command
            // modal switch to esc4way, will return only after 4way exit command
            // port parameters are shared with esc4way, no need to close/reopen it
            esc4wayProcess(msp->port);
            // continue processing
        }
*/
#endif
		// TODO: this should probably be in the main msp class
		/*
        if (self->isRebootScheduled) {
            waitForSerialPortToFinishTransmitting(msp->port);  // TODO - postpone reboot, allow all modules to react
			// TODO: remove the ifdef once we have refactored pwm
#ifndef UNIT_TEST
            pwmStopMotors(feature(FEATURE_ONESHOT125));
#endif
			// TODO: oneshot reset
            //handleOneshotFeatureChangeOnRestart();
            systemReset();
        }*/
    }
}
