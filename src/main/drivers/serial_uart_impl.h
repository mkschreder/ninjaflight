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

// device specific uart implementation is defined here

typedef struct {
    serialPort_t port;

    DMA_Channel_TypeDef *rxDMAChannel;
    DMA_Channel_TypeDef *txDMAChannel;

    uint32_t rxDMAIrq;
    uint32_t txDMAIrq;

    uint32_t rxDMAPos;
    bool txDMAEmpty;

    uint32_t txDMAPeripheralBaseAddr;
    uint32_t rxDMAPeripheralBaseAddr;

    USART_TypeDef *USARTx;
} uartPort_t;



void uartStartTxDMA(uartPort_t *s);

uartPort_t *serialUART1(uint32_t baudRate, portMode_t mode, portOptions_t options);
uartPort_t *serialUART2(uint32_t baudRate, portMode_t mode, portOptions_t options);
uartPort_t *serialUART3(uint32_t baudRate, portMode_t mode, portOptions_t options);
uartPort_t *serialUART4(uint32_t baudRate, portMode_t mode, portOptions_t options);
uartPort_t *serialUART5(uint32_t baudRate, portMode_t mode, portOptions_t options);

