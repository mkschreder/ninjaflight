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
#include <stdlib.h>

#include <platform.h>

#include "build_config.h"

#include "gpio.h"
#include "system.h"

#include "bus_i2c.h"
#include "nvic.h"

#ifndef SOFT_I2C

#define I2C1_SCL_GPIO        GPIOB
#define I2C1_SCL_GPIO_AF     GPIO_AF_4
#define I2C1_SCL_PIN         GPIO_Pin_6
#define I2C1_SCL_PIN_SOURCE  GPIO_PinSource6
#define I2C1_SCL_CLK_SOURCE  RCC_AHBPeriph_GPIOB
#define I2C1_SDA_GPIO        GPIOB
#define I2C1_SDA_GPIO_AF     GPIO_AF_4
#define I2C1_SDA_PIN         GPIO_Pin_7
#define I2C1_SDA_PIN_SOURCE  GPIO_PinSource7
#define I2C1_SDA_CLK_SOURCE  RCC_AHBPeriph_GPIOB

#if !defined(I2C2_SCL_GPIO)
#define I2C2_SCL_GPIO        GPIOF
#define I2C2_SCL_GPIO_AF     GPIO_AF_4
#define I2C2_SCL_PIN         GPIO_Pin_6
#define I2C2_SCL_PIN_SOURCE  GPIO_PinSource6
#define I2C2_SCL_CLK_SOURCE  RCC_AHBPeriph_GPIOF
#define I2C2_SDA_GPIO        GPIOA
#define I2C2_SDA_GPIO_AF     GPIO_AF_4
#define I2C2_SDA_PIN         GPIO_Pin_10
#define I2C2_SDA_PIN_SOURCE  GPIO_PinSource10
#define I2C2_SDA_CLK_SOURCE  RCC_AHBPeriph_GPIOA

#endif

static uint32_t i2cTimeout;

static volatile uint16_t i2c1ErrorCount = 0;
static volatile uint16_t i2c2ErrorCount = 0;

static I2C_TypeDef *I2Cx = NULL;

///////////////////////////////////////////////////////////////////////////////
// I2C TimeoutUserCallback
///////////////////////////////////////////////////////////////////////////////

static bool i2cOverClock;

void i2cSetOverclock(uint8_t OverClock)
{
    i2cOverClock = (OverClock) ? true : false;
}

static uint32_t i2cTimeoutUserCallback(I2C_TypeDef *dev)
{
    if (dev == I2C1) {
        i2c1ErrorCount++;
    } else {
        i2c2ErrorCount++;
    }
    return false;
}

static void i2cInitPort(I2C_TypeDef *dev)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;
	int irq_ev = 0, irq_er = 0;

    if (dev == I2C1) {
        RCC_AHBPeriphClockCmd(I2C1_SCL_CLK_SOURCE | I2C1_SDA_CLK_SOURCE, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
        RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);

        //i2cUnstick(dev);                                         // Clock out stuff to make sure slaves arent stuck

        GPIO_PinAFConfig(I2C1_SCL_GPIO, I2C1_SCL_PIN_SOURCE, I2C1_SCL_GPIO_AF);
        GPIO_PinAFConfig(I2C1_SDA_GPIO, I2C1_SDA_PIN_SOURCE, I2C1_SDA_GPIO_AF);

        GPIO_StructInit(&GPIO_InitStructure);
        I2C_StructInit(&I2C_InitStructure);

        // Init pins

        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

        GPIO_InitStructure.GPIO_Pin = I2C1_SCL_PIN;
        GPIO_Init(I2C1_SCL_GPIO, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin = I2C1_SDA_PIN;
        GPIO_Init(I2C1_SDA_GPIO, &GPIO_InitStructure);

		irq_ev = I2C1_EV_IRQn;
		irq_er = I2C1_EV_IRQn;
	} else if (dev == I2C2) {
        RCC_AHBPeriphClockCmd(I2C2_SCL_CLK_SOURCE | I2C2_SDA_CLK_SOURCE, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
        RCC_I2CCLKConfig(RCC_I2C2CLK_SYSCLK);

        //i2cUnstick(dev);                                         // Clock out stuff to make sure slaves arent stuck

        GPIO_PinAFConfig(I2C2_SCL_GPIO, I2C2_SCL_PIN_SOURCE, I2C2_SCL_GPIO_AF);
        GPIO_PinAFConfig(I2C2_SDA_GPIO, I2C2_SDA_PIN_SOURCE, I2C2_SDA_GPIO_AF);

        GPIO_StructInit(&GPIO_InitStructure);
        I2C_StructInit(&I2C_InitStructure);

        // Init pins
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

        GPIO_InitStructure.GPIO_Pin = I2C2_SCL_PIN;
        GPIO_Init(I2C2_SCL_GPIO, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin = I2C2_SDA_PIN;
        GPIO_Init(I2C2_SDA_GPIO, &GPIO_InitStructure);

		irq_ev = I2C2_EV_IRQn;
		irq_er = I2C2_EV_IRQn;
	} else {
		return;
	}

    I2C_StructInit(&I2C_InitStructure);

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	I2C_InitStructure.I2C_DigitalFilter = 0x00;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	if (i2cOverClock) {
		I2C_InitStructure.I2C_Timing = 0x00500E30; // 1000 Khz, 72Mhz Clock, Analog Filter Delay ON, Setup 40, Hold 4.
	} else {
		I2C_InitStructure.I2C_Timing = 0x00E0257A; // 400 Khz, 72Mhz Clock, Analog Filter Delay ON, Rise 100, Fall 10
	}
	//I2C_InitStructure.I2C_Timing              = 0x8000050B;
	I2C_Init(dev, &I2C_InitStructure);

	I2C_Cmd(dev, ENABLE);

    NVIC_InitTypeDef nvic;
	// I2C ER Interrupt
	nvic.NVIC_IRQChannel = irq_er;
	nvic.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_ER);
	nvic.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_ER);
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	// I2C EV Interrupt
	nvic.NVIC_IRQChannel = irq_ev;
	nvic.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_EV);
	nvic.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_EV);
	NVIC_Init(&nvic);
}

static void i2c_er_handler(void)
{
	if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BERR)){
		// bus error

	} else if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_ARLO)){
		// arbitration lost
	} else if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_OVR)){
		// overrun
	} else if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_PECERR)){
		// packet error checking error
	} else if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_TIMEOUT)){
		// SMBus timeout
	} else if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_ALERT)){
		// SMBus alert
	} else {
		return;
	}
}

void i2c_ev_handler(void)
{

}
void I2C1_ER_IRQHandler(void); 
void I2C1_EV_IRQHandler(void); 
void I2C2_ER_IRQHandler(void);
void I2C2_EV_IRQHandler(void); 

void I2C1_ER_IRQHandler(void)
{
    i2c_er_handler();
}

void I2C1_EV_IRQHandler(void)
{
    i2c_ev_handler();
}

void I2C2_ER_IRQHandler(void)
{
    i2c_er_handler();
}

void I2C2_EV_IRQHandler(void)
{
    i2c_ev_handler();
}


void i2c_init(void){
	i2cInit(I2C_DEVICE);
}

void i2cInit(I2CDevice index)
{
    if (index == I2CDEV_1) {
        I2Cx = I2C1;
    } else {
        I2Cx = I2C2;
    }
    i2cInitPort(I2Cx);
}

uint16_t i2cGetErrorCounter(void)
{
    if (I2Cx == I2C1) {
        return i2c1ErrorCount;
    }

    return i2c2ErrorCount;

}

bool i2cWrite(uint8_t addr_, uint8_t reg, uint8_t data)
{
    addr_ <<= 1;

    /* Test on BUSY Flag */
    i2cTimeout = I2C_DEFAULT_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_BUSY) != RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2Cx, addr_, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

    /* Wait until TXIS flag is set */
    i2cTimeout = I2C_DEFAULT_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Send Register address */
    I2C_SendData(I2Cx, (uint8_t) reg);

    /* Wait until TCR flag is set */
    i2cTimeout = I2C_DEFAULT_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TCR) == RESET)
    {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2Cx, addr_, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);

    /* Wait until TXIS flag is set */
    i2cTimeout = I2C_DEFAULT_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Write data to TXDR */
    I2C_SendData(I2Cx, data);

    /* Wait until STOPF flag is set */
    i2cTimeout = I2C_DEFAULT_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_STOPF) == RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Clear STOPF flag */
    I2C_ClearFlag(I2Cx, I2C_ICR_STOPCF);

    return true;
}

bool i2cRead(uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf)
{
    addr_ <<= 1;

    /* Test on BUSY Flag */
    i2cTimeout = I2C_DEFAULT_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_BUSY) != RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2Cx, addr_, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

    /* Wait until TXIS flag is set */
    i2cTimeout = I2C_DEFAULT_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Send Register address */
    I2C_SendData(I2Cx, (uint8_t) reg);

    /* Wait until TC flag is set */
    i2cTimeout = I2C_DEFAULT_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TC) == RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2Cx, addr_, len, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

    /* Wait until all data are received */
    while (len) {
        /* Wait until RXNE flag is set */
        i2cTimeout = I2C_DEFAULT_TIMEOUT;
        while (I2C_GetFlagStatus(I2Cx, I2C_ISR_RXNE) == RESET) {
            if ((i2cTimeout--) == 0) {
                return i2cTimeoutUserCallback(I2Cx);
            }
        }

        /* Read data from RXDR */
        *buf = I2C_ReceiveData(I2Cx);
        /* Point to the next location where the byte read will be saved */
        buf++;

        /* Decrement the read bytes counter */
        len--;
    }

    /* Wait until STOPF flag is set */
    i2cTimeout = I2C_DEFAULT_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_STOPF) == RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Clear STOPF flag */
    I2C_ClearFlag(I2Cx, I2C_ICR_STOPCF);

    /* If all operations OK */
    return true;
}

#endif
