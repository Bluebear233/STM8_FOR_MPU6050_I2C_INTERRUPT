/**
 ******************************************************************************
 * @file    i2c_master_interrupt.c
 * @author  MCD Application Team
 * @version V0.0.3
 * @date    Feb 2010
 * @brief   This file contains all I2c function for optimized I2C master
 ******************************************************************************
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 *                COPYRIGHT 2009 STMicroelectronics
 */

#include "stm8s.h"
#include "i2c_master_interrupt.h"
#include "TIM4.h"

// Global Variables definition

uint8_t g_STATE;
uint8_t g_Data_Len;
uint8_t *g_Data_Point;
uint8_t g_Slave_Address;
uint8_t g_Reg_Address;
uint8_t g_RW_State;

/******************************************************************************
 * Function name : I2C_Init
 * Description 	: Initialize I2C peripheral
 * Input param 	: None
 * Return 		    : None
 * See also 		  : None
 *******************************************************************************/
uint8_t I2C_Interrupt_Confing(void) {

	I2C_DeInit();

	I2C_Init(400000, 0x0000, I2C_DUTYCYCLE_16_9, I2C_ACK_CURR, I2C_ADDMODE_7BIT,
			16);

	I2C_ITConfig(I2C_IT_ERR | I2C_ITR_ITEVTEN, ENABLE);

	enableInterrupts();

	// Initialise I2C State Machine
	g_STATE = INI_00;

	return 0;
}
uint8_t I2C_Multiple_Write_With_Block(uint8_t slave_address,
		uint8_t reg_address, uint8_t data_len, u8 *data_point) {
	//等待上次通信结束
	while (g_STATE != INI_00)
		;
	//等待I2C空闲
	while ((I2C->SR3 & I2C_SR3_BUSY) == I2C_SR3_BUSY)
		;
	// copy parametters for interrupt routines
	g_RW_State = WRITE;
	g_Reg_Address = reg_address;
	g_Slave_Address = slave_address;
	g_Data_Len = data_len;
	g_Data_Point = data_point;
	// generate Start
	I2C->CR2 |= I2C_CR2_START;
	g_STATE = SB_01;
	while (g_STATE != INI_00)
		;
	return 0;
}

/******************************************************************************
 * Function name : I2C_ReadRegister
 * Description 	: Read defined number bytes from slave memory starting with defined offset
 * Input param 	: Slave Address ; Address type (TEN_BIT_ADDRESS or SEV_BIT_ADDRESS) ; STOP/NOSTOP ;
 *									Number byte to Read ; address of the application receive buffer
 * Return 		    : 0 : START Reading not performed -> Communication onging on the bus
 *                 1 : START Reading performed 
 * See also 		  : None
 *******************************************************************************/
uint8_t I2C_Multiple_Read_With_Block(uint8_t slave_address, uint8_t reg_address,
		u8 data_len, u8 *data_point) {
	//等待上次通信结束
	while (g_STATE != INI_00)
		;
	//等待I2C空闲
	while ((I2C->SR3 & I2C_SR3_BUSY) == I2C_SR3_BUSY)
		;
	// set ACK
	I2C->CR2 |= I2C_CR2_ACK;
	// reset POS
	I2C->CR2 &= ~I2C_CR2_POS;
	// setup I2C comm. in Read
	// copy parametters for interrupt routines
	g_RW_State = READ;
	g_Reg_Address = reg_address;
	g_Slave_Address = slave_address;
	g_Data_Len = data_len;
	g_Data_Point = data_point;
	I2C->CR2 |= I2C_CR2_START;
	g_STATE = SB_01;
	while (g_STATE != INI_00)
		;

	return 0;
}

/******************************************************************************
 * Function name : I2CInterruptHandle
 * Description 	: Manage all I2C interrupt (STATE MACHINE) 
 * Input param 	: None
 * Return 		    : None
 * See also 		  : None
 *******************************************************************************/
void I2CInterruptHandle(void) {

	uint8_t sr1;

	/* Get Value of Status registers and Control register 2 */
	sr1 = I2C->SR1;

	/* Start bit detected */
	if ((sr1 & I2C_SR1_SB) == I2C_SR1_SB) {
		switch (g_STATE) {
		case SB_01:
			I2C->DR = (g_Slave_Address << 1); // send 7-bit device address & Write (R/W = 0)
			g_STATE = ADDR_03;
			break;

		case SB_11:
			I2C->DR = (g_Slave_Address << 1) | 1; // send 7-bit device address & Write (R/W = 1)
			g_STATE = ADDR_13;
			break;
		}

	}

	/* ADDR*/
	if ((sr1 & I2C_SR1_ADDR) == I2C_SR1_ADDR) {
		switch (g_STATE) {
		case ADDR_13:
			switch (g_Data_Len) {
			case 1:
				I2C->CR2 &= ~I2C_CR2_ACK;
				/* Clear Add Ack Flag */
				I2C->SR3;
				I2C->CR2 |= I2C_CR2_STOP;
				I2C->ITR |= I2C_ITR_ITBUFEN;
				g_STATE = RXNE_18;
				break;
			case 2:
				// set POS bit
				I2C->CR2 |= I2C_CR2_POS;
				/* Clear Add Ack Flag */
				I2C->SR3;
				// set No ACK
				I2C->CR2 &= ~I2C_CR2_ACK;
				g_STATE = BTF_17;
				break;
			case 3:
				I2C->SR3;
				g_STATE = BTF_15;
				break;
			default:
				I2C->SR3;
				g_STATE = BTF_14;
				break;
			}
			break;

		case ADDR_03:
			/* Clear Add Ack Flag */
			I2C->SR3;
			I2C->DR = g_Reg_Address;
			if (g_RW_State == WRITE)
				g_STATE = BTF_04;
			else
				g_STATE = BTF_05;
			break;

		}
	}

	if ((sr1 & I2C_SR1_RXNE) == I2C_SR1_RXNE) {
		switch (g_STATE) {
		case RXNE_18:
			*(g_Data_Point++) = I2C->DR;			// Read next data byte
			g_STATE = INI_00;
			break;
		case RXNE_16:
			*(g_Data_Point++) = I2C->DR;            // Read next data byte
			g_STATE = INI_00;
			break;
		}
		I2C->ITR &= ~I2C_ITR_ITBUFEN;  // Disable Buffer interrupts (errata)
	}

	/* BTF */
	if ((sr1 & I2C_SR1_BTF) == I2C_SR1_BTF) {
		switch (g_STATE) {
		case BTF_17:
			I2C->CR2 |= I2C_CR2_STOP;     // generate stop request here (STOP=1)
			*(g_Data_Point++) = I2C->DR;			// Read next data byte
			*(g_Data_Point++) = I2C->DR;			// Read next data byte
			g_STATE = INI_00;
			break;

		case BTF_14:
			*(g_Data_Point++) = I2C->DR;
			g_Data_Len--;
			if (g_Data_Len <= 3)
				g_STATE = BTF_15;
			break;

		case BTF_15:
			I2C->CR2 &= ~I2C_CR2_ACK;                     	// Set NACK (ACK=0)
			*(g_Data_Point++) = I2C->DR;            // Read next data byte
			I2C->CR2 |= I2C_CR2_STOP;             // Generate stop here (STOP=1)
			*(g_Data_Point++) = I2C->DR;            // Read next data byte
			I2C->ITR |= I2C_ITR_ITBUFEN; 	// Enable Buffer interrupts (errata)
			g_STATE = RXNE_16;
			break;

		case BTF_04:
			if ((g_Data_Len) &&((I2C->SR1 & I2C_SR1_TXE) == I2C_SR1_TXE)) {
				I2C->DR = *g_Data_Point++;		// Write next data byte
				g_Data_Len--;
			} else {
				I2C->CR2 |= I2C_CR2_STOP;     // Generate stop here (STOP=1)
				g_STATE = INI_00;
			}
			break;

		case BTF_05:
			I2C->CR2 |= 1;
			g_STATE = SB_11;
			I2C->ITR |= 3;
			break;

		}
	}
}

