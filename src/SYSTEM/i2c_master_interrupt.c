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

// Global Variables definition

u8 STATE;								// curent I2C states machine state
volatile u8 err_state;  	// error state 
volatile u8 err_save;   	// I2C->SR2 copy in case of error
volatile u16 TIM4_tout;  // Timout Value  

u8 u8_regAddr;
u8 u8_Direction;

u8 u8_NumByte_cpy;
u8* pu8_DataBuffer_cpy;
u16 u16_SlaveAdd_cpy;
u8 u8_AddType_cpy;
u8 u8_NoStop_cpy;

/******************************************************************************
 * Function name : I2C_Init
 * Description 	: Initialize I2C peripheral
 * Input param 	: None
 * Return 		    : None
 * See also 		  : None
 *******************************************************************************/
void I2C_Interrupt_Confing(void) {

	I2C_DeInit();

	I2C_Init(400000, 0x0000, I2C_DUTYCYCLE_16_9, I2C_ACK_CURR, I2C_ADDMODE_7BIT,
			16);

	I2C_ITConfig(I2C_IT_ERR | I2C_ITR_ITEVTEN, ENABLE);

	// Initialise I2C State Machine
	err_save = 0;
	STATE = INI_00;
	set_tout_ms(0);
}

/******************************************************************************
 * Function name : ErrProc
 * Description 	: Managed Error durring I2C communication to be modified depending of final application
 * Input param 	: None
 * Return 		    : None
 * See also 		  : None.
 *******************************************************************************/
void ErrProc(void) {
	err_save = I2C->SR2;
	err_state = STATE;
	I2C->SR2 = 0;
	STATE = INI_00;
	set_tout_ms(0);
}

/******************************************************************************
 * Function name : I2C_WriteRegister
 * Description 	: write defined number bytes to slave memory starting with defined offset
 * Input param 	: Slave Address ; Address type (TEN_BIT_ADDRESS or SEV_BIT_ADDRESS) ; STOP/NOSTOP ;
 *									Number byte to Write ; address of the application send buffer
 * Return 		    : 0 : START Writing not performed -> Communication onging on the bus
 *                 1 : START Writing performed 
 * See also 		  : None.
 *******************************************************************************/
u8 I2C_WriteRegister(u16 u16_SlaveAdd, u8 u8_AddType, u8 u8_NoStop,
		u8 u8_NumByteToWrite, u8 *pu8_DataBuffer) {
	// check if communication on going
	if ((I2C->SR3 & I2C_SR3_BUSY) == I2C_SR3_BUSY)
		return 0;
	// check if STATE MACHINE is in state INI_00
	if (STATE != INI_00)
		return 0;
	// set ACK
	I2C->CR2 |= I2C_CR2_ACK;
	// reset POS
	I2C->CR2 &= ~I2C_CR2_POS;
	// setup I2C comm. in write
	u8_Direction = WRITE;
	// copy parametters for interrupt routines
	u8_NoStop_cpy = u8_NoStop;
	u16_SlaveAdd_cpy = u16_SlaveAdd;
	u8_NumByte_cpy = u8_NumByteToWrite;
	pu8_DataBuffer_cpy = pu8_DataBuffer;
	u8_AddType_cpy = u8_AddType;
	// set comunication Timeout
	set_tout_ms (I2C_TOUT);
	// generate Start
	I2C->CR2 |= I2C_CR2_START;
	STATE = SB_01;
	return 1;
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
u8 I2C_ReadRegister(u16 u16_SlaveAdd, u8 u8_AddType, u8 u8_NoStop,
		u8 u8_NumByteToRead, u8 *u8_DataBuffer) {
	// check if communication on going
	if (((I2C->SR3 & I2C_SR3_BUSY) == I2C_SR3_BUSY) && (u8_NoStop == 0))
		return 0;
	// check if STATE MACHINE is in state INI_00
	if (STATE != INI_00)
		return 0;
	// set ACK
	I2C->CR2 |= I2C_CR2_ACK;
	// reset POS
	I2C->CR2 &= ~I2C_CR2_POS;
	// setup I2C comm. in Read
	u8_Direction = READ;
	// copy parametters for interrupt routines
	u8_NoStop_cpy = u8_NoStop;
	u8_AddType_cpy = u8_AddType;
	u16_SlaveAdd_cpy = u16_SlaveAdd;
	u8_NumByte_cpy = u8_NumByteToRead;
	pu8_DataBuffer_cpy = u8_DataBuffer;
	// set comunication Timeout
	set_tout_ms (I2C_TOUT);
	//generate Start
	I2C->CR2 |= 1;
	STATE = SB_11;
	I2C->ITR |= 3;                  // re-enable interrupt
	return 1;
}

/******************************************************************************
 * Function name : I2CInterruptHandle
 * Description 	: Manage all I2C interrupt (STATE MACHINE) 
 * Input param 	: None
 * Return 		    : None
 * See also 		  : None
 *******************************************************************************/
void I2CInterruptHandle(void) {

	u8 sr1, sr2, cr2;

	/* Get Value of Status registers and Control register 2 */
	sr1 = I2C->SR1;
	sr2 = I2C->SR2;
	cr2 = I2C->CR2;

	/* Check for error in communication */
	if (sr2 != 0) {
		ErrProc();
	}

	/* Start bit detected */
	if ((sr1 & I2C_SR1_SB) == 1) {
		switch (STATE) {
		case SB_01:
			if (u8_AddType_cpy == TEN_BIT_ADDRESS) {
				I2C->DR = (u8)(((u16_SlaveAdd_cpy >> 7) & 6) | 0xF0); // send header of 10-bit device address (R/W = 0)
				STATE = ADD10_02;
				break;
			} else {
				I2C->DR = (u8)(u16_SlaveAdd_cpy << 1); // send 7-bit device address & Write (R/W = 0)
				STATE = ADDR_03;
				break;
			}

		case SB_11:
			if (u8_AddType_cpy == TEN_BIT_ADDRESS) {
				I2C->DR = (u8)(((u16_SlaveAdd_cpy >> 7) & 6) | 0xF1); // send header of 10-bit device address (R/W = 1)
			} else {
				I2C->DR = (u8)(u16_SlaveAdd_cpy << 1) | 1; // send 7-bit device address & Write (R/W = 1)
			}
			STATE = ADDR_13;
			break;

		default:
			ErrProc();
			break;
		}

	}

	/* ADD10*/
	if (I2C->SR1 & I2C_SR1_ADD10) {
		switch (STATE) {
		case ADD10_02:
			I2C->DR = (u8)(u16_SlaveAdd_cpy); // send lower 8-bit device address & Write  
			STATE = ADDR_03;
			break;

		default:
			ErrProc();
			break;
		}
	}

	/* ADDR*/
	if ((sr1 & I2C_SR1_ADDR) == I2C_SR1_ADDR) {
		switch (STATE) {
		case ADDR_13:

			if (u8_NumByte_cpy == 3) {
				I2C->SR3;
				STATE = BTF_15;
				break;
			}

			if (u8_NumByte_cpy == 2) {
				// set POS bit
				I2C->CR2 |= I2C_CR2_POS;
				/* Clear Add Ack Flag */
				I2C->SR3;
				// set No ACK
				I2C->CR2 &= ~I2C_CR2_ACK;
				STATE = BTF_17;
				break;
			}
			if (u8_NumByte_cpy == 1) {
				I2C->CR2 &= ~I2C_CR2_ACK;
				/* Clear Add Ack Flag */
				I2C->SR3;
				I2C->CR2 |= I2C_CR2_STOP;
				I2C->ITR |= I2C_ITR_ITBUFEN;
				STATE = RXNE_18;
				break;
			}
			if (u8_NumByte_cpy > 3) {
				I2C->SR3;
				STATE = BTF_14;
				break;
			}
			ErrProc();
			break;

		case ADDR_03:

			/* Clear Add Ack Flag */
			I2C->SR3;
			I2C->DR = *pu8_DataBuffer_cpy++;
			u8_NumByte_cpy--;
			STATE = BTF_04;
			break;

		default:
			ErrProc();
			break;

		}
	}

	if ((sr1 & I2C_SR1_RXNE) == I2C_SR1_RXNE) {
		switch (STATE) {
		case RXNE_18:
			*(pu8_DataBuffer_cpy++) = I2C->DR;			// Read next data byte
			STATE = INI_00;
			set_tout_ms(0);
			break;
		case RXNE_16:
			*(pu8_DataBuffer_cpy++) = I2C->DR;            // Read next data byte
			STATE = INI_00;
			set_tout_ms(0);
			break;
		}
		I2C->ITR &= ~I2C_ITR_ITBUFEN;  // Disable Buffer interrupts (errata)
	}

	/* BTF */
	if ((sr1 & I2C_SR1_BTF) == I2C_SR1_BTF) {
		switch (STATE) {
		case BTF_17:
			I2C->CR2 |= I2C_CR2_STOP;     // generate stop request here (STOP=1)
			*(pu8_DataBuffer_cpy++) = I2C->DR;			// Read next data byte
			*(pu8_DataBuffer_cpy++) = I2C->DR;			// Read next data byte
			STATE = INI_00;
			set_tout_ms(0);
			break;

		case BTF_14:
			*(pu8_DataBuffer_cpy++) = I2C->DR;
			u8_NumByte_cpy--;
			if (u8_NumByte_cpy <= 3)
				STATE = BTF_15;
			break;

		case BTF_15:
			I2C->CR2 &= ~I2C_CR2_ACK;                     	// Set NACK (ACK=0)
			*(pu8_DataBuffer_cpy++) = I2C->DR;            // Read next data byte
			I2C->CR2 |= I2C_CR2_STOP;             // Generate stop here (STOP=1)
			*(pu8_DataBuffer_cpy++) = I2C->DR;            // Read next data byte
			I2C->ITR |= I2C_ITR_ITBUFEN; 	// Enable Buffer interrupts (errata)
			STATE = RXNE_16;
			break;

		case BTF_04:
			if ((u8_NumByte_cpy) &&((I2C->SR1 & I2C_SR1_TXE) == I2C_SR1_TXE)) {
				I2C->DR = *pu8_DataBuffer_cpy++;		// Write next data byte
				u8_NumByte_cpy--;
				break;
			} else {
				if (u8_NoStop_cpy == 0) {
					I2C->CR2 |= I2C_CR2_STOP;     // Generate stop here (STOP=1)
				} else {
					I2C->ITR = 0;                  // disable interrupt 
				}
				STATE = INI_00;
				set_tout_ms(0);
				break;
			}
		}
	}
}

/******************************************************************************
 * Function name : TIM4_Init
 * Description 	: Initialize TIM4 peripheral
 * Input param 	: None
 * Return 		    : None
 * See also 		  : None
 *******************************************************************************/
void TIM4_Init1(void) {
	CLK->PCKENR1 |= 4;               // TIM4 clock enable

	TIM4->ARR = 0x80;                // init timer4 1ms interrupts
	TIM4->PSCR = 7;
	TIM4->IER = 1;
	TIM4->CR1 |= 1;
}

/******************************************************************************
 * Function name : TIM4InterruptHandle
 * Description 	: Testing load for Main 
 * Input param 	: None
 * Return 		    : None
 * See also 		  : None
 *******************************************************************************/
void TIM4InterruptHandle(void) {

	u8 dly = 10;

	TIM4->SR1 = 0;

	if (TIM4_tout)
		if (--TIM4_tout == 0) {
			ErrProc();
		}
	while (dly--)
		;
}

