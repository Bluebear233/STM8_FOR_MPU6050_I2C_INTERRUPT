/**
 ******************************************************************************
 * @file     src/SYSTEM/I2C_INTERRUPT.c
 * @author   luzhipeng
 * @version  V1.2
 * @date     19-Dec-2015
 * @brief    This file containsall the functions for I2C Communication.
 ******************************************************************************
 @par How to use it ?

 1.实现Hard I2C PIN的硬件I2C引脚定义
 2.实现Timer Define的函数定义，定时器的中断优先级为3,定时器中断频率为1000HZ
 3.根据所需功能实现Funtions Define
 4.如果要从中断里调用I2C通信要确保该中断的优先级为1
 5.将I2C_Timer_Interrupt_Handle()和I2C_Interrupt_Handle()加入到中断函数里
 6.定义了DEBUG，应在I2C初始化前先初始化串口
 7.非阻塞函数使用时要注意是否会出现临界状态，如果传入的指针请确保I2C已经读写完毕方可修改
 通过I2C_Get_Communication_Complete()函数查询读写是否完毕，切记。

 @updata

 v1.0：新建
 v1.1：在I2C中断增加定时器状态判断
 v1.2：增加非阻塞的读写，


 */

#include <I2C_INTERRUPT.h>
#include "stm8s.h"
#include "TIM4.h"

/* Hard I2C PIN --------------------------------------------------------------*/
#define I2C_SDA_GPIO GPIOB
#define I2C_SCL_GPIO GPIOB
#define I2C_SDA_PIN GPIO_PIN_5
#define I2C_SCL_PIN GPIO_PIN_4

/* Timer Define --------------------------------------------------------------*/
#define I2C_Timer_Config          TIM4_Config
#define I2C_Timer_Set_1ms_Count() TIM4_SetCounter(0)
#define I2C_Timer_Clear_IT()      TIM4_ClearITPendingBit(TIM4_IT_UPDATE)
#define I2C_Timer_Cmd(state)      TIM4_Cmd(state)
#define I2C_Timer_Get_On_State()  (TIM4->CR1&TIM4_CR1_CEN)

/* Funtions Define -----------------------------------------------------------*/
#define DEBUG

/* Private Define ------------------------------------------------------------*/
#if defined DEBUG
#include <stdio.h>
#define I2C_PRINT_ERROR I2C_Print_Error
#define SET_I2C_CONFIG_STATE g_I2C_Config_State = 1
#else
#define I2C_PRINT_ERROR
#define SET_I2C_CONFIG_STATE
#endif

#define WRITE 0
#define READ  1

// Define I2C STATE MACHINE :

#define INI_00 00

// Write states 0x
#define SB_01 01
#define ADD10_02 02
#define ADDR_03 03
#define BTF_04 04
#define BTF_05 05

// Read states 1x
#define SB_11 11
#define ADD10_12 12
#define ADDR_13 13
#define BTF_14 14
#define BTF_15 15
#define RXNE_16 16
#define BTF_17 17
#define RXNE_18 18

/* Pivate variables ----------------------------------------------------------*/
#if defined DEBUG
uint8_t g_I2C_Config_State = 0;
#endif
uint8_t g_STATE;
uint8_t g_Data_Len;
uint8_t *g_Data_Point;
uint8_t g_Slave_Address;
uint8_t g_Reg_Address;
uint8_t g_RW_State;

/* Pivate Function -----------------------------------------------------------*/
static void I2C_Print_Error();

/**
 * @brief  I2C Interrupt Config
 * @return 0 is ok
 */
uint8_t I2C_Interrupt_Confing(uint32_t OutputClockFrequencyHz) {

	I2C_DeInit();

	I2C_Init(OutputClockFrequencyHz, 0x0000, I2C_DUTYCYCLE_16_9, I2C_ACK_CURR,
			I2C_ADDMODE_7BIT, 16);

	I2C_ITConfig(I2C_IT_ERR | I2C_ITR_ITEVTEN, ENABLE);

	TIM4_Config();

	ITC_SetSoftwarePriority(ITC_IRQ_I2C, ITC_PRIORITYLEVEL_2);

	enableInterrupts();

	g_STATE = INI_00;

	SET_I2C_CONFIG_STATE;

	return 0;
}
/**
 * @brief I2C Multiple Write With Block
 * @param  slave_address : slave address.
 * @param  reg_address : register address.
 * @param  buff_point ： want to read buffer point
 * @param  data_len : write data lenght
 * @retval communication state 0 is ok
 */
uint8_t I2C_Multiple_Write_With_Block(uint8_t slave_address,
		uint8_t reg_address, uint8_t *data_point, uint8_t data_len) {
	//等待上次通信结束
	while (g_STATE != INI_00)
		;
	I2C_Timer_Set_1ms_Count();
	I2C_Timer_Cmd(ENABLE);
	//等待I2C空闲
	while ((I2C->SR3 & I2C_SR3_BUSY) == I2C_SR3_BUSY)
		;
	I2C_Timer_Set_1ms_Count();
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
		//这里是等待完成
		;
	return 0;
}

/**
 * @brief  I2C Multiple Read With Block
 * @param  slave_address : slave address.
 * @param  reg_address : register address.
 * @param  buff_point ： want to write buffer point
 * @param  data_len : read lenght
 * @retval communication state 0 is ok
 */
uint8_t I2C_Multiple_Read_With_Block(uint8_t slave_address, uint8_t reg_address,
		uint8_t *data_point, uint8_t data_len) {
	//等待上次通信结束
	while (g_STATE != INI_00)
		;
	I2C_Timer_Set_1ms_Count();
	I2C_Timer_Cmd(ENABLE);
	//等待I2C空闲
	while ((I2C->SR3 & I2C_SR3_BUSY) == I2C_SR3_BUSY)
		;
	I2C_Timer_Set_1ms_Count();
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
		//这里是等待完成
		;

	return 0;
}
/**
 * @brief I2C Multiple Write Without Block
 * @param  slave_address : slave address.
 * @param  reg_address : register address.
 * @param  buff_point ： want to read buffer point
 * @param  data_len : write data lenght
 * @retval communication state 0 is ok
 */
uint8_t I2C_Multiple_Write_Without_Block(uint8_t slave_address,
		uint8_t reg_address, uint8_t *data_point, uint8_t data_len) {
	//等待上次通信结束
	while (g_STATE != INI_00)
		;
	I2C_Timer_Set_1ms_Count();
	I2C_Timer_Cmd(ENABLE);
	//等待I2C空闲
	while ((I2C->SR3 & I2C_SR3_BUSY) == I2C_SR3_BUSY)
		;
	I2C_Timer_Set_1ms_Count();
	// copy parametters for interrupt routines
	g_RW_State = WRITE;
	g_Reg_Address = reg_address;
	g_Slave_Address = slave_address;
	g_Data_Len = data_len;
	g_Data_Point = data_point;
	// generate Start
	I2C->CR2 |= I2C_CR2_START;
	g_STATE = SB_01;

	return 0;
}

/**
 * @brief  I2C Multiple Read Without Block
 * @param  slave_address : slave address.
 * @param  reg_address : register address.
 * @param  buff_point ： want to write buffer point
 * @param  data_len : read lenght
 * @retval communication state 0 is ok
 */
uint8_t I2C_Multiple_Read_Without_Block(uint8_t slave_address,
		uint8_t reg_address, uint8_t *data_point, uint8_t data_len) {
	//等待上次通信结束
	while (g_STATE != INI_00)
		;
	I2C_Timer_Set_1ms_Count();
	I2C_Timer_Cmd(ENABLE);
	//等待I2C空闲
	while ((I2C->SR3 & I2C_SR3_BUSY) == I2C_SR3_BUSY)
		;
	I2C_Timer_Set_1ms_Count();
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

	return 0;
}
/**
 * @brief I2C get communication complete
 * @retval  1 - Complete
 *          0 - Communicaing
 */
uint8_t I2C_Get_Communication_Complete(void) {
	return (g_STATE == INI_00 ? 1 : 0);
}
/**
 * @brief I2C_Interrupt_Handle
 */
void I2C_Interrupt_Handle(void) {
	uint8_t sr1;

	/* Get Value of Status registers and Control register 2 */
	sr1 = I2C->SR1;

	if (!I2C_Timer_Get_On_State()) {
		I2C_Timer_Set_1ms_Count();
		I2C_Timer_Cmd(ENABLE);
	}

	/* Start bit detected */
	if ((sr1 & I2C_SR1_SB) == I2C_SR1_SB) {
		switch (g_STATE) {
		case SB_01:
			I2C_Timer_Set_1ms_Count();
			I2C->DR = (g_Slave_Address << 1); // send 7-bit device address & Write (R/W = 0)
			g_STATE = ADDR_03;
			break;

		case SB_11:
			I2C_Timer_Set_1ms_Count();
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
				I2C_Timer_Set_1ms_Count();
				I2C->CR2 &= ~I2C_CR2_ACK;
				/* Clear Add Ack Flag */
				I2C->SR3;
				I2C->CR2 |= I2C_CR2_STOP;
				I2C->ITR |= I2C_ITR_ITBUFEN;
				g_STATE = RXNE_18;
				break;
			case 2:
				I2C_Timer_Set_1ms_Count();
				// set POS bit
				I2C->CR2 |= I2C_CR2_POS;
				/* Clear Add Ack Flag */
				I2C->SR3;
				// set No ACK
				I2C->CR2 &= ~I2C_CR2_ACK;
				g_STATE = BTF_17;
				break;
			case 3:
				I2C_Timer_Set_1ms_Count();
				I2C->SR3;
				g_STATE = BTF_15;
				break;
			default:
				I2C_Timer_Set_1ms_Count();
				I2C->SR3;
				g_STATE = BTF_14;
				break;
			}
			break;

		case ADDR_03:
			/* Clear Add Ack Flag */
			I2C_Timer_Set_1ms_Count();
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
			I2C_Timer_Cmd(DISABLE);
			*(g_Data_Point++) = I2C->DR;			// Read next data byte
			g_STATE = INI_00;
			break;
		case RXNE_16:
			I2C_Timer_Cmd(DISABLE);
			*(g_Data_Point++) = I2C->DR;			// Read next data byte
			g_STATE = INI_00;
			break;
		}
		I2C->ITR &= ~I2C_ITR_ITBUFEN;  // Disable Buffer interrupts (errata)
	}

	/* BTF */
	if ((sr1 & I2C_SR1_BTF) == I2C_SR1_BTF) {
		switch (g_STATE) {
		case BTF_17:
			I2C_Timer_Cmd(DISABLE);
			I2C->CR2 |= I2C_CR2_STOP;     // generate stop request here (STOP=1)
			*(g_Data_Point++) = I2C->DR;     // Read next data byte
			*(g_Data_Point++) = I2C->DR;     // Read next data byte
			g_STATE = INI_00;
			break;

		case BTF_14:
			I2C_Timer_Set_1ms_Count();
			*(g_Data_Point++) = I2C->DR;
			g_Data_Len--;
			if (g_Data_Len <= 3)
				g_STATE = BTF_15;
			break;

		case BTF_15:
			I2C_Timer_Set_1ms_Count();
			I2C->CR2 &= ~I2C_CR2_ACK;     // Set NACK (ACK=0)
			*(g_Data_Point++) = I2C->DR;     // Read next data byte
			I2C->CR2 |= I2C_CR2_STOP;     // Generate stop here (STOP=1)
			*(g_Data_Point++) = I2C->DR;     // Read next data byte
			I2C->ITR |= I2C_ITR_ITBUFEN;    // Enable Buffer interrupts (errata)
			g_STATE = RXNE_16;
			break;

		case BTF_04:
			if ((g_Data_Len) &&((I2C->SR1 & I2C_SR1_TXE) == I2C_SR1_TXE)) {
				I2C_Timer_Set_1ms_Count();
				I2C->DR = *g_Data_Point++;		// Write next data byte
				g_Data_Len--;
			} else {
				I2C_Timer_Cmd(DISABLE);
				I2C->CR2 |= I2C_CR2_STOP;     // Generate stop here (STOP=1)
				g_STATE = INI_00;
			}
			break;

		case BTF_05:
			I2C_Timer_Set_1ms_Count();
			I2C->CR2 |= 1;
			g_STATE = SB_11;
			I2C->ITR |= 3;
			break;

		}
	}
}
/**
 * @brief I2C Timer Interrupt Handle
 */
void I2C_Timer_Interrupt_Handle(void) {
	static uint8_t state = 0;
	switch (state) {
	case 0:
		I2C_PRINT_ERROR();
		I2C_DeInit();
		CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C, DISABLE);
		GPIO_Init(I2C_SDA_GPIO, I2C_SDA_PIN, GPIO_MODE_OUT_PP_HIGH_FAST);
		GPIO_Init(I2C_SCL_GPIO, I2C_SCL_PIN, GPIO_MODE_OUT_PP_HIGH_FAST);
		state = 1;
		break;
	case 255:
		IWDG_Enable();

		IWDG_WriteAccessCmd (IWDG_WriteAccess_Enable);

		IWDG_SetPrescaler (IWDG_Prescaler_4);

		IWDG_WriteAccessCmd (IWDG_WriteAccess_Disable);

		break;
	default:
		if (GPIO_ReadInputPin(I2C_SDA_GPIO, I2C_SDA_PIN) == RESET) {
			GPIO_WriteReverse(I2C_SCL_GPIO, I2C_SCL_PIN);
			state++;
		} else {
			state = 255;
		}

	}
	I2C_Timer_Clear_IT();
}
#ifdef DEBUG
/**
 * @brief I2C Print Error
 */
void I2C_Print_Error() {
	if (!g_I2C_Config_State) {
		printf("I2C ERROR:I2C NOT CONFIG \n\r\n");
		return;
	}
	switch (g_STATE) {
	case INI_00:
		printf("I2C ERROR:I2C WAS BUSY,PLEASE CHECK LINE COMMUNITION! \n\r\n");
		break;
	case SB_01:
		printf(
				"I2C ERROR:I2C WAS NO RESPONSE,PLEASE CHECK LINE COMMUNITION! \n\r\n");
		break;
	case ADDR_03:
		printf(
				"I2C ERROR:I2C NO FINE SLAVE,PLEASE CHECK SLAVE ADDRESS! \n\r\n");
		break;
	default:
		printf("I2C ERROR:I2C COMUNITION TIME OUT \n\r\n");
		break;
	}
}
#endif
