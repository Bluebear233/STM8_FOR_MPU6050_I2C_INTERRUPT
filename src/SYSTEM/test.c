/*
 * test.c
 *
 *  Created on: 2015年12月15日
 *      Author: Peng
 */
#include "i2c_master_interrupt.h"
#include <stdio.h>
#include "test.h"

uint8_t I2C_Config() {

	// Initialize I2C for communication
	I2C_Interrupt_Confing();

	// Enable all interrupt
	enableInterrupts();
	return 0;
}
uint8_t I2C_Multiple_Read(uint8_t slave_address, uint8_t reg_address,
		uint8_t * buff_point, uint8_t read_len) {

	while (!I2C_WriteRegister(slave_address, SEV_BIT_ADDRESS, NOSTOP, 1,
			&reg_address))
		;
	while (!I2C_ReadRegister(slave_address, SEV_BIT_ADDRESS, NOSTOP, read_len,
			buff_point))
		;

	extern uint8_t STATE;

	while (STATE != INI_00)
		;

	return 0;
}

uint8_t I2C_Multiple_Write(uint8_t slave_address, uint8_t reg_address,
		uint8_t *buff_point, uint8_t data_len) {

	static uint8_t buf[256];
	buf[0] = reg_address;
	for (uint8_t len = 0; len < data_len; len++) {
		buf[len + 1] = *buff_point;
		buff_point++;
	}

	while (!I2C_WriteRegister(slave_address, SEV_BIT_ADDRESS, STOP,
			data_len + 1, buf))
		;
	extern uint8_t STATE;
	while (STATE != INI_00)
		;

	return 0;

}
