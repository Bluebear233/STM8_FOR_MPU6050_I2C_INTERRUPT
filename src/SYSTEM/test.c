///*
// * test.c
// *
// *  Created on: 2015年12月15日
// *      Author: Peng
// */
//#include "i2c_master_interrupt.h"
//#include <stdio.h>
//#include "test.h"
//
//uint8_t I2C_Config() {
//
//	// Initialize I2C for communication
//	I2C_Interrupt_Confing();
//
//	// Enable all interrupt
//	enableInterrupts();
//	return 0;
//}
//uint8_t I2C_Multiple_Read(uint8_t slave_address, uint8_t reg_address,
//		uint8_t * buff_point, uint8_t read_len) {
//
//	//I2C_WriteRegister(slave_address, reg_address, NOSTOP, 0, &reg_address);
//
//	//I2C_ReadRegister(slave_address, reg_address, NOSTOP, read_len, buff_point);
//
////	extern uint8_t STATE;
////
////	while (STATE != INI_00)
////		;
//
//	return 0;
//}
//
////uint8_t I2C_Multiple_Write(uint8_t slave_address, uint8_t reg_address,
////		uint8_t *buff_point, uint8_t data_len) {
////
////	while (!I2C_WriteRegister(slave_address, reg_address, STOP, data_len,
////			buff_point))
////		;
////	extern uint8_t STATE;
////	while (STATE != INI_00)
////		;
////
////	return 0;
////
////}
