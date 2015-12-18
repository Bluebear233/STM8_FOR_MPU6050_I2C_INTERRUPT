/*
 * test.h
 *
 *  Created on: 2015年12月15日
 *      Author: Peng
 */

#ifndef SRC_SYSTEM_TEST_H_
#define SRC_SYSTEM_TEST_H_

unsigned char I2C_Config();
unsigned char I2C_Multiple_Read(unsigned char slave_address,
		unsigned char reg_address, unsigned char * buff_point,
		unsigned char read_len);
unsigned char I2C_Multiple_Write_With_Block(unsigned char slave_address,
		unsigned char reg_address, unsigned char *buff_point,
		unsigned char data_len);

#endif /* SRC_SYSTEM_TEST_H_ */
