#ifndef __I2C_INTERRUPT__
#define __I2C_INTERRUPT__

unsigned char I2C_Interrupt_Confing(unsigned long OutputClockFrequencyHz);
unsigned char I2C_Multiple_Write_With_Block(unsigned char slave_address,
		unsigned char reg_address, unsigned char *data_point,
		unsigned char data_len);
unsigned char I2C_Multiple_Read_With_Block(unsigned char slave_address,
		unsigned char reg_address, unsigned char *data_point,
		unsigned char data_len);
void I2C_Interrupt_Handle(void);
void I2C_Timer_Interrupt_Handle(void);

#endif /* __I2C_MST_INT_H */

