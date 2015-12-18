#ifndef __I2C_MST_INT_H
#define __I2C_MST_INT_H

#include "stm8s.h"

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

// Exported function 

uint8_t I2C_Interrupt_Confing(void);
void I2CInterruptHandle(void);

// Exported Interrupt handler 

#endif /* __I2C_MST_INT_H */

