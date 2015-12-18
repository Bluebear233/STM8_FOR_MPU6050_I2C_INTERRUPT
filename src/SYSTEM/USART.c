#include "USART.h"
#include "stm8s.h"
#include <stdio.h>

/* Private funcitons ****************************************************/
int putchar(int c);
/* Public functions *****************************************************/
/**
* @brief  Configure USART peripheral to print characters on Hyperteminal
* @param  None
* @retval None
*/
void USART_Config(void)
{
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_UART1, ENABLE);
    
    UART2_DeInit();
    /* USART configuration ------------------------------------------------------*/
    /* USART configured as follow:
    - BaudRate = 256000 baud  
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - transmit enabled
    */
    UART2_Init((uint32_t)256000, UART2_WORDLENGTH_8D, UART2_STOPBITS_1,
               UART2_PARITY_NO, UART2_SYNCMODE_CLOCK_DISABLE, 
               (UART2_Mode_TypeDef)(UART2_MODE_TX_ENABLE));
}


/**
* @brief  Retargets the C library printf function to the USART.
* @param  c Character to send
* @retval char Character sent
*/
int putchar(int c)  
{  
    /* Write a character to the USART */
    UART2_SendData8(c);
    
    /* Loop until the end of transmission */
    while (UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET);
    
    return (c);
}