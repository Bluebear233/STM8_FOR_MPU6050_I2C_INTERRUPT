/**
 ******************************************************************************
 * @file    GPIO_Toggle\main.c
 * @author  MCD Application Team
 * @version  V2.0.1
 * @date     18-November-2011
 * @brief   This file contains the main function for GPIO Toggle example.
 ******************************************************************************
 * @attention
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "USART.h"
#include "delay.h"
#include "i2c_master_interrupt.h"
#include "stm8s.h"
#include "inv_mpu.h"
#include "test.h"
#include <stdio.h>

/**
 * @addtogroup GPIO_Toggle
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Evalboard I/Os configuration */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
void main(void) {
	/* System Init */
	CLK_SYSCLKConfig (CLK_PRESCALER_HSIDIV2);

	delay_init();

	/* USART configured as follow:
	 - BaudRate = 256000 baud  
	 - Word Length = 8 Bits
	 - One Stop Bit
	 - No parity
	 - transmit enabled
	 */
	USART_Config();

	while (mpu_dmp_init() != 0)
		;

	float pitch, roll, yaw; 		//Å·À­½Ç
	while (TRUE) {
		if (mpu_dmp_get_data(&pitch, &roll, &yaw) == 0) {
			printf("Pitch: %7.2f Roll: %7.2f  Yaw: %7.2f \n\r\n", pitch, roll,
					yaw);
			//printf("ACCEL:%7d  %7d  %7d  ",accel[0],accel[1],accel[2]);
			//printf("GYRO:%7d  %7d  %7d \n\r\n",gyro[0],gyro[1],gyro[2]);
		}
	}

}

#ifdef USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
		printf("Wrong parameters value: file %s on line %lu\r\n", file, line);
	}
}
#endif

/**
 * @}
 */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
