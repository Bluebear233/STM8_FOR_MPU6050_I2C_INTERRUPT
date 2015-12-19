/*
 * TIM4.c
 *
 *  Created on: 2015年12月19日
 *      Author: Peng
 */
#include "stm8s.h"
#include "TIM4.h"
void TIM4_Config(void) {
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER4, ENABLE);

	TIM4_DeInit();

	TIM4_TimeBaseInit(TIM4_PRESCALER_128, 125);

	TIM4_ClearFlag (TIM4_FLAG_UPDATE);

	TIM4_Cmd (ENABLE);

	while (TIM4_GetFlagStatus(TIM4_FLAG_UPDATE) != SET)
		;

	TIM4_Cmd (DISABLE);

	TIM4_ClearFlag(TIM4_FLAG_UPDATE);

	TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);

}
