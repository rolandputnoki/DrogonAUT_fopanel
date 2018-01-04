/*
 * gyari_radio_vevo_jel_meres.c
 *
 *  Created on: 2017. okt. 29.
 *      Author: Roland
 */
#include <motor_pwm.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_cortex.h"

//PA11-re kötjük a RC vevő motor csatornáját
void set_gy_rv_af_motor(){
	GPIO_InitTypeDef  GPIO_InitStructure;
	__GPIOA_CLK_ENABLE();
	// GPIO alternate function lábak
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
	GPIO_InitStructure.Pin =  GPIO_PIN_11;
	GPIO_InitStructure.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
}

//PB15-re kötjük a RC vevő szervo csatornáját
void set_gy_rv_af_szervo(){
	GPIO_InitTypeDef  GPIO_InitStructure;
	__GPIOB_CLK_ENABLE();
	// GPIO alternate function lábak
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
	GPIO_InitStructure.Pin =  GPIO_PIN_15;
	GPIO_InitStructure.Alternate = GPIO_AF9_TIM12;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}



TIM_IC_InitTypeDef TIM_ICInitStructure;
TIM_HandleTypeDef Tim12Handle;

void Init_input_capture_motor()
{
	/*
	__TIM1_CLK_ENABLE();
	//TIM1 APB2-n, így 168MHz
	Tim1Handle.Instance = TIM1;
	// Timer1 konfigurációja PWM üzemmódban
	Tim1Handle.Instance = TIM1;
	Tim1Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	Tim1Handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
	//	Tim1Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	Tim1Handle.Init.Prescaler = 0;
	Tim1Handle.Init.Period = SAJAT_MOTOR_COUNTER_MAX;
	Tim1Handle.State = HAL_TIM_STATE_RESET;

	HAL_TIM_Base_Init(&Tim1Handle);
*/

	//TIM1 már konfigurálva van a motorvezérléshez
	HAL_TIM_IC_Init(&Tim1Handle);

	TIM_ICInitStructure.ICFilter = 15;
	TIM_ICInitStructure.ICSelection	= TIM_ICSELECTION_DIRECTTI;
	TIM_ICInitStructure.ICPolarity = TIM_ICPOLARITY_RISING;
	TIM_ICInitStructure.ICPrescaler = TIM_ETRPRESCALER_DIV1;
	HAL_TIM_IC_ConfigChannel(&Tim1Handle, &TIM_ICInitStructure, TIM_CHANNEL_4);

	HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
	Tim1Handle.Instance->CCR4 = 0;
	HAL_TIM_IC_Start_IT(&Tim1Handle, TIM_CHANNEL_4);
}


void Init_input_capture_szervo()
{
	__TIM12_CLK_ENABLE();

	//TIM12 84 MHz
	Tim12Handle.Instance = TIM12;
	Tim12Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	Tim12Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	Tim12Handle.Init.Prescaler = 21;
	Tim12Handle.Init.Period = 65000;
	Tim12Handle.State = HAL_TIM_STATE_RESET;


	HAL_TIM_Base_Init(&Tim12Handle);
	HAL_TIM_IC_Init(&Tim12Handle);

	TIM_ICInitStructure.ICFilter = 15;
	TIM_ICInitStructure.ICSelection	= TIM_ICSELECTION_DIRECTTI;
	TIM_ICInitStructure.ICPolarity = TIM_ICPOLARITY_RISING;
	TIM_ICInitStructure.ICPrescaler = TIM_ETRPRESCALER_DIV1;
	HAL_TIM_IC_ConfigChannel(&Tim12Handle, &TIM_ICInitStructure, TIM_CHANNEL_2);

	HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
	Tim12Handle.Instance->CCR2 = 0;
	HAL_TIM_IC_Start_IT(&Tim12Handle, TIM_CHANNEL_2);
}

uint8_t rising_edge = 1;
uint16_t cmp1, cmp2, cmp;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == TIM1)
	{

		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4 )
		{
			cmp = htim->Instance->CCR4;

			// Gyári szervo pwm-jére kötjük
//			Tim4Handle.Instance->CCR1 = cmp;

			// Gyári motor pwm-jére kötjük
//			set_gyari_motor_compare_value(cmp);
			if(rising_edge){
				rising_edge = 0;
				TIM_ICInitStructure.ICPolarity = TIM_ICPOLARITY_FALLING;
				__HAL_TIM_SetCounter(&Tim1Handle, 0);
				HAL_TIM_IC_ConfigChannel(&Tim1Handle, &TIM_ICInitStructure, TIM_CHANNEL_4);
				HAL_TIM_IC_Start_IT(&Tim1Handle, TIM_CHANNEL_4);

			} else {
				rising_edge = 1;
				TIM_ICInitStructure.ICPolarity = TIM_ICPOLARITY_RISING;
				HAL_TIM_IC_ConfigChannel(&Tim1Handle, &TIM_ICInitStructure, TIM_CHANNEL_4);
				HAL_TIM_IC_Start_IT(&Tim1Handle, TIM_CHANNEL_4);
			}
		}
	} else if(htim->Instance == TIM12)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2 )
		{
			cmp = htim->Instance->CCR2;
			Tim4Handle.Instance->CCR1 = cmp;
			if(rising_edge){
				rising_edge = 0;
				TIM_ICInitStructure.ICPolarity = TIM_ICPOLARITY_FALLING;
				__HAL_TIM_SetCounter(&Tim12Handle, 0);
				HAL_TIM_IC_ConfigChannel(&Tim12Handle, &TIM_ICInitStructure, TIM_CHANNEL_2);
				HAL_TIM_IC_Start_IT(&Tim12Handle, TIM_CHANNEL_2);

			} else {
				rising_edge = 1;
				TIM_ICInitStructure.ICPolarity = TIM_ICPOLARITY_RISING;
				HAL_TIM_IC_ConfigChannel(&Tim12Handle, &TIM_ICInitStructure, TIM_CHANNEL_2);
				HAL_TIM_IC_Start_IT(&Tim12Handle, TIM_CHANNEL_2);
			}
		}
	}
}


void TIM1_CC_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&Tim1Handle);
}

void TIM8_BRK_TIM12_IRQHandler(void){
	HAL_TIM_IRQHandler(&Tim12Handle);
}



