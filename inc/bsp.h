/*
 * bsp.h
 *
 *  Created on: 2017. okt. 30.
 *      Author: Roland
 */

#ifndef BSP_H_
#define BSP_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_cortex.h"

void Sys_DelayUs(int us);
void Sys_DelayMs(int ms);
void GlobalFunctions_Init(void);
void SystemClock_Config(void);
void init_all();

//#define UNUSED(a) (void)a;

void init_LED2();

extern TIM_HandleTypeDef Tim7Handle;

uint8_t vonalak_szama();

extern uint8_t new_cycle;

#endif /* BSP_H_ */
