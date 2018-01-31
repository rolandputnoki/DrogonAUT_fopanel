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
void init_sebesseg_mero_timer();

//#define UNUSED(a) (void)a;

void init_LED2();

extern TIM_HandleTypeDef Tim7Handle;

uint8_t vonalak_szama();
uint8_t csikok_szama(uint32_t);

extern uint8_t new_cycle;

uint16_t capture_ertek;

extern float speed_of_drogon;

#define ONE_INC_IN_METER 0.00003f
#define L_SENSORS 182.0f
#define SENSORS_DIFF 0.8135f
#define ElSO_KORR_MM 2.9845f
#define HATSO_KORR_MM 3.721f

#define RADIAN_TO_DEGREE_CONV 57.2958f

typedef enum {
	START = 2,
	KORFORGALOM = 4,
	HORDO = 5,
	UTCA_SAROK = 6,
	GYORSIT = 7,
	LASSIT = 8,
	DRONE_KOVETKEZIK = 9,
	DRONE_ELOTT_ALLUNK = 10,
	DRONE_FELSZALLT = 11

} Robot_state;

/**************************/
/* Giroszkóp alapértékek */
#define X_ALAP 0
#define Y_ALAP 0
#define Z_ALAP 982
/**************************/


#endif /* BSP_H_ */
