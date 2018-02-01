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


void SystemClock_Config();
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
uint8_t vil_ledek_szama();
uint8_t csikok_szama(uint32_t);
void korforgalom_jelzes_felismeres();
void fal_felismeres();
void sebesseg_szabalyzas();
void jelzes_felismeres(uint8_t);
void kereszt_vonal_felismeres(uint8_t);
void konvoj_elhaladas_felismeres();

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
	KORFORGALOM_KOVETKEZIK = 4,
	HORDO_KOVETKEZIK = 5,
	UTCA_SAROK_DUPLA_FAL = 6,
	UTCA_SAROK_MASODIK_FAL_JOBB = 13,
	UTCA_SAROK_MASODIK_FAL_BAL = 14,
	UTCA_SAROK_DUPLA_FAL_UTANI_SZUNET = 15,
	UTCA_SAROK_MASODIK_FAL_UTANI_SZUNET = 16,
	UTCA_SAROK_JOBB_FALAS_TOLATAS = 31,
	UTCA_SAROK_BAL_FALAS_TOLATAS = 32,
	GYORSIT = 7,
	LASSIT = 8,
	DRONE_KOVETKEZIK = 9,
	DRONE_ELOTT_ALLUNK = 10,
	DRONE_FELSZALLT = 11,
	JUST_GOING = 12,
	KONVOJ_KOVETKEZIK_JELZES_A_BAL_OLDALON = 17,
	KONVOJ_KOVETKEZIK_JELZES_A_JOBB_OLDALON = 18,
	KORFORG_JOBBRA_1 = 19,
	KORFORG_JOBBRA_2 = 20,
	KORFORG_JOBBRA_3 = 21,
	KORFORG_BALRA_1 = 22,
	KORFORG_BALRA_2 = 23,
	KORFORG_BALRA_3 = 24,
	GYOZELEM = 25,
	VASUTI_ATJARO_KOVETKEZIK = 26,
	VASUTI_ATJARO_KONVOJ_ELHALADASRA_VAR_ELOSZOR = 27,
	VASUTI_ATJARO_EGYSZER_ATHALADTUNK = 28,
	VASUTI_ATJARO_KONVOJ_ELHALADASRA_VAR_MASODJARA = 29,
	VASUTI_ATJARO_KETSZER_ATHALADTUNK = 30

} Robot_state;


typedef enum {
	JOBBRA_ELSO = 1,
	JOBBRA_MASODIK = 2,
	JOBBRA_HARMADIK = 3,
	BALRA_ELSO = 4,
	BALRA_MASODIK = 5,
	BALRA_HARMADIK = 6
} KORFORGALOM_UZENET;

/**************************/
/* Giroszkóp alapértékek */
#define X_ALAP 0
#define Y_ALAP 0
#define Z_ALAP 982
/**************************/

/***********************************/
/* Drón 2másodperces várakozásához */
uint16_t milisec_szamlalo;
uint16_t start_milisec_szamlalo;
/***********************************/


/***********************************/
/* Körforgalom jelvétel */
KORFORGALOM_UZENET korforgalom_uzenet;
uint8_t korforgalom_cim_stimmel;
/***********************************/


/**************************************************************/
/* Konvoj elhaladás felismerés segédváltozók  */
/* Konvoj elhaladásának méréséhez */
uint16_t egy_auto_elhaladasanak_ideje;
uint16_t egy_res_ideje;
uint8_t most_rest_merunk;
uint8_t most_autot_merunk;
uint8_t autot_erzekeltem;
uint16_t varakozasi_ido;
/**************************************************************/


#endif /* BSP_H_ */
