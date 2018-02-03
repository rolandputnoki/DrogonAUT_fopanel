#include "bsp.h"
#include "szervo_pwm.h"
#include "motor_pwm.h"
#include "uart_communication.h"
#include "bsp.h"
#include "adc.h"
#include "dma.h"
#include "encoder.h"

#include "sharp_hosszu.h"
#include "sharp_rovid.h"

//SPI1
SPI_HandleTypeDef spi;

uint32_t adc_eredmeny = 0;

//SPI fogad�si flag
uint8_t data_received = 1;

uint16_t KP_kormany = 0;
uint16_t KD_kormany = 0;


//Szab�lyoz�s

uint8_t kormany_szabalyzas_on = 1;
uint8_t sebesseg_szabalyzas_elore_on = 1;

float prev_pos = 0.0f;
float p_elso = 0.0f;
float p_hatso = 0.0f;

float p_elso_in_mm = 0.0f;
float p_hatso_in_mm = 0.0f;


float p_atmenet = 0.0f;
float D = 0.0f;
int32_t p_prev_konv = 0;
float error = 0.0f;
float prev_error = 0.0f;
float setValue = 0.0f;
int16_t pd_value = 0;


//PWM compare �rt�kek �ll�t�sa



uint16_t KP_speed = 80;
uint16_t KI_speed = 5;

float integrator_ertek = 0;

/**********************************/
/*     Sharp szenzorok adata      */
uint16_t elulso_sharp_szenzor;
uint16_t jobb_oldali_sharp_szenzor;
uint16_t bal_oldali_sharp_szenzor;

uint32_t adc_eredmeny_jobb = 0;
uint32_t adc_eredmeny_bal = 0;
uint32_t adc_eredmeny_elso = 0;
/**********************************/


/**********************************/


/**********************************/

//H�ny vonalat �rz�kel�nk
uint8_t vonalak = 0;

uint16_t mag_dec = 0;
uint32_t encoder_value = 0;
uint8_t vonalak_szama_a_mereskor = 0;

Robot_state state_of_robot = START;

float wanted_speed = 0.9f;
//float wanted_speed = 1.1f;


uint8_t sebesseg_tarto_counter = 0;

float speed_diff = 0;

int main()
{

	KP_kormany = KP_slow;
	KD_kormany = KD_slow;

	// HAL_Init, System_Clock_config �s hardware inicializ�ci�
    init_all();                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             	init_all();
    HAL_Delay(100);



	char buffer[10];
	HAL_ADC_Start_DMA(&hadc1, &adc_eredmeny_bal, (uint32_t)1);
	HAL_ADC_Start_DMA(&hadc2, &adc_eredmeny_elso, (uint32_t)1);
	HAL_ADC_Start_DMA(&hadc3, &adc_eredmeny_jobb, (uint32_t)1);

	while(1)
	{

/*
 *
		HAL_Delay(100);
		itoa(adc_eredmeny, buffer, 10);
		BT_UART_SendString(buffer);
		BT_UART_SendString("\r\n");


*/



		if(new_cycle)
		{
			elulso_sharp_szenzor = sharp_tomb_hosszu[adc_eredmeny_elso];
			jobb_oldali_sharp_szenzor = sharp_tomb_rovid[adc_eredmeny_jobb];
			bal_oldali_sharp_szenzor = sharp_tomb_rovid[adc_eredmeny_bal];
			ciklus();

/* TODO
			BT_UART_SendString("J:  ");
			itoa((int)(adc_eredmeny_jobb), buffer, 10);
			BT_UART_SendString(buffer);
			BT_UART_SendString("   ");


			itoa((int)(jobb_oldali_sharp_szenzor), buffer, 10);
			BT_UART_SendString(buffer);


			BT_UART_SendString("\r\n");

			BT_UART_SendString("B:");
			itoa((int)(adc_eredmeny_bal), buffer, 10);
			BT_UART_SendString(buffer);
			BT_UART_SendString("   ");


			itoa((int)(bal_oldali_sharp_szenzor), buffer, 10);
			BT_UART_SendString(buffer);

			BT_UART_SendString("\r\n");
*/
		}

	}

}





char parameter_buffer[50];
int32_t p_konv;
int32_t D_konv;

uint32_t cycle_counter = 0;
uint8_t dir = 0;

uint8_t elozo_vonalak[3];
uint8_t vonal_szamlalo = 0;
uint8_t csucs_kereses = 0;
uint8_t dummy_flag = 0;





//Az els� SPI adatfogad�s hib�s
//TODO: Gyere r� mi�rt
uint8_t first_cycle = 1;



/*****************************************/
//Gyorsas�gi p�lya jelz�s felismer�s
/*****************************************/
#define ONE_CYCLE_MEASERES 100
uint8_t vonalak_szama_sorrendben[ONE_CYCLE_MEASERES];
uint16_t felismeres_cycle_counter = 0;
uint8_t adott_meres_vonal_szama = 0;
uint8_t meresi_cikluson_beluli_3as_vonal_szam = 0;
/*****************************************/



/*****************************************/
//V�ltoz�k az spi adatfogad�shoz
//�s poz�ci� sz�mol�shoz mindk�t szenzorsor eset�n
/*****************************************/
uint16_t adcAdatok_hatso[32];
uint16_t adcAdatok_elso[32];

uint16_t adcAdatok_buffer_hatso[32];
uint16_t adcAdatok_buffer_elso[32];

int32_t adcAdatok_sulyozott_elso[32];
int32_t adcAdatok_sulyozott_hatso[32];

uint32_t szumma_adc_values_elso = 0;
int32_t szumma_sulyozott_elso = 0;

uint32_t szumma_adc_values_hatso = 0;
int32_t szumma_sulyozott_hatso = 0;
/*****************************************/


uint32_t meres_kezdeti_encoder_ertek = 0;
uint32_t meres_mostani_encoder_ertek = 0;
uint32_t meres_megtett_tavolsag = 0;
char buf10[10];

uint8_t vil_led = 0;

/*************************************************/
/*                 UTCASAROK seg�dv�ltoz�k  */

//JOBB
int32_t utca_sarok_jobb_fal_kezdet_encoder = 0;
int32_t utca_sarok_jobb_fal_mostani_encoder = 0;
uint8_t utca_sarok_jobb_fal_kezdet = 1;
int32_t utca_sarok_jobb_fal_hossz = 0;

int32_t jobb_tolatas_hossz = 0;
int32_t jobb_tolatas_kezdet_encoder_ertek = 0;
int32_t jobb_tolatas_mostani_encoder_ertek = 0;

uint8_t eloszor_van_jobb_tolatas = 1;


//BAL
int32_t utca_sarok_bal_fal_kezdet_encoder = 0;
int32_t utca_sarok_bal_fal_mostani_encoder = 0;
uint8_t utca_sarok_bal_fal_kezdet = 1;
int32_t utca_sarok_bal_fal_hossz = 0;

int32_t bal_tolatas_hossz = 0;
int32_t bal_tolatas_kezdet_encoder_ertek = 0;
int32_t bal_tolatas_mostani_encoder_ertek = 0;

uint8_t eloszor_van_bal_tolatas = 1;


//MINDKETT�
uint8_t utca_sarok_megalltunk = 0;
uint8_t arasz_nulla_led_vilagitott_elobb = 0;
uint8_t arasz_hanyszor_volt_nulla = 0;
uint8_t tolatas_veget_ert = 1;
/*************************************************/



/*************************************************/
/*                 VAS�TI �TJ�R� seg�dv�ltoz�k  */
int32_t v_a_jelzes_kezdet_encoder = 0;
int32_t v_a_jelzes_mostani_encoder = 0;
uint8_t v_a_jelezs_kezdet = 1;
int32_t v_a_jelzes_utan_megtett_ut = 0;
uint8_t meg_volt_a_kereszt_vonal = 0;

uint8_t koztes_szakasz_kezdet = 1;
int32_t koztes_szak_kezdet_encoder = 0;
int32_t koztes_szak_mostani_encoder = 0;
int32_t koztes_szak_hossz = 0;

uint8_t megalltunk = 0;

uint8_t v_a_elso_dupla_kereszt_mar_meg_volt = 0;
uint8_t v_a_masodiik_dupla_kereszt_is_meg_volt = 0;


uint8_t nulla_led_vilagitott_elobb = 0;
uint8_t elobb_nulla_led_volt = 0;
uint8_t hanyszor_volt_nulla = 0;
uint8_t most_nulla = 0;
/*************************************************/


/*************************************************/
/*   K�RFORGALOM seg�dv�ltoz�k  */
int32_t elozo_encoder_ertek = 0;
int32_t mostani_encoder_ertek = 0;
uint8_t mar_lementem_a_vonalrol = 0;
uint8_t korforg_hanyszor_volt_nulla = 0;
uint8_t korforg_elobb_nulla_led_volt = 0;
uint8_t korforg_null_led_van = 0;
uint8_t korforg_megalltunk = 0;
uint8_t infra_inicializalva = 0;

int32_t korforgalom_jelzes_utani_elso_encoder_ertek = 0;
int32_t korforg_jel_utani_mostani_encoder_ertek = 0;
int32_t korforg_jel_utani_hossz = 0;

/*          JOBBRA 1                   */
int32_t j1_kezdet_encoder_ertek = 0;
int32_t j1_mostani_encoder_ertek = 0;
uint8_t j1_most_kezd_merni = 1;
int32_t j1_hossz = 0;

/*          JOBBRA 2                   */




/*          JOBBRA 3                   */

/*          BALRA 1                   */
int32_t b1_kezdet_encoder_ertek = 0;
int32_t b1_mostani_encoder_ertek = 0;
uint8_t b1_most_kezd_merni = 1;
int32_t b1_hossz = 0;




/*   Fix �vek amiken fordulunk  */
int32_t kor_ford_kezdet_encoder_ertek = 0;
int32_t kor_ford_mostani_encoder_ertek = 0;
int32_t kor_ford_hossz = 0;
uint8_t kor_ford_most_kezd_merni = 0;
uint8_t kor_az_elso_iv_megvolt = 0;
/*************************************************/


void ciklus(){

	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
	szumma_adc_values_elso = 0;
	szumma_sulyozott_elso = 0;

	szumma_adc_values_hatso = 0;
	szumma_sulyozott_hatso = 0;


	// Adatfogad�s kezdete
	data_received = 0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_SPI_Receive_IT(&spi, adcAdatok_buffer_hatso, 64);
	while(!data_received);


	data_received = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_SPI_Receive_IT(&spi, adcAdatok_buffer_elso, 64);
	while(!data_received);

	// Adatfogad�s v�ge

	if(first_cycle){
		first_cycle = 0;

		meres_kezdeti_encoder_ertek = get_encoder_counter();
	} else {

		// Csak a vil�g�t� ledekhez tartoz� AD adatokat tartjuk meg
		for(uint8_t n = 0; n<32; n++){

			if(adcAdatok_buffer_hatso[n] <= 2000){

				adcAdatok_hatso[n] = 0;
			} else {
				adcAdatok_hatso[n] = adcAdatok_buffer_hatso[n];
			}

			if(adcAdatok_buffer_elso[n] <= 2000){

				adcAdatok_elso[n] = 0;
			} else {
				adcAdatok_elso[n] = adcAdatok_buffer_elso[n];
			}
		}

/****************************************************************/
		/* Itt kell m�g m�dos�tani az adcAdatokon, ha tolatni akarunk  */
/****************************************************************/


		//ar�nyos t�nyez� sz�m�t�sa
		for(int i = 0; i < 32; i++)
		{
			szumma_adc_values_elso += adcAdatok_elso[i];
			adcAdatok_sulyozott_elso[i] = adcAdatok_elso[i] * sorszam[i];			//s�lyoz�s
			szumma_sulyozott_elso += adcAdatok_sulyozott_elso[i];

			szumma_adc_values_hatso += adcAdatok_hatso[i];
			adcAdatok_sulyozott_hatso[i] = adcAdatok_hatso[i] * sorszam[i];			//s�lyoz�s
			szumma_sulyozott_hatso += adcAdatok_sulyozott_hatso[i];

		}

		//Els� szenzorsor poz�ci�
		if(szumma_adc_values_elso != 0){
			p_elso = (float)szumma_sulyozott_elso/szumma_adc_values_elso;
		} else {
			p_elso = 0;
		}

		//H�ts� szenzorsor poz�ci�
		if(szumma_adc_values_hatso != 0){
			p_hatso = (float)szumma_sulyozott_hatso/szumma_adc_values_hatso;
		} else {
			p_hatso = 0;
		}

/****************************************************************/

		/*** Id�ig tart a poz�ci� kisz�mol�sa ***/
/****************************************************************/
//PD szab�lyzo� egy vonalszenzor alapj�n

		//Differencia az el�z� poz�ci� hib�t�l
		D = p_elso-prev_pos;


		if(prev_pos > 14 || prev_pos < -14){
			p_atmenet = p_elso;
			if(p_atmenet < 0){
				p_atmenet *= -1;
			}
			if(p_atmenet < 3){
				p_elso = prev_pos;
			}
		}

		if(!vil_ledek_szama()){
			p_elso = prev_pos;
		}

		pd_value = KP_kormany*p_elso + KD_kormany*D;
		prev_pos = p_elso;


		szervo_value = DIGIT_SZ_KOZEP + (int16_t)pd_value;

		if(kormany_szabalyzas_on){

			if(vil_ledek_szama() == 0){
//				set_compare_value_digit_szervo(33000);
				set_compare_value_digit_szervo(33500);
			} else {
				set_compare_value_digit_szervo(szervo_value);
			}


		}


		meres_mostani_encoder_ertek = get_encoder_counter();
		meres_megtett_tavolsag = (meres_kezdeti_encoder_ertek - meres_mostani_encoder_ertek)*ENCODER_VALUE_TO_MM;
/*
		char buff3 [10];

		vil_led = vil_ledek_szama();
		itoa(vil_led, buff3, 10);
		BT_UART_SendString(buff3);
		BT_UART_SendString("\r\n");
*/
/************************************************************************/
/************************************************************************/

		switch(state_of_robot){





		case START:
			if(meg_jott_a_start_kapu_jele){
				state_of_robot = JUST_GOING;
			}
			break;

/****************************************************/
			/* DR�N �LLAPOTAI */
/****************************************************/

		case DRONE_ELOTT_ALLUNK:
			BT_UART_SendString("DRONE all\r\n");

/*TODO: Kiszedni, mert ez csak teszt
			itoa(elulso_sharp_szenzor, buf10, 10);
			BT_UART_SendString(buf10);
			BT_UART_SendString("\r\n");

*/
			if(elulso_sharp_szenzor >= 1200){
				state_of_robot = DRONE_FELSZALLT;
			}
			break;

		case DRONE_KOVETKEZIK:
			BT_UART_SendString("DRONE k�v\r\n");
			wanted_speed = (elulso_sharp_szenzor - 550)*0.00057f;
			if(!speed_of_drogon || elulso_sharp_szenzor <= 550){
					wanted_speed = 0.0f;
					state_of_robot = DRONE_ELOTT_ALLUNK;
			}
			break;
		case DRONE_FELSZALLT:
			BT_UART_SendString("DRONE szal\r\n");

			start_milisec_szamlalo = 1;
			if(milisec_szamlalo > 2100){
				wanted_speed = 0.9f;
				state_of_robot = JUST_GOING;
			}
			break;

/****************************************************/
			/* UTCASAROK �LLAPOTAI */
/****************************************************/

		case UTCA_SAROK_DUPLA_FAL:
			BT_UART_SendString("DUPLA FAL");
			BT_UART_SendString("\r\n");
			wanted_speed = 0.8f;
			break;

		case UTCA_SAROK_DUPLA_FAL_UTANI_SZUNET:
			BT_UART_SendString("DUPLA FAL U SZ");
			BT_UART_SendString("\r\n");

			break;

		case UTCA_SAROK_MASODIK_FAL_JOBB:
			BT_UART_SendString("2. J");
			BT_UART_SendString("\r\n");
			if(utca_sarok_jobb_fal_kezdet){
				utca_sarok_jobb_fal_kezdet_encoder = get_encoder_counter();
				utca_sarok_jobb_fal_kezdet = 0;
			} else {
				utca_sarok_jobb_fal_mostani_encoder = get_encoder_counter();
				utca_sarok_jobb_fal_hossz = (utca_sarok_jobb_fal_kezdet_encoder - utca_sarok_jobb_fal_mostani_encoder)*ENCODER_VALUE_TO_MM;

				if(utca_sarok_jobb_fal_hossz >= 300){
					state_of_robot = UTCA_SAROK_JOBB_ARASZOLAS;
				}
			}

			break;


		case UTCA_SAROK_JOBB_ARASZOLAS:
//			BT_UART_SendString("J ARASZ\r\n");

			if(!utca_sarok_megalltunk){
				set_gyari_motor_compare_value(5800);
				if(speed_of_drogon < 0.01f)
				{
					utca_sarok_megalltunk = 1;
				}
			} else {
/*
				itoa((int)integrator_ertek, buf10, 10);
				BT_UART_SendString(buf10);
				BT_UART_SendString("\r\n");

*/
				set_gyari_motor_compare_value(6510);
//				wanted_speed = 0.5f;
				sebesseg_szabalyzas_elore_on = 0;
			}

			if(vil_ledek_szama() == 0){



				if(!arasz_nulla_led_vilagitott_elobb){
					arasz_nulla_led_vilagitott_elobb = 1;
				} else {
					arasz_hanyszor_volt_nulla++;
				}

				if(arasz_hanyszor_volt_nulla >= 5){
					state_of_robot = UTCA_SAROK_JOBB_FALAS_TOLATAS;
				}

			} else {
				arasz_nulla_led_vilagitott_elobb = 0;
				arasz_hanyszor_volt_nulla = 0;
			}

			break;

		case UTCA_SAROK_JOBB_FALAS_TOLATAS:

//TODO: bt t�rl�s
			BT_UART_SendString("J TOLAT\r\n");

			if(eloszor_van_jobb_tolatas) {
				set_gyari_motor_compare_value(5800);
				integrator_ertek = 200;
				eloszor_van_jobb_tolatas = 0;
				jobb_tolatas_kezdet_encoder_ertek = get_encoder_counter();
			} else {
				jobb_tolatas_mostani_encoder_ertek = get_encoder_counter();
				jobb_tolatas_hossz = (jobb_tolatas_mostani_encoder_ertek - jobb_tolatas_kezdet_encoder_ertek)*ENCODER_VALUE_TO_MM;
			}

			if(jobb_tolatas_hossz >= 1400){
				if(tolatas_veget_ert){
					tolatas_veget_ert = 0;
					varjuk_meg_a_kozep_erteket = 0;
					megvartuk_a_kozepet = 0;
					kozep_ido_milisec = 0;
					set_gyari_motor_compare_value(6200);
					varjuk_meg_a_kozep_erteket = 1;
				} else {
					if(kozep_ido_milisec >= 50)
					{
						megvartuk_a_kozepet = 1;
						state_of_robot = UTCA_SAROK_JOBB_TOLATAS_VEGE;
					}
				}
			} else {
				if(!varjuk_meg_a_kozep_erteket){
					if(!varjuk_meg_a_hatra_erteket){
						varjuk_meg_a_hatra_erteket = 1;
						set_gyari_motor_compare_value(5800);
					} else {

						if(hatra_ido_milisec >= 300){
							varjuk_meg_a_kozep_erteket = 1;
							set_gyari_motor_compare_value(6200);
							varjuk_meg_a_kozep_erteket = 1;
							megvartuk_a_hatrat = 1;
						}
					}
				} else {
					if(kozep_ido_milisec >= 300){
						megvartuk_a_kozepet = 1;
						set_compare_value_digit_szervo(36750);
						set_gyari_motor_compare_value(5600);
					}
				}
			}
			kormany_szabalyzas_on = 0;
			sebesseg_szabalyzas_elore_on = 0;
			break;


		case UTCA_SAROK_JOBB_TOLATAS_VEGE:
			BT_UART_SendString("J T VEGE\r\n");
			kormany_szabalyzas_on = 1;
			set_gyari_motor_compare_value(6510);

			break;

		case UTCA_SAROK_MASODIK_FAL_BAL:
			BT_UART_SendString("2. B");
			BT_UART_SendString("\r\n");
			if(utca_sarok_bal_fal_kezdet){
				utca_sarok_bal_fal_kezdet_encoder = get_encoder_counter();
				utca_sarok_bal_fal_kezdet = 0;
			} else {
				utca_sarok_bal_fal_mostani_encoder = get_encoder_counter();
				utca_sarok_bal_fal_hossz = (utca_sarok_bal_fal_kezdet_encoder - utca_sarok_bal_fal_mostani_encoder)*ENCODER_VALUE_TO_MM;

				if(utca_sarok_bal_fal_hossz >= 300){
					state_of_robot = UTCA_SAROK_BAL_ARASZOLAS;
				}
			}
			break;

		case UTCA_SAROK_BAL_ARASZOLAS:
			BT_UART_SendString("B ARASZ\r\n");

			if(!utca_sarok_megalltunk){
				set_gyari_motor_compare_value(5600);
				if(speed_of_drogon < 0.01f)
				{
					utca_sarok_megalltunk = 1;
				}
			} else {
				set_gyari_motor_compare_value(6510);
//				wanted_speed = 0.5f;
				sebesseg_szabalyzas_elore_on = 0;
			}

			if(vil_ledek_szama() == 0){
				if(!arasz_nulla_led_vilagitott_elobb){
					arasz_nulla_led_vilagitott_elobb = 1;
				} else {
					arasz_hanyszor_volt_nulla++;
				}
				if(arasz_hanyszor_volt_nulla >= 5){
					state_of_robot = UTCA_SAROK_BAL_FALAS_TOLATAS;
				}
			} else {
				arasz_nulla_led_vilagitott_elobb = 0;
				arasz_hanyszor_volt_nulla = 0;
			}

			break;

		case UTCA_SAROK_BAL_FALAS_TOLATAS:

//TODO BT
			BT_UART_SendString("B TOLAT\r\n");
			if(eloszor_van_bal_tolatas) {
				set_gyari_motor_compare_value(5600);
				eloszor_van_bal_tolatas = 0;
				bal_tolatas_kezdet_encoder_ertek = get_encoder_counter();
			} else {
				bal_tolatas_mostani_encoder_ertek = get_encoder_counter();
				bal_tolatas_hossz = (bal_tolatas_mostani_encoder_ertek - bal_tolatas_kezdet_encoder_ertek)*ENCODER_VALUE_TO_MM;
			}

			if(bal_tolatas_hossz >= 1350){
				if(tolatas_veget_ert){
					tolatas_veget_ert = 0;
					varjuk_meg_a_kozep_erteket = 0;
					megvartuk_a_kozepet = 0;
					kozep_ido_milisec = 0;
					set_gyari_motor_compare_value(6200);
					varjuk_meg_a_kozep_erteket = 1;
				} else {
					if(kozep_ido_milisec >= 50)
					{
						megvartuk_a_kozepet = 1;
						state_of_robot = UTCA_SAROK_BAL_TOLATAS_VEGE;
					}
				}
			} else {
				if(!varjuk_meg_a_kozep_erteket)
				{
					if(!varjuk_meg_a_hatra_erteket)
					{
						varjuk_meg_a_hatra_erteket = 1;
						set_gyari_motor_compare_value(5600);
					} else {
						if(hatra_ido_milisec >= 300)
						{
							BT_UART_SendString("H M V\r\n");
							varjuk_meg_a_kozep_erteket = 1;
							set_gyari_motor_compare_value(6200);
							varjuk_meg_a_kozep_erteket = 1;
							megvartuk_a_hatrat = 1;
						}
					}
				} else {
					if(kozep_ido_milisec >= 300)
					{
						megvartuk_a_kozepet = 1;
						set_compare_value_digit_szervo(28000);
						set_gyari_motor_compare_value(5600);
					}
				}
			}
			kormany_szabalyzas_on = 0;
			sebesseg_szabalyzas_elore_on = 0;

			break;


		case UTCA_SAROK_MASODIK_FAL_UTANI_SZUNET:
			BT_UART_SendString("2. SZ�N");
			BT_UART_SendString("\r\n");
			set_gyari_motor_compare_value(5800);
			break;



/****************************************************/
			/* KONVOJ �LLAPOTAI */
/****************************************************/

		case KONVOJ_KOVETKEZIK_JELZES_A_JOBB_OLDALON:
			BT_UART_SendString("KONV J\r\n");
			set_gyari_motor_compare_value(6200);
			while(1){

			}
			break;

		case KONVOJ_JELZES_JOBB_KOZELEDES:
			break;

		case KONVOJ_JELZES_JOBB_RAALLAS:

			break;

		case KONVOJ_KOVETKEZIK_JELZES_A_BAL_OLDALON:
			BT_UART_SendString("KONV B\r\n");
			set_gyari_motor_compare_value(6200);
			while(1){

			}

			break;

		case KONVOJ_JELZES_BAL_KOZELEDES:
			break;


		case KONVOJ_JELZES_BAL_RAALLAS:
			break;

/****************************************************/
			/* K�RFORGALOM �LLAPOTAI */
/****************************************************/
		case KORFORGALOM_KOVETKEZIK:
			BT_UART_SendString("K�RFO\r\n");

			korforg_jel_utani_mostani_encoder_ertek = get_encoder_counter();
			korforg_jel_utani_hossz = (korforgalom_jelzes_utani_elso_encoder_ertek - korforg_jel_utani_mostani_encoder_ertek)*ENCODER_VALUE_TO_MM;
			if(!infra_inicializalva){
				init_infra_timer();
				infra_inicializalva = 1;
			}

			if(!megalltunk){
				set_gyari_motor_compare_value(5600);
				mostani_encoder_ertek = get_encoder_counter();

				if(mostani_encoder_ertek == elozo_encoder_ertek){
					megalltunk = 1;
				}
				elozo_encoder_ertek = mostani_encoder_ertek;
			} else {
/*			 Els� sharp alapj�n k�zeltj�k meg a k�rforgalmat.
				if(elulso_sharp_szenzor >= 550){
					set_gyari_motor_compare_value(6510);

					itoa(elulso_sharp_szenzor, buf10, 10);
					BT_UART_SendString(buf10);
					BT_UART_SendString("MESSZE\r\n");
				} else {
					set_gyari_motor_compare_value(5600);
					mostani_encoder_ertek = get_encoder_counter();

					if(mostani_encoder_ertek == elozo_encoder_ertek){
						korforgalom_jelzes_felismeres();
					}
					elozo_encoder_ertek = mostani_encoder_ertek;
				}
				
*/

/* Megtett t�vols�g alapj�n k�zel�tj�k a k�rforgalmat*/
				if(korforg_jel_utani_hossz <= 440){
					set_gyari_motor_compare_value(6510);
/*
					itoa(elulso_sharp_szenzor, buf10, 10);
					BT_UART_SendString(buf10);
					BT_UART_SendString("MESSZE\r\n");

					*/
				} else {
					set_gyari_motor_compare_value(5600);
					mostani_encoder_ertek = get_encoder_counter();

					if(mostani_encoder_ertek == elozo_encoder_ertek){
						BT_UART_SendString("JEL F ISM\r\n");
						korforgalom_jelzes_felismeres();
					}
					elozo_encoder_ertek = mostani_encoder_ertek;
				}
			}

/****************************************************/
/*Egy m�d a meg�ll�sra */

			sebesseg_szabalyzas_elore_on = 0;
//			wanted_speed = 0.0f;
			break;

		case KORFORG_JOBBRA_1:
			set_gyari_motor_compare_value(6510);
			BT_UART_SendString("K J 1\r\n");




			if(j1_most_kezd_merni)
			{
				j1_most_kezd_merni = 0;
				set_compare_value_digit_szervo(26600);
				kormany_szabalyzas_on = 0;
				j1_kezdet_encoder_ertek = get_encoder_counter();

			} else {
				j1_mostani_encoder_ertek = get_encoder_counter();
				j1_hossz = (j1_kezdet_encoder_ertek - j1_mostani_encoder_ertek )*ENCODER_VALUE_TO_MM;
				if(j1_hossz >= 500){

					if(kormany_szabalyzas_on){

					} else {
						set_compare_value_digit_szervo(32500);
					}

				}
				if(j1_hossz >= 700){
					kormany_szabalyzas_on = 1;
					BT_UART_SendString("KORM ON\r\n");
				}
			}

/*
			if(vil_ledek_szama() == 0)
			{
				if(!korforg_elobb_nulla_led_volt)
				{
					korforg_elobb_nulla_led_volt = 1;
				} else
				{
					korforg_hanyszor_volt_nulla++;
					if(korforg_hanyszor_volt_nulla >= 3)
					{
						state_of_robot = KORFORG_JOBBRA_1_LENT_A_VONALROL;
					}
				}
			} else {
				korforg_elobb_nulla_led_volt = 0;
				korforg_hanyszor_volt_nulla = 0;
			}
*/


			break;


		case KORFORG_JOBBRA_1_LENT_A_VONALROL:
		
			BT_UART_SendString("K J 1 L V\r\n");

			if(vil_ledek_szama() == 0){

			} else {
				kormany_szabalyzas_on = 1;
			}

			break;

		case KORFORG_JOBBRA_2:
			BT_UART_SendString("K J 2\r\n");

			if(kor_ford_most_kezd_merni)
			{
				kor_ford_most_kezd_merni = 0;
				set_compare_value_digit_szervo(26600);
				kormany_szabalyzas_on = 0;
				kor_ford_kezdet_encoder_ertek = get_encoder_counter();

			} else {

				kor_ford_mostani_encoder_ertek = get_encoder_counter();
				kor_ford_hossz = (kor_ford_kezdet_encoder_ertek - kor_ford_mostani_encoder_ertek )*ENCODER_VALUE_TO_MM;
				if(!kor_az_elso_iv_megvolt)
				{
					if(kor_ford_hossz >= 500){
						kor_az_elso_iv_megvolt = 1;
					}
				} else {
					if(kor_ford_hossz >= 1000){
						kormany_szabalyzas_on = 1;
					} else {
						set_compare_value_digit_szervo(36600);
					}
				}

			}

			break;

		case KORFORG_JOBBRA_3:
			BT_UART_SendString("K J 3\r\n");

			if(kor_ford_most_kezd_merni)
			{
				kor_ford_most_kezd_merni = 0;
				set_compare_value_digit_szervo(26600);
				kormany_szabalyzas_on = 0;
				kor_ford_kezdet_encoder_ertek = get_encoder_counter();

			} else {

				kor_ford_mostani_encoder_ertek = get_encoder_counter();
				kor_ford_hossz = (kor_ford_kezdet_encoder_ertek - kor_ford_mostani_encoder_ertek )*ENCODER_VALUE_TO_MM;
				if(!kor_az_elso_iv_megvolt)
				{
					if(kor_ford_hossz >= 500){
						kor_az_elso_iv_megvolt = 1;
					}
				} else {
					if(kor_ford_hossz >= 1500){
						kormany_szabalyzas_on = 1;
					} else {
						set_compare_value_digit_szervo(36600);
					}
				}

			}
			break;

		case KORFORG_BALRA_1:

			BT_UART_SendString("K B 1\r\n");

			set_gyari_motor_compare_value(6510);





			if(b1_most_kezd_merni)
			{
				b1_most_kezd_merni = 0;
				set_compare_value_digit_szervo(36600);
				kormany_szabalyzas_on = 0;
				b1_kezdet_encoder_ertek = get_encoder_counter();

			} else {
				b1_mostani_encoder_ertek = get_encoder_counter();
				b1_hossz = (b1_kezdet_encoder_ertek - b1_mostani_encoder_ertek )*ENCODER_VALUE_TO_MM;
				if(b1_hossz >= 500){

					if(kormany_szabalyzas_on){

					} else {
						set_compare_value_digit_szervo(32500);
					}

				}
				if(b1_hossz >= 700){
					kormany_szabalyzas_on = 1;
					BT_UART_SendString("KORM ON\r\n");
				}
			}
			break;

		case KORFORG_BALRA_2:
			BT_UART_SendString("K B 2\r\n");


			if(kor_ford_most_kezd_merni)
			{
				kor_ford_most_kezd_merni = 0;
				set_compare_value_digit_szervo(36600);
				kormany_szabalyzas_on = 0;
				kor_ford_kezdet_encoder_ertek = get_encoder_counter();

			} else {

				kor_ford_mostani_encoder_ertek = get_encoder_counter();
				kor_ford_hossz = (kor_ford_kezdet_encoder_ertek - kor_ford_mostani_encoder_ertek )*ENCODER_VALUE_TO_MM;
				if(!kor_az_elso_iv_megvolt)
				{
					if(kor_ford_hossz >= 500){
						kor_az_elso_iv_megvolt = 1;
					}
				} else {
					if(kor_ford_hossz >= 1000){
						kormany_szabalyzas_on = 1;
					} else {
						set_compare_value_digit_szervo(26600);
					}
				}

			}

			break;

		case KORFORG_BALRA_3:

			BT_UART_SendString("K B 3\r\n");


			if(kor_ford_most_kezd_merni)
			{
				kor_ford_most_kezd_merni = 0;
				set_compare_value_digit_szervo(36600);
				kormany_szabalyzas_on = 0;
				kor_ford_kezdet_encoder_ertek = get_encoder_counter();

			} else {

				kor_ford_mostani_encoder_ertek = get_encoder_counter();
				kor_ford_hossz = (kor_ford_kezdet_encoder_ertek - kor_ford_mostani_encoder_ertek )*ENCODER_VALUE_TO_MM;
				if(!kor_az_elso_iv_megvolt)
				{
					if(kor_ford_hossz >= 500){
						kor_az_elso_iv_megvolt = 1;
					}
				} else {
					if(kor_ford_hossz >= 1500){
						kormany_szabalyzas_on = 1;
					} else {
						set_compare_value_digit_szervo(26600);
					}
				}

			}
			break;


/****************************************************/
			/* VAS�TI �TJ�R� �LLAPOTAI */
/****************************************************/

		case VASUTI_ATJARO_KOVETKEZIK:
			BT_UART_SendString("VAS_�T jel\r\n");

			if(v_a_jelezs_kezdet)
			{
				v_a_jelzes_kezdet_encoder = get_encoder_counter();
				v_a_jelezs_kezdet = 0;
				set_gyari_motor_compare_value(5800);
			} else {

/*********************************************************************/
				/* Egyik gondolatom, hogy a jelz�s ut�n a megtett t�vols�ggal ar�nyosan cs�kken a sebess�g
				 * De egyenl�re kipr�b�ljuk, hogy csak lecs�kkentem, �s a keresztvonal ut�n �llok meg
				 *
				 *
				 *
				 * */

/*********************************************************************/
				v_a_jelzes_mostani_encoder = get_encoder_counter();
				v_a_jelzes_utan_megtett_ut = (v_a_jelzes_kezdet_encoder - v_a_jelzes_mostani_encoder)*ENCODER_VALUE_TO_MM;
			}
			if(!megalltunk){
				set_gyari_motor_compare_value(5800);
				if(speed_of_drogon < 0.01f)
				{
					megalltunk = 1;
				}
			} else {
				set_gyari_motor_compare_value(6520);
				wanted_speed = 0.5f;
			}

			if(vil_ledek_szama() == 0){



				if(!nulla_led_vilagitott_elobb){
					nulla_led_vilagitott_elobb = 1;
				} else {
					hanyszor_volt_nulla++;
				}

				if(hanyszor_volt_nulla >= 10){
					state_of_robot = VASUTI_ATJARO_KONVOJ_ELHALADASRA_VAR_ELOSZOR;
				}

			} else {
				nulla_led_vilagitott_elobb = 0;
				hanyszor_volt_nulla = 0;
			}
			break;

		case VASUTI_ATJARO_KONVOJ_ELHALADASRA_VAR_ELOSZOR:
			BT_UART_SendString("1. V�R\r\n");
			set_gyari_motor_compare_value(5800);
			itoa(elulso_sharp_szenzor, buf10, 10);
			BT_UART_SendString(buf10);
			BT_UART_SendString("\r\n");
			konvoj_elhaladas_felismeres();
			break;

		case VASUTI_ATJARO_EGYSZER_ATHALADTUNK:
			BT_UART_SendString("1x �T\r\n");
			wanted_speed = 0.8f;
			if(koztes_szakasz_kezdet)
			{
				koztes_szakasz_kezdet = 0;
				koztes_szak_kezdet_encoder = get_encoder_counter();
			} else
			{
				koztes_szak_mostani_encoder = get_encoder_counter();
				koztes_szak_hossz = (koztes_szak_kezdet_encoder - koztes_szak_mostani_encoder)*ENCODER_VALUE_TO_MM;

				itoa(koztes_szak_hossz, buf10, 10);
				BT_UART_SendString(buf10);
				BT_UART_SendString("\r\n");
				if(koztes_szak_hossz >= 1250){
					set_gyari_motor_compare_value(5800);
					if(speed_of_drogon == 0.0f)
					{
						state_of_robot = VASUTI_ATJARO_KONVOJ_ELHALADASRA_VAR_MASODJARA;
					}
				}
			}
			break;

		case VASUTI_ATJARO_KONVOJ_ELHALADASRA_VAR_MASODJARA:
			BT_UART_SendString("2. V�R\r\n");
			set_gyari_motor_compare_value(5800);


			konvoj_elhaladas_felismeres();
			break;

		case VASUTI_ATJARO_KETSZER_ATHALADTUNK:
			BT_UART_SendString("2x �T\r\n");
			wanted_speed = 0.5f;
			/*
			if(v_a_masodiik_dupla_kereszt_is_meg_volt){
				set_gyari_motor_compare_value(5800);
				while(1);
			}
			*/
			break;


/****************************************************/
			/* HORD� �LLAPOTAI */
/****************************************************/


		case HORDO_KOVETKEZIK:
			BT_UART_SendString("HORD� jel\r\n");
			wanted_speed = 1.4f;
			while(1){

			}
			break;


/****************************************************/
			/* C�L �LLAPOTAI */
/****************************************************/


		case GYOZELEM:
			set_gyari_motor_compare_value(5800);
			BT_UART_SendString("GYOZELEM");
			while(1);

			break;


		case JUST_GOING:
			BT_UART_SendString("CSILL\r\n");
			break;

		default:

			break;
		}




/************************************************************************/

		if(!meg_jott_a_start_kapu_jele){

		} else {

			if(sebesseg_szabalyzas_elore_on)
			{
				sebesseg_szabalyzas();
			}

		}
			fal_felismeres();
			kereszt_vonal_felismeres(vil_ledek_szama());
			jelzes_felismeres(vonalak_szama());



/*******************************************/
//Ciklus �jra ind�t�sa csak a timer lej�rta ut�n t�rt�nhet meg. Ez�rt kell null�zni,
//hogy ha a ciklus v�get is �rt, a main-ben ne h�v�djon meg �jra, m�g a timer owerflowja
//be nem billenti a flaget
		new_cycle = 0;
/*******************************************/
	}

}



uint8_t vonalak_szama(){
	uint8_t csucs_kezdete = 0;
	uint8_t cnt = 0;
	uint8_t m;
	for(m = 0; m < 32; m++){
		if(!csucs_kezdete){
			if(adcAdatok_elso[m] != 0){
				csucs_kezdete = 1;

			}
		} else {
			if(adcAdatok_elso[m] == 0){
				csucs_kezdete = 0;
				cnt++;
			}
		}
	}
	return cnt;
}

uint8_t vil_ledek_szama()
{
	uint8_t cnt = 0;
	uint8_t m = 0;
	for(m = 0; m < 32; m++)
	{
		if(adcAdatok_elso[m] != 0){
			cnt++;
		}
	}
	return cnt;
}

/***************************/
/* Fal felismeri seg�dv�ltoz�k */
uint8_t meg_nem_volt_utca_sarok_elso_fal = 1;
/***************************/



/***************************/
//Jelzes felismer�s seg�dv�ltoz�k

//3 vonalhoz -----> Dr�n el�tt
uint8_t hanyszor_volt_3_vonal = 0;
uint8_t harmas_vonal_van = 0;
uint8_t elobb_3_vonal_volt = 0;
int32_t harmas_vonal_kezdete_encoder_ertek = 0;
int32_t harmas_vonal_vege_encoder_ertek = 0;
int32_t harmas_vonal_mostani_encoder_ertek = 0;
int32_t harmas_vonal_hossza = 0;

//0 vonalhoz ----> K�rforgalom el�tt
uint8_t hanyszor_volt_0_vonal = 0;
uint8_t nullas_vonal_van = 0;
uint8_t elobb_0_vonal_volt = 0;
int32_t nullas_vonal_kezdete_encoder_ertek = 0;
int32_t nullas_vonal_vege_encoder_ertek = 0;
int32_t nullas_vonal_mostani_encoder_ertek = 0;
int32_t nullas_vonal_hossza = 0;

//2 vonalhoz ----> Hord� el�tt
uint8_t hanyszor_volt_2_vonal = 0;
uint8_t kettes_vonal_van = 0;
uint8_t elobb_2_vonal_volt = 0;
int32_t kettes_vonal_kezdete_encoder_ertek = 0;
int32_t kettes_vonal_vege_encoder_ertek = 0;
int32_t kettes_vonal_mostani_encoder_ertek = 0;
int32_t kettes_vonal_hossza = 0;
/***************************/


void jelzes_felismeres(uint8_t vonal_szam){


/***************************************************************/
/****************** DR�N ************************/
	if(vonal_szam == 3)
	{
		if(elobb_3_vonal_volt){
			hanyszor_volt_3_vonal++;
		}

		if(hanyszor_volt_3_vonal >= 2){
			harmas_vonal_kezdete_encoder_ertek = get_encoder_counter();
			harmas_vonal_van = 1;
			hanyszor_volt_3_vonal = 0;
			elobb_3_vonal_volt = 0;
		}

		if(!harmas_vonal_van){
			elobb_3_vonal_volt = 1;
		}

	} else if(vonal_szam == 1)
	{

	}

	if(harmas_vonal_van)
	{
		harmas_vonal_vege_encoder_ertek = get_encoder_counter();
		harmas_vonal_hossza = (harmas_vonal_kezdete_encoder_ertek - harmas_vonal_vege_encoder_ertek)*ENCODER_VALUE_TO_MM;

		if(harmas_vonal_hossza > 110)
		{
//			set_gyari_motor_compare_value(6200);
			state_of_robot = DRONE_KOVETKEZIK;
		}
		if(vonal_szam == 1)
		{
			harmas_vonal_van = 0;
			elobb_3_vonal_volt = 0;
			hanyszor_volt_3_vonal = 0;
			if(harmas_vonal_hossza < 100){

			}
		}




		/*TODO		TESZT
					char bufi[10];
					itoa(harmas_vonal_kezdete_encoder_ertek, bufi, 10);
					BT_UART_SendString(bufi);
					BT_UART_SendString("\r\n");

					itoa(harmas_vonal_vege_encoder_ertek, bufi, 10);
					BT_UART_SendString(bufi);
					BT_UART_SendString("\r\n");

					itoa(harmas_vonal_hossza, bufi, 10);
					BT_UART_SendString(bufi);
					BT_UART_SendString("\r\n");
		*/
	}



	if(state_of_robot == DRONE_FELSZALLT){
		hanyszor_volt_3_vonal = 0;
		harmas_vonal_van = 0;
		elobb_3_vonal_volt = 0;
		harmas_vonal_kezdete_encoder_ertek = 0;
		harmas_vonal_vege_encoder_ertek = 0;
		harmas_vonal_mostani_encoder_ertek = 0;
		harmas_vonal_hossza = 0;
	}

/***************************************************************/
/****************** K�RFORGALOM ************************/

	if(vonal_szam == 0 && vil_ledek_szama() == 0)
	{
		if(elobb_0_vonal_volt)
		{
			hanyszor_volt_0_vonal++;
		}

		if(hanyszor_volt_0_vonal >= 2)
		{
			nullas_vonal_kezdete_encoder_ertek = get_encoder_counter();
			nullas_vonal_van = 1;
			hanyszor_volt_0_vonal = 0;
			elobb_0_vonal_volt = 0;
		}

		if(!nullas_vonal_van)
		{
			elobb_0_vonal_volt = 1;
		}

	}


	if(nullas_vonal_van)
	{
		nullas_vonal_vege_encoder_ertek = get_encoder_counter();
		nullas_vonal_hossza = (nullas_vonal_kezdete_encoder_ertek - nullas_vonal_vege_encoder_ertek)*ENCODER_VALUE_TO_MM;

		if(nullas_vonal_hossza > 110)
		{
//			BT_UART_SendString("Hosszu nullas\r\n");
		}
		if(vonal_szam == 1)
		{
			nullas_vonal_van = 0;
			elobb_0_vonal_volt = 0;
			hanyszor_volt_0_vonal = 0;
			if(nullas_vonal_hossza <= 100 && nullas_vonal_hossza >= 50){
				if(state_of_robot != KORFORGALOM_KOVETKEZIK)
				{
					state_of_robot = KORFORGALOM_KOVETKEZIK;
					korforgalom_jelzes_utani_elso_encoder_ertek = get_encoder_counter();
				}

			}
			nullas_vonal_hossza = 0;
		}
	}




/***************************************************************/
/****************** HORD� ************************/


	if(vonal_szam == 2)
	{
		if(elobb_2_vonal_volt)
		{
			hanyszor_volt_2_vonal++;
		}

		if(hanyszor_volt_2_vonal >= 2)
		{
			kettes_vonal_kezdete_encoder_ertek = get_encoder_counter();
			kettes_vonal_van = 1;
			hanyszor_volt_2_vonal = 0;
			elobb_2_vonal_volt = 0;
		}

		if(!kettes_vonal_van)
		{
			elobb_2_vonal_volt = 1;
		}

	}


	if(kettes_vonal_van)
	{
		kettes_vonal_vege_encoder_ertek = get_encoder_counter();
		kettes_vonal_hossza = (kettes_vonal_kezdete_encoder_ertek - kettes_vonal_vege_encoder_ertek)*ENCODER_VALUE_TO_MM;

		if(kettes_vonal_hossza > 110)
		{
		}
		if(vonal_szam == 1)
		{
			kettes_vonal_van = 0;
			elobb_2_vonal_volt = 0;
			hanyszor_volt_2_vonal = 0;
			if(kettes_vonal_hossza <= 100 && kettes_vonal_hossza >= 50){
				state_of_robot = HORDO_KOVETKEZIK;
			}
		}
	}

}


/**************************************************************/

/* Konvoj elhalad�s felismer�s seg�dv�ltoz�k  egyszer�bb v�ltozat*/
uint8_t auto_van = 0;
uint8_t szunet_van = 0;
uint8_t szunet_szam = 0;
uint8_t auto_szam = 0;

/**************************************************************/


void konvoj_elhaladas_felismeres()
{

	if(!autot_erzekeltem){
		if(elulso_sharp_szenzor <= 250){
			autot_erzekeltem = 1;
			auto_van = 1;
			szunet_van = 0;
			auto_szam++;
		}
	} else {

		if(auto_van){
			if(elulso_sharp_szenzor >= 1000){
				auto_van = 0;
				szunet_van = 1;
				szunet_szam++;
			}
		}

		if(szunet_van){
			if(elulso_sharp_szenzor <= 500){
				auto_van = 1;
				szunet_van = 0;
			}
		}
	}

	if(state_of_robot == VASUTI_ATJARO_KONVOJ_ELHALADASRA_VAR_ELOSZOR){
		if(varakozasi_ido >= 2000){
			state_of_robot = VASUTI_ATJARO_EGYSZER_ATHALADTUNK;
			varakozasi_ido = 0;
			autot_erzekeltem = 0;
			auto_van = 0;
			szunet_van = 0;
			szunet_szam = 0;
			auto_szam = 0;
		}
	} else if(state_of_robot == VASUTI_ATJARO_KONVOJ_ELHALADASRA_VAR_MASODJARA){
		if(varakozasi_ido >= 2000){
			state_of_robot = VASUTI_ATJARO_KETSZER_ATHALADTUNK;
			varakozasi_ido = 0;
			autot_erzekeltem = 0;
			auto_van = 0;
			szunet_van = 0;
			szunet_szam = 0;
			auto_szam = 0;
		}
	}
}



void konvoj_elhaladas_felismeres_oldalt()
{

	if(state_of_robot == KONVOJ_JELZES_BAL_KOZELEDES)
	{
		if(!autot_erzekeltem){
			if(jobb_oldali_sharp_szenzor <= 250){
				autot_erzekeltem = 1;
				auto_van = 1;
				szunet_van = 0;
				auto_szam++;
			}
		} else {

			if(auto_van){
				if(jobb_oldali_sharp_szenzor >= 300){
					auto_van = 0;
					szunet_van = 1;
					szunet_szam++;
				}
			}

			if(szunet_van){
				if(jobb_oldali_sharp_szenzor <= 250){
					auto_van = 1;
					szunet_van = 0;
				}
			}
		}

		if(state_of_robot == VASUTI_ATJARO_KONVOJ_ELHALADASRA_VAR_ELOSZOR){
			if(varakozasi_ido >= 2000){
				state_of_robot = VASUTI_ATJARO_EGYSZER_ATHALADTUNK;
				varakozasi_ido = 0;
				autot_erzekeltem = 0;
				auto_van = 0;
				szunet_van = 0;
				szunet_szam = 0;
				auto_szam = 0;
			}
		}
	}
	else if(state_of_robot == KONVOJ_JELZES_JOBB_KOZELEDES)
	{
		if(!autot_erzekeltem){
			if(bal_oldali_sharp_szenzor <= 250){
				autot_erzekeltem = 1;
				auto_van = 1;
				szunet_van = 0;
				auto_szam++;
			}
		} else {

			if(auto_van){
				if(bal_oldali_sharp_szenzor >= 300){
					auto_van = 0;
					szunet_van = 1;
					szunet_szam++;
				}
			}

			if(szunet_van){
				if(bal_oldali_sharp_szenzor <= 250){
					auto_van = 1;
					szunet_van = 0;
				}
			}
		}

		if(state_of_robot == VASUTI_ATJARO_KONVOJ_ELHALADASRA_VAR_ELOSZOR){
			if(varakozasi_ido >= 2000){
				state_of_robot = VASUTI_ATJARO_EGYSZER_ATHALADTUNK;
				varakozasi_ido = 0;
				autot_erzekeltem = 0;
				auto_van = 0;
				szunet_van = 0;
				szunet_szam = 0;
				auto_szam = 0;
			}
		}

	}




}

/**************************************************************/

/* Keresztvonal felismer�s seg�dv�ltoz�k  */

uint8_t hanyszor_volt_kereszt_vonal = 0;
uint8_t kereszt_vonal_van = 0;
uint8_t elobb_kereszt_vonal_volt = 0;
int32_t kereszt_vonal_utani_egyes_kezdete_encoder_ertek = 0;
int32_t kereszt_vonal_utani_egyes_vege_encoder_ertek = 0;
int32_t kereszt_vonal_utani_egyes_mostani_encoder_ertek = 0;
int32_t kereszt_vonal_utani_egyes_hossza = 0;
uint8_t egyes_vonal_volt_a_kereszt_utan = 0;


/**************************************************************/


void kereszt_vonal_felismeres(uint8_t ledek_szama)
{

	if(ledek_szama >= 30)
		{


			if(egyes_vonal_volt_a_kereszt_utan)
			{
				if(kereszt_vonal_utani_egyes_hossza <= 50){

					if(v_a_elso_dupla_kereszt_mar_meg_volt)
					{
						v_a_masodiik_dupla_kereszt_is_meg_volt = 1;
					} else
					{
						state_of_robot = VASUTI_ATJARO_KOVETKEZIK;
						v_a_elso_dupla_kereszt_mar_meg_volt = 1;
					}

				}
			}
			if(elobb_kereszt_vonal_volt){
				hanyszor_volt_kereszt_vonal++;
			}

			if(hanyszor_volt_kereszt_vonal >= 1){
				kereszt_vonal_van = 1;
				hanyszor_volt_kereszt_vonal = 0;
				elobb_kereszt_vonal_volt = 0;
			}

			if(!kereszt_vonal_van){
				elobb_kereszt_vonal_volt = 1;
			}

		}

		if(kereszt_vonal_van)
		{

/**********************************************************************/
			/* Egy m�dja a vas�ti �tj�r� el�tti meg�ll�snak, hogy
			 * A keresztvonal �rz�kel�se ut�n kikapcsolom a motort
			 */
/**********************************************************************/
			if(state_of_robot == VASUTI_ATJARO_KOVETKEZIK){
				meg_volt_a_kereszt_vonal = 1;
			}


			if(ledek_szama <= 5){

				if(!egyes_vonal_volt_a_kereszt_utan)
				{
					kereszt_vonal_utani_egyes_kezdete_encoder_ertek = get_encoder_counter();
					egyes_vonal_volt_a_kereszt_utan = 1;
				}
				else
				{
					kereszt_vonal_utani_egyes_mostani_encoder_ertek = get_encoder_counter();
					kereszt_vonal_utani_egyes_hossza = (kereszt_vonal_utani_egyes_kezdete_encoder_ertek - kereszt_vonal_utani_egyes_mostani_encoder_ertek)*ENCODER_VALUE_TO_MM;

					if(kereszt_vonal_utani_egyes_hossza >= 50)
					{
						if(state_of_robot == CEL_KOVETKEZIK){
							state_of_robot = GYOZELEM;
						}

					}
				}

			}
		}
}


/**************************************************************/

/* fal felismer�s seg�dv�ltoz�k  */
uint8_t jobb_oldalon_fal_van = 0;
int32_t jobb_oldali_fal_kezdete_encoder_ertek;
int32_t jobb_oldali_fal_mostani_encoder_ertek;
int32_t jobb_oldali_fal_hossza = 0;

uint8_t bal_oldalon_fal_van = 0;
int32_t bal_oldali_fal_kezdete_encoder_ertek;
int32_t bal_oldali_fal_mostani_encoder_ertek;
int32_t bal_oldali_fal_hossza = 0;

/**************************************************************/

void fal_felismeres(){


	if(meg_nem_volt_utca_sarok_elso_fal){
		if(jobb_oldali_sharp_szenzor <= 250 && bal_oldali_sharp_szenzor <= 250)
		{
			state_of_robot = UTCA_SAROK_DUPLA_FAL;
			meg_nem_volt_utca_sarok_elso_fal = 0;
		}
	}


	if(state_of_robot == UTCA_SAROK_DUPLA_FAL)
	{

		if(jobb_oldali_sharp_szenzor >= 300 && bal_oldali_sharp_szenzor >= 300){
			state_of_robot = UTCA_SAROK_DUPLA_FAL_UTANI_SZUNET;
		}
	}

	if(state_of_robot == UTCA_SAROK_DUPLA_FAL_UTANI_SZUNET)
	{
		if(jobb_oldali_sharp_szenzor <= 250 && bal_oldali_sharp_szenzor >= 300){
			state_of_robot = UTCA_SAROK_MASODIK_FAL_JOBB;
		}

		if(jobb_oldali_sharp_szenzor >= 300 && bal_oldali_sharp_szenzor <= 250){
			state_of_robot = UTCA_SAROK_MASODIK_FAL_BAL;
		}
	}

	if(state_of_robot == UTCA_SAROK_MASODIK_FAL_JOBB || state_of_robot == UTCA_SAROK_MASODIK_FAL_BAL ){

		if(jobb_oldali_sharp_szenzor >= 300 && bal_oldali_sharp_szenzor >= 300){
			state_of_robot = UTCA_SAROK_MASODIK_FAL_UTANI_SZUNET;
		}
	}

	if(state_of_robot == JUST_GOING)
	{
		if(jobb_oldali_sharp_szenzor <= 250 && bal_oldali_sharp_szenzor >= 300){

			if(!jobb_oldalon_fal_van)
			{
				jobb_oldalon_fal_van = 1;
				jobb_oldali_fal_kezdete_encoder_ertek = get_encoder_counter();
			} else {
				jobb_oldali_fal_mostani_encoder_ertek = get_encoder_counter();
				jobb_oldali_fal_hossza = (jobb_oldali_fal_kezdete_encoder_ertek - jobb_oldali_fal_mostani_encoder_ertek)*ENCODER_VALUE_TO_MM;

				if(jobb_oldali_fal_hossza >= 50){
					state_of_robot = KONVOJ_KOVETKEZIK_JELZES_A_JOBB_OLDALON;
				}
			}
		}

		if(jobb_oldali_sharp_szenzor >= 300 && bal_oldali_sharp_szenzor <= 250){


			if(!bal_oldalon_fal_van)
			{
				bal_oldalon_fal_van = 1;
				bal_oldali_fal_kezdete_encoder_ertek = get_encoder_counter();
			} else {
				bal_oldali_fal_mostani_encoder_ertek = get_encoder_counter();
				bal_oldali_fal_hossza = (bal_oldali_fal_kezdete_encoder_ertek - bal_oldali_fal_mostani_encoder_ertek)*ENCODER_VALUE_TO_MM;

				if(bal_oldali_fal_hossza >= 50){
					state_of_robot = KONVOJ_KOVETKEZIK_JELZES_A_BAL_OLDALON;
				}
			}
		}
	}
}


void korforgalom_jelzes_felismeres()
{
	if(korforgalom_cim_stimmel){

		switch(korforgalom_uzenet){

		case JOBBRA_ELSO:
			state_of_robot = KORFORG_JOBBRA_1;
			break;

		case JOBBRA_MASODIK:
			state_of_robot = KORFORG_JOBBRA_2;
			break;

		case JOBBRA_HARMADIK:
			state_of_robot = KORFORG_JOBBRA_3;
			break;

		case BALRA_ELSO:
			state_of_robot = KORFORG_BALRA_1;
			break;

		case BALRA_MASODIK:
			state_of_robot = KORFORG_BALRA_2;
			break;

		case BALRA_HARMADIK:
			state_of_robot = KORFORG_BALRA_3;
			break;

		default:
			break;

		}
	}
}


void send_adc_values_over_bt(void){
	uint8_t i = 0;
	char buff[5];

	BT_UART_SendString("Hatso_szenzor:\r\n");
	for(i = 0; i<32; i++){
		buff[0] = 0;
		buff[1] = 0;
		buff[2] = 0;
		buff[3] = 0;
		buff[4] = 0;
		itoa(adcAdatok_buffer_hatso[i], buff, 10);
		BT_UART_SendString(buff);
		BT_UART_SendString(" ");
	}
	BT_UART_SendString("\r\n");
	BT_UART_SendString("Elso_szenzor:\r\n");
	for(i = 0; i<32; i++){
		buff[0] = 0;
		buff[1] = 0;
		buff[2] = 0;
		buff[3] = 0;
		buff[4] = 0;
		itoa(adcAdatok_buffer_elso[i], buff, 10);
		BT_UART_SendString(buff);
		BT_UART_SendString(" ");
	}
	BT_UART_SendString("\r\n");
}


void send_encoder_values_over_uart(){
	char buff[20];
	itoa(encoder_value, buff, 10);
	BT_UART_SendString(buff);
	BT_UART_SendString("\r\n");


/*
	itoa(ic2, buff, 10);
	BT_UART_SendString(buff);
	BT_UART_SendString("\r\n");

	*/
}


void sebesseg_szabalyzas(){
	if(sebesseg_tarto_counter > 5){
		speed_diff = wanted_speed - speed_of_drogon;

		integrator_ertek += speed_diff;
		motor_value = 6480 + (int)(speed_diff*KP_speed) + (int)(KI_speed*integrator_ertek);

		if(motor_value > GYARI_MOTOR_COUNTER_MAX){
			motor_value = GYARI_MOTOR_COUNTER_MAX;
		} else if(motor_value < GYARI_MOTOR_COUNTER_KOZEP){
			motor_value = GYARI_MOTOR_COUNTER_KOZEP;
		}
		set_gyari_motor_compare_value(motor_value);

	sebesseg_tarto_counter = 0;
	} else {
	sebesseg_tarto_counter++;
	}
}

