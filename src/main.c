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


//Akadály konstansok
#include "utca_sarok.h"
#include "korforgalom.h"
uint8_t teszt_helyzet = 1;
uint8_t start_kapu_fala_meg_volt = 0;

//SPI1
SPI_HandleTypeDef spi;

uint32_t adc_eredmeny = 0;

//SPI fogadási flag
uint8_t data_received = 1;

uint16_t KP_kormany = 0;
uint16_t KD_kormany = 0;


//Szabályozás

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


//PWM compare értékek állítása



uint16_t KP_speed = 80;
uint16_t KI_speed = 5;


uint16_t KP_speed_konvoj = 160;
uint16_t KI_speed_konvoj = 5;

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

//Hány vonalat érzékelünk
uint8_t vonalak = 0;

uint16_t mag_dec = 0;
uint32_t encoder_value = 0;
uint8_t vonalak_szama_a_mereskor = 0;

Robot_state state_of_robot = START;

float wanted_speed = 0.9f;
//float wanted_speed = 0.5f;

float wanted_speed_konvoj = 0.7f;
//float wanted_speed = 1.1f;


uint8_t sebesseg_tarto_counter = 0;

uint8_t korforgalom_meg_volt = 0;

float speed_diff = 0;

int32_t x_acc,y_acc,z_acc;

int main()
{

 	KP_kormany = KP_slow;
	KD_kormany = KD_slow;

	// HAL_Init, System_Clock_config és hardware inicializáció
    init_all();
    set_gyari_motor_compare_value(6200);
    HAL_Delay(2000);



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
//
//			kormany_szabalyzas_on = 0;
			ciklus();
/*


			szervo_value = DIGIT_SZ_KOZEP + (uint16_t)(x*30);
			set_compare_value_digit_szervo(szervo_value);
			*/
/*
			BT_UART_SendString("E:  ");
			itoa((int)(meg_jott_a_start_kapu_jele), buffer, 10);
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

/*
			if(teszt_helyzet)
			{
				BT_UART_SendString("x: ");
				itoa((int)(x), buffer, 10);
				BT_UART_SendString(buffer);
				BT_UART_SendString("    ");

				BT_UART_SendString("Y: ");
				itoa((int)(y), buffer, 10);
				BT_UART_SendString(buffer);
				BT_UART_SendString("   ");

				BT_UART_SendString("z: ");
				itoa((int)(z), buffer, 10);
				BT_UART_SendString(buffer);
				BT_UART_SendString("\r\n");
			}
			*/
		}

	}

}



uint8_t mar_meg_volt_a_drone = 0;

char parameter_buffer[50];
int32_t p_konv;
int32_t D_konv;

uint32_t cycle_counter = 0;
uint8_t dir = 0;

uint8_t elozo_vonalak[3];
uint8_t vonal_szamlalo = 0;
uint8_t csucs_kereses = 0;
uint8_t dummy_flag = 0;





//Az elsõ SPI adatfogadás hibás
//TODO: Gyere rá miért
uint8_t first_cycle = 1;



/*****************************************/
//Gyorsasági pálya jelzés felismerés
/*****************************************/
#define ONE_CYCLE_MEASERES 100
uint8_t vonalak_szama_sorrendben[ONE_CYCLE_MEASERES];
uint16_t felismeres_cycle_counter = 0;
uint8_t adott_meres_vonal_szama = 0;
uint8_t meresi_cikluson_beluli_3as_vonal_szam = 0;
/*****************************************/



/*****************************************/
//Változók az spi adatfogadáshoz
//és pozíció számoláshoz mindkét szenzorsor esetén
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
/*                 UTCASAROK segédváltozók  */

//JOBB
int32_t utca_sarok_jobb_fal_kezdet_encoder = 0;
int32_t utca_sarok_jobb_fal_mostani_encoder = 0;
uint8_t utca_sarok_jobb_fal_kezdet = 1;
int32_t utca_sarok_jobb_fal_hossz = 0;

int32_t jobb_tolatas_hossz = 0;
int32_t jobb_tolatas_kezdet_encoder_ertek = 0;
int32_t jobb_tolatas_mostani_encoder_ertek = 0;

uint8_t eloszor_van_jobb_tolatas = 1;


int32_t utca_sarok_tolatas_utani_encoder_ertek = 0;
int32_t utca_sarok_tolatas_utani_mostani_encoder_ertek = 0;
int32_t utca_sarok_utani_eloremenet_hossza = 0;
uint8_t utca_sarok_most_indulunk_elore = 1;


//BAL
int32_t utca_sarok_bal_fal_kezdet_encoder = 0;
int32_t utca_sarok_bal_fal_mostani_encoder = 0;
uint8_t utca_sarok_bal_fal_kezdet = 1;
int32_t utca_sarok_bal_fal_hossz = 0;

int32_t bal_tolatas_hossz = 0;
int32_t bal_tolatas_kezdet_encoder_ertek = 0;
int32_t bal_tolatas_mostani_encoder_ertek = 0;

uint8_t eloszor_van_bal_tolatas = 1;

uint8_t utca_egyenesen_hatra = 0;


//MINDKETTÕ
uint8_t utca_sarok_megalltunk = 0;
uint8_t arasz_nulla_led_vilagitott_elobb = 0;
uint8_t arasz_hanyszor_volt_nulla = 0;
uint8_t tolatas_veget_ert = 1;
/*************************************************/



/*************************************************/
/*                 VASÚTI ÁTJÁRÓ segédváltozók  */
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


uint8_t egy_vonal_volt_elobb = 0;
uint8_t elobb_egy_vonal_volt = 0;
uint8_t hanyszor_volt_egy_vonal = 0;
uint8_t mar_egy_vonalon_voltunk = 0;


uint8_t fekezni_kell = 0;
/*************************************************/


/*************************************************/
/*   KÖRFORGALOM segédváltozók  */
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

/*          BALRA 1                   */
int32_t b1_kezdet_encoder_ertek = 0;
int32_t b1_mostani_encoder_ertek = 0;
uint8_t b1_most_kezd_merni = 1;
int32_t b1_hossz = 0;



/*   Fix ívek amiken fordulunk  */
int32_t kor_ford_kezdet_encoder_ertek = 0;
int32_t kor_ford_mostani_encoder_ertek = 0;
int32_t kor_ford_hossz = 0;
uint8_t kor_ford_most_kezd_merni = 1;
uint8_t kor_az_elso_iv_megvolt = 0;
uint8_t kor_a_masodik_iv_megvolt = 0;
/*************************************************/


/*************************************************/
/*                 KONVOJ segédváltozók  */
uint8_t konvoj_kozeledes_eloszor = 1;
int32_t konvoj_koz_kezdet_encoder_ertek = 0;
int32_t konvoj_koz_mostani_encoder_ertek = 0;
int32_t konvoj_koz_hossz = 0;


uint8_t konvoj_vonalra_allas_eloszor = 1;
int32_t konvoj_von_kezdet_encoder_ertek = 0;
int32_t konvoj_von_mostani_encoder_ertek = 0;
int32_t konvoj_von_hossz = 0;

uint8_t megtettuk_a_kort = 0;
uint8_t lejottunk_a_korrol = 0;
uint8_t nulla_led_volt = 0;
uint8_t hanyszor_volt_nulla_led = 0;


uint8_t nem_nulla_led_volt = 0;
uint8_t hanyszor_volt_nem_nulla_led = 0;

uint8_t konv_elso_iven = 1;
uint8_t konv_masodik_iven = 0;


/*************************************************/

/*************************************************/
/*                 HORDÓ segédváltozók          */

int32_t hordo_jel_utan_tav_kezdet_encoder_ertek = 0;
int32_t hordo_jel_utan_tav_mostani_encoder_ertek = 0;
uint8_t hordo_jel_utan_elso_meres = 1;
int32_t hordo_jel_utan_tav_hossz = 0;



/*************************************************/
void ciklus(){

	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
	szumma_adc_values_elso = 0;
	szumma_sulyozott_elso = 0;

	szumma_adc_values_hatso = 0;
	szumma_sulyozott_hatso = 0;


	// Adatfogadás kezdete
	data_received = 0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_SPI_Receive_IT(&spi, adcAdatok_buffer_hatso, 64);
	while(!data_received);


	data_received = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_SPI_Receive_IT(&spi, adcAdatok_buffer_elso, 64);
	while(!data_received);

	// Adatfogadás vége

	if(first_cycle){
		first_cycle = 0;

		meres_kezdeti_encoder_ertek = get_encoder_counter();
	} else {

		// Csak a világító ledekhez tartozó AD adatokat tartjuk meg
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
		/* Itt kell még módosítani az adcAdatokon, ha tolatni akarunk  */
/****************************************************************/
		if(state_of_robot == UTCA_SAROK_JOBB_TOLATAS_VEGE){
			utca_sarok_jobb_elofeldolgozas();
		} else if(state_of_robot == UTCA_SAROK_BAL_TOLATAS_VEGE)
		{
			utca_sarok_bal_elofeldolgozas();
		}



		//arányos tényezõ számítása
		for(int i = 0; i < 32; i++)
		{
			szumma_adc_values_elso += adcAdatok_elso[i];
			adcAdatok_sulyozott_elso[i] = adcAdatok_elso[i] * sorszam[i];			//súlyozás
			szumma_sulyozott_elso += adcAdatok_sulyozott_elso[i];

			szumma_adc_values_hatso += adcAdatok_hatso[i];
			adcAdatok_sulyozott_hatso[i] = adcAdatok_hatso[i] * sorszam[i];			//súlyozás
			szumma_sulyozott_hatso += adcAdatok_sulyozott_hatso[i];

		}

		//Elsõ szenzorsor pozíció
		if(szumma_adc_values_elso != 0){
			p_elso = (float)szumma_sulyozott_elso/szumma_adc_values_elso;
		} else {
			p_elso = 0;
		}

		//Hátsó szenzorsor pozíció
		if(szumma_adc_values_hatso != 0){
			p_hatso = (float)szumma_sulyozott_hatso/szumma_adc_values_hatso;
		} else {
			p_hatso = 0;
		}

/****************************************************************/

		/*** Idáig tart a pozíció kiszámolása ***/
/****************************************************************/
//PD szabályzoó egy vonalszenzor alapján

		//Differencia az elõzõ pozíció hibától
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
			/* DRÓN ÁLLAPOTAI */
/****************************************************/

		case DRONE_ELOTT_ALLUNK:

			if(teszt_helyzet) {
				BT_UART_SendString("DRONE all\r\n");
			}


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

			if(teszt_helyzet) {
				BT_UART_SendString("DRONE köv\r\n");
			}

			wanted_speed = (elulso_sharp_szenzor - 450)*0.00057f;
			if(elulso_sharp_szenzor <= 600){
				wanted_speed = 0.0f;
//					sebesseg_szabalyzas_elore_on = 0;
//					set_gyari_motor_compare_value(00);
					state_of_robot = DRONE_ELOTT_ALLUNK;
			}
			break;
		case DRONE_FELSZALLT:

			if(teszt_helyzet)
			{
				BT_UART_SendString("DRONE szal\r\n");
			}


			start_milisec_szamlalo = 1;
			if(milisec_szamlalo > 2100){
//				sebesseg_szabalyzas_elore_on = 1;
				mar_meg_volt_a_drone = 1;
				state_of_robot = JUST_GOING;
			}
			break;

/****************************************************/
			/* UTCASAROK ÁLLAPOTAI */
/****************************************************/

		case UTCA_SAROK_DUPLA_FAL:


			if(teszt_helyzet){
				BT_UART_SendString("DUPLA FAL\r\n");
			}


			fal_felismeres();

			wanted_speed = 0.8f;
			break;

		case UTCA_SAROK_DUPLA_FAL_UTANI_SZUNET:
			if(teszt_helyzet)
			{
				BT_UART_SendString("DUPLA FAL U SZ\r\n");
			}
			fal_felismeres();
			break;

		case UTCA_SAROK_MASODIK_FAL_JOBB:

			if(teszt_helyzet)
			{
				BT_UART_SendString("2. J\r\n");
			}
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
			if(teszt_helyzet)
			{
				BT_UART_SendString("J ARASZ\r\n");
			}

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
			if(teszt_helyzet)
			{
				BT_UART_SendString("J TOLAT\r\n");
			}

			if(eloszor_van_jobb_tolatas) {
				set_gyari_motor_compare_value(5600);
				eloszor_van_jobb_tolatas = 0;
				jobb_tolatas_kezdet_encoder_ertek = get_encoder_counter();
			} else {
				jobb_tolatas_mostani_encoder_ertek = get_encoder_counter();
				jobb_tolatas_hossz = (jobb_tolatas_mostani_encoder_ertek - jobb_tolatas_kezdet_encoder_ertek)*ENCODER_VALUE_TO_MM;
			}

			if(jobb_tolatas_hossz >= JOBB_MENNYIT_TOLATUNK_MM){
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

				if(jobb_tolatas_hossz >= JOBB_MIKORTOL_EGYENESEN_MM)
				{
					set_compare_value_digit_szervo(JOBB_MASODIK_SZOG);
					utca_egyenesen_hatra = 1;
				}

				if(!varjuk_meg_a_kozep_erteket){
					if(!varjuk_meg_a_hatra_erteket){
						varjuk_meg_a_hatra_erteket = 1;
						set_gyari_motor_compare_value(5800);
					} else {

						if(hatra_ido_milisec >= 300){
							varjuk_meg_a_kozep_erteket = 1;
							set_gyari_motor_compare_value(6200);
							megvartuk_a_hatrat = 1;
						}
					}
				} else {
					if(kozep_ido_milisec >= 300){
						megvartuk_a_kozepet = 1;
						set_compare_value_digit_szervo(JOBB_ELSO_SZOG);
						set_gyari_motor_compare_value(5600);
					}
				}
			}
			kormany_szabalyzas_on = 0;
			sebesseg_szabalyzas_elore_on = 0;
			break;


		case UTCA_SAROK_JOBB_TOLATAS_VEGE:

			if(teszt_helyzet){
				BT_UART_SendString("J T VEGE\r\n");
			}
			if(utca_sarok_most_indulunk_elore){
				utca_sarok_most_indulunk_elore = 0;
				utca_sarok_tolatas_utani_encoder_ertek = get_encoder_counter();
			} else {
				utca_sarok_tolatas_utani_mostani_encoder_ertek = get_encoder_counter();
				utca_sarok_utani_eloremenet_hossza = (utca_sarok_tolatas_utani_encoder_ertek - utca_sarok_tolatas_utani_mostani_encoder_ertek)*ENCODER_VALUE_TO_MM;
				if(utca_sarok_utani_eloremenet_hossza >= 600){
					state_of_robot = JUST_GOING;
				}
			}


			kormany_szabalyzas_on = 1;
			set_gyari_motor_compare_value(6510);

			break;

		case UTCA_SAROK_MASODIK_FAL_BAL:

			if(teszt_helyzet)
			{
				BT_UART_SendString("2.B\r\n");
			}

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
			if(teszt_helyzet)
			{
				BT_UART_SendString("B ARASZ\r\n");
			}


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


//A vonal végéig megyünk
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


//Miután megálltunk
			break;

		case UTCA_SAROK_BAL_FALAS_TOLATAS:

			if(teszt_helyzet)
			{
				BT_UART_SendString("B TOLAT\r\n");
			}

			if(eloszor_van_bal_tolatas) {
				set_gyari_motor_compare_value(5600);
				eloszor_van_bal_tolatas = 0;
				bal_tolatas_kezdet_encoder_ertek = get_encoder_counter();
			} else {
				bal_tolatas_mostani_encoder_ertek = get_encoder_counter();
				bal_tolatas_hossz = (bal_tolatas_mostani_encoder_ertek - bal_tolatas_kezdet_encoder_ertek)*ENCODER_VALUE_TO_MM;
			}

			if(bal_tolatas_hossz >= BAL_MENNYIT_TOLATUNK_MM){
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

				if(bal_tolatas_hossz >= BAL_MIKORTOL_EGYENESEN_MM)
				{
					set_compare_value_digit_szervo(BAL_MASODIK_SZOG);
					utca_egyenesen_hatra = 1;
				}



				if(!varjuk_meg_a_kozep_erteket)
				{
					if(!varjuk_meg_a_hatra_erteket)
					{
						varjuk_meg_a_hatra_erteket = 1;
						set_gyari_motor_compare_value(5600);
					} else {
						if(hatra_ido_milisec >= 300)
						{
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
						if(!utca_egyenesen_hatra){
							set_compare_value_digit_szervo(BAL_ELSO_SZOG);
						}
						set_gyari_motor_compare_value(5600);
					}
				}
			}
			kormany_szabalyzas_on = 0;
			sebesseg_szabalyzas_elore_on = 0;

			break;


		case UTCA_SAROK_BAL_TOLATAS_VEGE:

			if(teszt_helyzet){
				BT_UART_SendString("B T VEGE\r\n");
			}
			if(utca_sarok_most_indulunk_elore){
				utca_sarok_most_indulunk_elore = 0;
				utca_sarok_tolatas_utani_encoder_ertek = get_encoder_counter();
			} else {
				utca_sarok_tolatas_utani_mostani_encoder_ertek = get_encoder_counter();
				utca_sarok_utani_eloremenet_hossza = (utca_sarok_tolatas_utani_encoder_ertek - utca_sarok_tolatas_utani_mostani_encoder_ertek)*ENCODER_VALUE_TO_MM;
				if(utca_sarok_utani_eloremenet_hossza >= 600){
					state_of_robot = JUST_GOING;
				}
			}


			kormany_szabalyzas_on = 1;
			set_gyari_motor_compare_value(6510);
			break;


		case UTCA_SAROK_MASODIK_FAL_UTANI_SZUNET:

			if(teszt_helyzet)
			{
				BT_UART_SendString("2. SZÜN\r\n");
			}

			set_gyari_motor_compare_value(5800);
			break;



/****************************************************/
			/* KONVOJ ÁLLAPOTAI */
/****************************************************/

		case KONVOJ_KOVETKEZIK_JELZES_A_JOBB_OLDALON:

			if(teszt_helyzet)
			{
				BT_UART_SendString("KONV J\r\n");
			}

			set_gyari_motor_compare_value(5600);
			if(speed_of_drogon <= 0.01f){
				state_of_robot = KONVOJ_JELZES_JOBB_KOZELEDES;
			}

			break;

		case KONVOJ_JELZES_JOBB_KOZELEDES:
			if(teszt_helyzet)
			{
				BT_UART_SendString("KONV J KÖZ\r\n");
			}

			kormany_szabalyzas_on = 0;
			if(konvoj_kozeledes_eloszor){
				BT_UART_SendString("ELSO K J K\r\n");
				set_compare_value_digit_szervo(39000);
				sebesseg_szabalyzas_elore_on = 0;
				set_gyari_motor_compare_value(6510);
				konvoj_kozeledes_eloszor = 0;
				konvoj_koz_kezdet_encoder_ertek = get_encoder_counter();
			} else {
				konvoj_koz_mostani_encoder_ertek = get_encoder_counter();
				konvoj_koz_hossz = (konvoj_koz_kezdet_encoder_ertek - konvoj_koz_mostani_encoder_ertek)*ENCODER_VALUE_TO_MM;


				if(konvoj_koz_hossz >= 400){
					set_gyari_motor_compare_value(6200);
					state_of_robot = KONVOJ_JELZES_JOBB_RAALLAS;
				}
			}
			break;

		case KONVOJ_JELZES_JOBB_RAALLAS:

			if(teszt_helyzet)
			{
				BT_UART_SendString("KONV J RÁ\r\n");
			}

//			set_compare_value_digit_szervo(32500);
			konvoj_elhaladas_felismeres_korbe();

			break;

		case KONVOJ_JELZES_JOBB_KORRE_MENET:

			if(teszt_helyzet)
			{
				BT_UART_SendString("KONV J KORRE\r\n");
			}


			if(konvoj_vonalra_allas_eloszor)

			{
				set_gyari_motor_compare_value(6520);
				konvoj_vonalra_allas_eloszor = 0;
				set_compare_value_digit_szervo(29500);
				konvoj_von_kezdet_encoder_ertek = get_encoder_counter();
			} else {
				konvoj_von_mostani_encoder_ertek = get_encoder_counter();
				konvoj_von_hossz = (konvoj_von_kezdet_encoder_ertek - konvoj_von_mostani_encoder_ertek)*ENCODER_VALUE_TO_MM;
				if(konvoj_von_hossz >= 200){
					kormany_szabalyzas_on = 1;
					BT_UART_SendString("KORM ON\r\n");
					state_of_robot = KONVOJ_JELZES_JOBB_KOVETES;
				}

				if(vil_ledek_szama() != 0 && p_elso <= 8){


					kormany_szabalyzas_on = 1;
					state_of_robot = KONVOJ_JELZES_JOBB_KOVETES;
	//				sebesseg_szabalyzas_elore_on = 1;
	//				wanted_speed = 0.5f;
				}
			}


			break;


		case KONVOJ_JELZES_JOBB_KOVETES:


			BT_UART_SendString("WS: ");
			itoa((int)(wanted_speed_konvoj*1000), buf10, 10);
			BT_UART_SendString(buf10);
			BT_UART_SendString("\r\n");


			BT_UART_SendString("M: ");
			itoa(motor_value, buf10, 10);
			BT_UART_SendString(buf10);
			BT_UART_SendString("\r\n");

			konvoj_von_mostani_encoder_ertek = get_encoder_counter();
			konvoj_von_hossz = (konvoj_von_kezdet_encoder_ertek - konvoj_von_mostani_encoder_ertek)*ENCODER_VALUE_TO_MM;

			if(konvoj_von_hossz >= 8700 && megtettuk_a_kort == 0)
			{
				megtettuk_a_kort = 1;
				kormany_szabalyzas_on = 0;
			}

			if(!megtettuk_a_kort)
			{
				konvoj_kovetes_sebesseg_allitas();
				sebesseg_szabalyzas_konvojhoz();
			} else {


				BT_UART_SendString("Megttt \r\n");
				kormany_szabalyzas_on = 0;
				set_gyari_motor_compare_value(6510);

				if(konv_elso_iven){
					set_compare_value_digit_szervo(27000);
				}


				if(konvoj_von_hossz >= 9100 && !konv_masodik_iven){
					konv_elso_iven = 0;
					set_compare_value_digit_szervo(39000);
				}

				if(konvoj_von_hossz >= 9600)
				{
					konv_masodik_iven = 1;
					kormany_szabalyzas_on = 1;
				}


/*
				if(!lejottunk_a_korrol)
				{
					if(vil_ledek_szama() == 0)
					{
						if(!nulla_led_volt){
							nulla_led_volt = 1;
						} else
						{
							hanyszor_volt_nulla_led++;
							if(hanyszor_volt_nulla_led >= 2)
							{
								lejottunk_a_korrol = 1;
							}
						}
					}
				}
				else {
					kormany_szabalyzas_on = 1;
					state_of_robot = JUST_GOING;
				}
*/
			}

			break;

		case KONVOJ_KOVETKEZIK_JELZES_A_BAL_OLDALON:

			if(teszt_helyzet)
			{
				BT_UART_SendString("KONV B\r\n");
			}

			set_gyari_motor_compare_value(6200);
			while(1){

			}

			break;

		case KONVOJ_JELZES_BAL_KOZELEDES:
			break;


		case KONVOJ_JELZES_BAL_RAALLAS:
			break;

/****************************************************/
			/* KÖRFORGALOM ÁLLAPOTAI */
/****************************************************/
		case KORFORGALOM_KOVETKEZIK:

			if(teszt_helyzet)
			{
				BT_UART_SendString("KÖRFO\r\n");
			}


			korforg_jel_utani_mostani_encoder_ertek = get_encoder_counter();
			korforg_jel_utani_hossz = (korforgalom_jelzes_utani_elso_encoder_ertek - korforg_jel_utani_mostani_encoder_ertek)*ENCODER_VALUE_TO_MM;

			itoa((int)korforg_jel_utani_hossz, buf10, 10);
			BT_UART_SendString(buf10);
			BT_UART_SendString("\r\n");


			if(!infra_inicializalva){
				init_infra_timer();
				infra_inicializalva = 1;
			}

			if(!megalltunk){
				set_gyari_motor_compare_value(5600);
				mostani_encoder_ertek = get_encoder_counter();

				if(mostani_encoder_ertek == elozo_encoder_ertek){
					BT_UART_SendString("MEGALLTUNK\r\n");
					megalltunk = 1;
				}
				elozo_encoder_ertek = mostani_encoder_ertek;

				if(speed_of_drogon  <= 0.1f){
					megalltunk = 1;
					BT_UART_SendString("DROGON\r\n");
				}
			} else {
/*			 Elsõ sharp alapján közeltjük meg a körforgalmat.
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

/* Megtett távolság alapján közelítjük a körforgalmat*/
				if(korforg_jel_utani_hossz <= 550){
					set_gyari_motor_compare_value(6510);

					BT_UART_SendString("HEL\r\n");
				} else {
					set_gyari_motor_compare_value(5600);
					mostani_encoder_ertek = get_encoder_counter();

					if(mostani_encoder_ertek == elozo_encoder_ertek){
						BT_UART_SendString("JEL F ISM\r\n");
						korforgalom_jelzes_felismeres();
					}

					if(speed_of_drogon < 0.1f){
						korforgalom_jelzes_felismeres();
					}
					elozo_encoder_ertek = mostani_encoder_ertek;
				}
			}

/****************************************************/
/*Egy mód a megállásra */

			sebesseg_szabalyzas_elore_on = 0;
//			wanted_speed = 0.0f;
			break;

		case KORFORG_JOBBRA_1:


			if(teszt_helyzet)
			{
				BT_UART_SendString("K J 1\r\n");
			}
			set_gyari_motor_compare_value(6520);

			if(j1_most_kezd_merni)
			{
				j1_most_kezd_merni = 0;
				set_compare_value_digit_szervo(J1_ELSO_SZERVO_SZOG);
				kormany_szabalyzas_on = 0;
				j1_kezdet_encoder_ertek = get_encoder_counter();

			} else {
				j1_mostani_encoder_ertek = get_encoder_counter();
				j1_hossz = (j1_kezdet_encoder_ertek - j1_mostani_encoder_ertek )*ENCODER_VALUE_TO_MM;
				if(!kor_az_elso_iv_megvolt)
				{
					if(j1_hossz >= J1_ELSO_IV_HOSSZ){
						kor_az_elso_iv_megvolt = 1;
					}
				} else {
					
					if(!kor_a_masodik_iv_megvolt){
						
						if(j1_hossz >= J1_MASODIK_IV_HOSSZ){
							kor_a_masodik_iv_megvolt = 1;
						} else {
							set_compare_value_digit_szervo(J1_MASODIK_SZERVO_SZOG);
						}
					} else {
						if(j1_hossz >= J1_HARMADIK_IV_HOSSZ){
							kormany_szabalyzas_on = 1;
							state_of_robot = JUST_GOING;
							korforgalom_meg_volt = 1;
						} else {
							set_compare_value_digit_szervo(J1_HARMADIK_SZERVO_SZOG);
						}
					}
					
					
					

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
		
			if(teszt_helyzet){
				BT_UART_SendString("K J 1 L V\r\n");
			}

			if(vil_ledek_szama() == 0){

			} else {
				kormany_szabalyzas_on = 1;
			}

			break;

		case KORFORG_JOBBRA_2:
			if(teszt_helyzet)
			{
				BT_UART_SendString("K J 2\r\n");
			}

			set_gyari_motor_compare_value(6520);

			if(kor_ford_most_kezd_merni)
			{
				kor_ford_most_kezd_merni = 0;
				set_compare_value_digit_szervo(J2_ELSO_SZERVO_SZOG);
				kormany_szabalyzas_on = 0;
				kor_ford_kezdet_encoder_ertek = get_encoder_counter();

			} else {

				kor_ford_mostani_encoder_ertek = get_encoder_counter();
				kor_ford_hossz = (kor_ford_kezdet_encoder_ertek - kor_ford_mostani_encoder_ertek )*ENCODER_VALUE_TO_MM;
				if(!kor_az_elso_iv_megvolt)
				{
					if(kor_ford_hossz >= J2_ELSO_IV_HOSSZ){
						kor_az_elso_iv_megvolt = 1;
					}
				} else {
					if(!kor_a_masodik_iv_megvolt){

						if(kor_ford_hossz >= J2_MASODIK_IV_HOSSZ){
							kor_a_masodik_iv_megvolt = 1;
						} else {
							set_compare_value_digit_szervo(J2_MASODIK_SZERVO_SZOG);
						}
					} else {
						if(kor_ford_hossz >= J2_HARMADIK_IV_HOSSZ){
							kormany_szabalyzas_on = 1;
							state_of_robot = JUST_GOING;
							korforgalom_meg_volt = 1;
						} else {
							set_compare_value_digit_szervo(J2_HARMADIK_SZERVO_SZOG);
						}
					}
				}

			}

			break;

		case KORFORG_JOBBRA_3:
			if(teszt_helyzet)
			{
				BT_UART_SendString("K J 3\r\n");
			}


			set_gyari_motor_compare_value(6540);
			if(kor_ford_most_kezd_merni)
			{
				kor_ford_most_kezd_merni = 0;
				set_compare_value_digit_szervo(J3_ELSO_SZERVO_SZOG);
				kormany_szabalyzas_on = 0;
				kor_ford_kezdet_encoder_ertek = get_encoder_counter();

			} else {

				kor_ford_mostani_encoder_ertek = get_encoder_counter();
				kor_ford_hossz = (kor_ford_kezdet_encoder_ertek - kor_ford_mostani_encoder_ertek )*ENCODER_VALUE_TO_MM;
				if(!kor_az_elso_iv_megvolt)
				{
					if(kor_ford_hossz >= J3_ELSO_IV_HOSSZ){
						kor_az_elso_iv_megvolt = 1;
					}
				} else {
					if(!kor_a_masodik_iv_megvolt){

						if(kor_ford_hossz >= J3_MASODIK_IV_HOSSZ){
							kor_a_masodik_iv_megvolt = 1;
						} else {
							set_compare_value_digit_szervo(J3_MASODIK_SZERVO_SZOG);
						}
					} else {
						if(kor_ford_hossz >= J3_HARMADIK_IV_HOSSZ){
							kormany_szabalyzas_on = 1;
							state_of_robot = JUST_GOING;
							korforgalom_meg_volt = 1;
						} else {
							set_compare_value_digit_szervo(J3_HARMADIK_SZERVO_SZOG);
						}
					}
				}

			}
			break;

		case KORFORG_BALRA_1:

			if(teszt_helyzet)
			{
				BT_UART_SendString("K B 1\r\n");
			}


			set_gyari_motor_compare_value(6520);





			if(b1_most_kezd_merni)
			{
				b1_most_kezd_merni = 0;
				set_compare_value_digit_szervo(B1_ELSO_SZERVO_SZOG);
				kormany_szabalyzas_on = 0;
				b1_kezdet_encoder_ertek = get_encoder_counter();

			} else {
				b1_mostani_encoder_ertek = get_encoder_counter();
				b1_hossz = (b1_kezdet_encoder_ertek - b1_mostani_encoder_ertek )*ENCODER_VALUE_TO_MM;
				if(!kor_az_elso_iv_megvolt)
				{
					if(b1_hossz >= B1_ELSO_IV_HOSSZ){
						kor_az_elso_iv_megvolt = 1;
					}
				} else {

					if(!kor_a_masodik_iv_megvolt){

						if(b1_hossz >= B1_MASODIK_IV_HOSSZ){
							kor_a_masodik_iv_megvolt = 1;
						} else {
							set_compare_value_digit_szervo(B1_MASODIK_SZERVO_SZOG);
						}
					} else {
						if(b1_hossz >= B1_HARMADIK_IV_HOSSZ){
							kormany_szabalyzas_on = 1;
							state_of_robot = JUST_GOING;
							korforgalom_meg_volt = 1;
						} else {
							set_compare_value_digit_szervo(B1_HARMADIK_SZERVO_SZOG);
						}
					}

				}
			}
			break;

		case KORFORG_BALRA_2:

			if(teszt_helyzet)
			{
				BT_UART_SendString("K B 2\r\n");
			}

			set_gyari_motor_compare_value(6510);

			if(kor_ford_most_kezd_merni)
			{
				kor_ford_most_kezd_merni = 0;
				set_compare_value_digit_szervo(B2_ELSO_SZERVO_SZOG);
				kormany_szabalyzas_on = 0;
				kor_ford_kezdet_encoder_ertek = get_encoder_counter();

			} else {

				kor_ford_mostani_encoder_ertek = get_encoder_counter();
				kor_ford_hossz = (kor_ford_kezdet_encoder_ertek - kor_ford_mostani_encoder_ertek )*ENCODER_VALUE_TO_MM;
				if(!kor_az_elso_iv_megvolt)
				{
					if(kor_ford_hossz >= B2_ELSO_IV_HOSSZ){
						kor_az_elso_iv_megvolt = 1;
					}
				} else {
					if(!kor_a_masodik_iv_megvolt){

						if(kor_ford_hossz >= B2_MASODIK_IV_HOSSZ){
							kor_a_masodik_iv_megvolt = 1;
						} else {
							set_compare_value_digit_szervo(B2_MASODIK_SZERVO_SZOG);
						}
					} else {
						if(kor_ford_hossz >= B2_HARMADIK_IV_HOSSZ){
							kormany_szabalyzas_on = 1;
							state_of_robot = JUST_GOING;
							korforgalom_meg_volt = 1;
						} else {
							set_compare_value_digit_szervo(B2_HARMADIK_SZERVO_SZOG);
						}
					}
				}

			}

			break;

		case KORFORG_BALRA_3:

			if(teszt_helyzet)
			{
				BT_UART_SendString("K B 3\r\n");
			}

			set_gyari_motor_compare_value(6540);

			if(kor_ford_most_kezd_merni)
			{
				kor_ford_most_kezd_merni = 0;
				set_compare_value_digit_szervo(B3_ELSO_SZERVO_SZOG);
				kormany_szabalyzas_on = 0;
				kor_ford_kezdet_encoder_ertek = get_encoder_counter();

			} else {

				kor_ford_mostani_encoder_ertek = get_encoder_counter();
				kor_ford_hossz = (kor_ford_kezdet_encoder_ertek - kor_ford_mostani_encoder_ertek )*ENCODER_VALUE_TO_MM;
				if(!kor_az_elso_iv_megvolt)
				{
					if(kor_ford_hossz >= B3_ELSO_IV_HOSSZ){
						kor_az_elso_iv_megvolt = 1;
					}
				} else {
					if(!kor_a_masodik_iv_megvolt){

						if(kor_ford_hossz >= B3_MASODIK_IV_HOSSZ){
							kor_a_masodik_iv_megvolt = 1;
						} else {
							set_compare_value_digit_szervo(B3_MASODIK_SZERVO_SZOG);
						}
					} else {
						if(kor_ford_hossz >= B3_HARMADIK_IV_HOSSZ){
							kormany_szabalyzas_on = 1;
							state_of_robot = JUST_GOING;
							korforgalom_meg_volt = 1;
						} else {
							set_compare_value_digit_szervo(B3_HARMADIK_SZERVO_SZOG);
						}
					}
				}

			}
			break;


/****************************************************/
			/* VASÚTI ÁTJÁRÓ ÁLLAPOTAI */
/****************************************************/

		case VASUTI_ATJARO_KOVETKEZIK:

			if(teszt_helyzet){
				BT_UART_SendString("VAS_ÁT jel\r\n");
			}

/*
			if(v_a_jelezs_kezdet)
			{
				v_a_jelzes_kezdet_encoder = get_encoder_counter();
				v_a_jelezs_kezdet = 0;
				set_gyari_motor_compare_value(5800);
			} else {
				v_a_jelzes_mostani_encoder = get_encoder_counter();
				v_a_jelzes_utan_megtett_ut = (v_a_jelzes_kezdet_encoder - v_a_jelzes_mostani_encoder)*ENCODER_VALUE_TO_MM;
			}
*/
/*
			if(!megalltunk){
				set_gyari_motor_compare_value(5800);
				if(speed_of_drogon < 0.01f)
				{
					megalltunk = 1;
				}
			} else {
				set_gyari_motor_compare_value(6510);
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
	*/

			if(vil_ledek_szama() == 0){



				if(!nulla_led_vilagitott_elobb){
					nulla_led_vilagitott_elobb = 1;
				} else {
					hanyszor_volt_nulla++;
				}

				if(hanyszor_volt_nulla >= 10){

					fekezni_kell = 1;

				}

			} else {
				nulla_led_vilagitott_elobb = 0;
				hanyszor_volt_nulla = 0;
			}


			if(fekezni_kell)
			{
				sebesseg_szabalyzas_elore_on = 0;
				if(fekez())
				{

					state_of_robot = VASUTI_ATJARO_KONVOJ_ELHALADASRA_VAR_ELOSZOR;
					fekezni_kell = 0;
				}
			}

			break;

		case VASUTI_ATJARO_KONVOJ_ELHALADASRA_VAR_ELOSZOR:
			if(teszt_helyzet)
			{
				BT_UART_SendString("1. VÁR\r\n");
			}
			itoa(elulso_sharp_szenzor, buf10, 10);
			BT_UART_SendString(buf10);
			BT_UART_SendString("\r\n");
			sebesseg_szabalyzas_elore_on = 0;
			konvoj_elhaladas_felismeres();
			break;

		case VASUTI_ATJARO_EGYSZER_ATHALADTUNK:
			if(teszt_helyzet)
			{
				BT_UART_SendString("1x ÁT\r\n");
			}


//			wanted_speed = 0.8f;

			sebesseg_szabalyzas_elore_on = 0;
			if(koztes_szakasz_kezdet)
			{
				set_gyari_motor_compare_value(6530);
				koztes_szakasz_kezdet = 0;
				koztes_szak_kezdet_encoder = get_encoder_counter();
			} else
			{



				koztes_szak_mostani_encoder = get_encoder_counter();
				koztes_szak_hossz = (koztes_szak_kezdet_encoder - koztes_szak_mostani_encoder)*ENCODER_VALUE_TO_MM;

				itoa(koztes_szak_hossz, buf10, 10);
				BT_UART_SendString(buf10);
				BT_UART_SendString("\r\n");
				if(koztes_szak_hossz >= 1500){

					fekezni_kell = 1;
/*
			set_gyari_motor_compare_value(5600);
					if(speed_of_drogon == 0.0f)
					{

						state_of_robot = VASUTI_ATJARO_KONVOJ_ELHALADASRA_VAR_MASODJARA;
					}
*/
				}



				/*
		if(!mar_egy_vonalon_voltunk)
		{
			if(vonalak_szama() == 1){
				if(!egy_vonal_volt_elobb){
					egy_vonal_volt_elobb = 1;
				} else {
					hanyszor_volt_egy_vonal++;
				}

				if(hanyszor_volt_egy_vonal >= 10){
					mar_egy_vonalon_voltunk = 1;
				}

			} else {
				egy_vonal_volt_elobb = 0;
				hanyszor_volt_egy_vonal = 0;
			}
		}
		else
		{
			if(vil_ledek_szama() == 0){



				if(!nulla_led_vilagitott_elobb){
					nulla_led_vilagitott_elobb = 1;
				} else {
					hanyszor_volt_nulla++;
				}

				if(hanyszor_volt_nulla >= 10){

					fekezni_kell = 1;

				}

			} else {
				nulla_led_vilagitott_elobb = 0;
				hanyszor_volt_nulla = 0;
			}
		}
*/






				if(fekezni_kell)
				{
					if(fekez())
					{

						state_of_robot = VASUTI_ATJARO_KONVOJ_ELHALADASRA_VAR_MASODJARA;
						fekezni_kell = 0;
					}
				}


			}
			break;

		case VASUTI_ATJARO_KONVOJ_ELHALADASRA_VAR_MASODJARA:

			if(teszt_helyzet)
			{
				BT_UART_SendString("2. VÁR\r\n");
			}

//			set_gyari_motor_compare_value(5600);


			itoa((int)autot_erzekeltem, buf10, 10);
			BT_UART_SendString(buf10);
			BT_UART_SendString("\r\n");

			itoa((int)varakozasi_ido, buf10, 10);
			BT_UART_SendString(buf10);
			BT_UART_SendString("\r\n");

			konvoj_elhaladas_felismeres();
			break;

		case VASUTI_ATJARO_KETSZER_ATHALADTUNK:

			if(teszt_helyzet)
			{
				BT_UART_SendString("2x ÁT\r\n");
			}


			set_gyari_motor_compare_value(6510);

//			dupla_keresztvonal_felismeres();

//			kereszt_vonal_felismeres(vil_ledek_szama());

//			state_of_robot = JUST_GOING;
/*
			if(v_a_masodiik_dupla_kereszt_is_meg_volt){
				set_gyari_motor_compare_value(5800);
				while(1);
			}
			*/
			break;


/****************************************************/
			/* HORDÓ ÁLLAPOTAI */
/****************************************************/


		case HORDO_KOVETKEZIK:


			if(teszt_helyzet)
			{
				BT_UART_SendString("HORDÓ jel\r\n");
			}


			if(hordo_jel_utan_elso_meres)
			{
				hordo_jel_utan_elso_meres = 0;
				hordo_jel_utan_tav_kezdet_encoder_ertek = get_encoder_counter();
			}
			else {
				hordo_jel_utan_tav_mostani_encoder_ertek = get_encoder_counter();
				hordo_jel_utan_tav_hossz = (hordo_jel_utan_tav_kezdet_encoder_ertek - hordo_jel_utan_tav_mostani_encoder_ertek)*ENCODER_VALUE_TO_MM;
			}

			if(hordo_jel_utan_tav_hossz >= 200)
			{

				state_of_robot = HORDO_KORM_SZAB_KI;
			}

			itoa((int)hordo_jel_utan_tav_hossz, buf10, 10);
			BT_UART_SendString(buf10);
			BT_UART_SendString("\r\n");


			set_gyari_motor_compare_value(6600);
			sebesseg_szabalyzas_elore_on = 0;
			break;


		case HORDO_KORM_SZAB_KI:


			if(teszt_helyzet)
			{
				BT_UART_SendString("HORDÓ KORM KI\r\n");
			}

			hordo_jel_utan_tav_mostani_encoder_ertek = get_encoder_counter();
			hordo_jel_utan_tav_hossz = (hordo_jel_utan_tav_kezdet_encoder_ertek - hordo_jel_utan_tav_mostani_encoder_ertek)*ENCODER_VALUE_TO_MM;
			itoa((int)hordo_jel_utan_tav_hossz, buf10, 10);
			BT_UART_SendString(buf10);
			BT_UART_SendString("\r\n");
			if(fekez())
			{
				BT_UART_SendString("FEK_OKE\r\n");
//				set_gyari_motor_compare_value(6200);
			}
//			set_gyari_motor_compare_value(5800);
/*
			kormany_szabalyzas_on = 0;
			LMS6DS3_Read_Axes_with_correction(&x_acc, &y_acc, &z_acc);

			szervo_value = DIGIT_SZ_KOZEP + (uint16_t)(x_acc*30);
			set_compare_value_digit_szervo(szervo_value);
*/
			break;


/****************************************************/
			/* CÉL ÁLLAPOTAI */
/****************************************************/


		case GYOZELEM:

			if(teszt_helyzet)
			{
				BT_UART_SendString("GYOZELEM");
			}
			set_gyari_motor_compare_value(5800);

			while(1);

			break;


		case JUST_GOING:

			wanted_speed = 0.6f;
			if(teszt_helyzet){
				BT_UART_SendString("CSILL\r\n");
			}

			fal_felismeres();

//			dupla_keresztvonal_felismeres();
			kereszt_vonal_felismeres(vil_ledek_szama());
			jelzes_felismeres(vonalak_szama());

/*TODO: teszt
			BT_UART_SendString("J: ");
			itoa((int)(jobb_oldali_sharp_szenzor), buf10, 10);
			BT_UART_SendString(buf10);
			BT_UART_SendString("   ");

			BT_UART_SendString("B: ");
			itoa((int)(bal_oldali_sharp_szenzor), buf10, 10);
			BT_UART_SendString(buf10);

			BT_UART_SendString("\r\n");
*/
			break;

		default:

			break;
		}




/************************************************************************/

		if(!meg_jott_a_start_kapu_jele){
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET)
			{
				meg_jott_a_start_kapu_jele = 0;
			} else {
				meg_jott_a_start_kapu_jele = 1;
				start_kapu_fala_meg_volt = 1;

			}

		} else {

			if(sebesseg_szabalyzas_elore_on)
			{
				sebesseg_szabalyzas();
			}

		}




/*******************************************/
//Ciklus újra indítása csak a timer lejárta után történhet meg. Ezért kell nullázni,
//hogy ha a ciklus véget is ért, a main-ben ne hívódjon meg újra, míg a timer owerflowja
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
/* Fal felismeri segédváltozók */
uint8_t meg_nem_volt_utca_sarok_elso_fal = 1;
/***************************/



/***************************/
//Jelzes felismerés segédváltozók

//3 vonalhoz -----> Drón elõtt
uint8_t hanyszor_volt_3_vonal = 0;
uint8_t harmas_vonal_van = 0;
uint8_t elobb_3_vonal_volt = 0;
int32_t harmas_vonal_kezdete_encoder_ertek = 0;
int32_t harmas_vonal_vege_encoder_ertek = 0;
int32_t harmas_vonal_mostani_encoder_ertek = 0;
int32_t harmas_vonal_hossza = 0;

//0 vonalhoz ----> Körforgalom elõtt
uint8_t hanyszor_volt_0_vonal = 0;
uint8_t nullas_vonal_van = 0;
uint8_t elobb_0_vonal_volt = 0;
int32_t nullas_vonal_kezdete_encoder_ertek = 0;
int32_t nullas_vonal_vege_encoder_ertek = 0;
int32_t nullas_vonal_mostani_encoder_ertek = 0;
int32_t nullas_vonal_hossza = 0;

//2 vonalhoz ----> Hordó elõtt
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
/****************** DRÓN ************************/
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

			if(!mar_meg_volt_a_drone)
			{
				state_of_robot = DRONE_KOVETKEZIK;
			}

		}
		if(vonal_szam == 1)
		{
			harmas_vonal_van = 0;
			elobb_3_vonal_volt = 0;
			hanyszor_volt_3_vonal = 0;

			if(harmas_vonal_hossza < 100){

			}
			harmas_vonal_hossza = 0;
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
/****************** KÖRFORGALOM ************************/

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

/*
				if(state_of_robot != KORFORGALOM_KOVETKEZIK || state_of_robot != VASUTI_ATJARO_KETSZER_ATHALADTUNK)
				{
					state_of_robot = KORFORGALOM_KOVETKEZIK;
					korforgalom_jelzes_utani_elso_encoder_ertek = get_encoder_counter();
				}
*/

				if(state_of_robot == JUST_GOING){


					if(!korforgalom_meg_volt)
					{
						state_of_robot = KORFORGALOM_KOVETKEZIK;
						korforgalom_jelzes_utani_elso_encoder_ertek = get_encoder_counter();
						itoa((int)korforgalom_jelzes_utani_elso_encoder_ertek, buf10, 10);
						BT_UART_SendString(buf10);
						BT_UART_SendString("\r\n");
					}

				}
			}
			nullas_vonal_hossza = 0;
		}
	}




/***************************************************************/
/****************** HORDÓ ************************/


	if(vonal_szam == 2)
	{
		if(elobb_2_vonal_volt)
		{
			hanyszor_volt_2_vonal++;
		}

		if(hanyszor_volt_2_vonal >= 4)
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

/* Konvoj elhaladás felismerés segédváltozók  egyszerûbb változat*/
uint8_t auto_van = 0;
uint8_t szunet_van = 0;
uint8_t szunet_szam = 0;
uint8_t auto_szam = 0;
uint8_t uj_autot_erzekeltem = 0;
/**************************************************************/


void konvoj_elhaladas_felismeres()
{

	if(!autot_erzekeltem){
		if(elulso_sharp_szenzor <= 450){
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

				uj_autot_erzekeltem = 1;
				varakozasi_ido = 0;
			}
		}
	}

	if(state_of_robot == VASUTI_ATJARO_KONVOJ_ELHALADASRA_VAR_ELOSZOR){
		if(varakozasi_ido >= 1000){
			state_of_robot = VASUTI_ATJARO_EGYSZER_ATHALADTUNK;
			autot_erzekeltem = 0;
			varakozasi_ido = 0;
			auto_van = 0;
			szunet_van = 0;
			szunet_szam = 0;
			auto_szam = 0;
		}
	} else if(state_of_robot == VASUTI_ATJARO_KONVOJ_ELHALADASRA_VAR_MASODJARA){
		if(varakozasi_ido >= 1000){
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


/****************************************************/
/* Konvoj elhaladas körbe felismerése segédváltozók */
uint8_t hanyszor_erzekeltem_autot = 0;
uint8_t elobb_autot_erzekeltem = 0;

/****************************************************/


void konvoj_elhaladas_felismeres_korbe()
{
	if(!autot_erzekeltem){

		if(elulso_sharp_szenzor <= 1000){

			if(!elobb_autot_erzekeltem){
				elobb_autot_erzekeltem = 1;
			} else
			{
				hanyszor_erzekeltem_autot++;

				if(hanyszor_erzekeltem_autot >= 3)
				{
					autot_erzekeltem = 1;
					auto_van = 1;
					szunet_van = 0;
					auto_szam++;
				}
			}

		} else {
			elobb_autot_erzekeltem = 0;
			hanyszor_erzekeltem_autot = 0;
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

				uj_autot_erzekeltem = 1;
				varakozasi_ido = 0;
			}
		}
	}

	if(state_of_robot == KONVOJ_JELZES_JOBB_RAALLAS){
		if(varakozasi_ido >= 800){
			state_of_robot = KONVOJ_JELZES_JOBB_KORRE_MENET;
			autot_erzekeltem = 0;
			varakozasi_ido = 0;
			auto_van = 0;
			szunet_van = 0;
			szunet_szam = 0;
			auto_szam = 0;
		}
	} else if(state_of_robot == KONVOJ_JELZES_BAL_RAALLAS){
		if(varakozasi_ido >= 800){
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

/**************************************************************/

/* Keresztvonal felismerés segédváltozók  */

uint8_t hanyszor_volt_kereszt_vonal = 0;
uint8_t kereszt_vonal_van = 0;
uint8_t elobb_kereszt_vonal_volt = 0;
int32_t kereszt_vonal_utani_egyes_kezdete_encoder_ertek = 0;
int32_t kereszt_vonal_utani_egyes_vege_encoder_ertek = 0;
int32_t kereszt_vonal_utani_egyes_mostani_encoder_ertek = 0;
int32_t kereszt_vonal_utani_egyes_hossza = 0;
uint8_t egyes_vonal_volt_a_kereszt_utan = 0;


uint8_t masodik_athaladas_utani_kereszt_meg_volt = 0;

/**************************************************************/


void kereszt_vonal_felismeres(uint8_t ledek_szama)
{

	if(state_of_robot == VASUTI_ATJARO_KETSZER_ATHALADTUNK)
	{
		BT_UART_SendString("ITT VAGYOK \r\n");
	}

	if(ledek_szama >= 30)
		{
			if(egyes_vonal_volt_a_kereszt_utan)
			{
				BT_UART_SendString("Ker\r\n");
				if(kereszt_vonal_utani_egyes_hossza <= 50){

					BT_UART_SendString("Rek\r\n");
					if(v_a_elso_dupla_kereszt_mar_meg_volt)
					{
						BT_UART_SendString("2. D K\r\n");


						state_of_robot = JUST_GOING;
					} else
					{
						state_of_robot = VASUTI_ATJARO_KOVETKEZIK;
						hanyszor_volt_kereszt_vonal = 0;
						kereszt_vonal_van = 0;
						elobb_kereszt_vonal_volt = 0;
						kereszt_vonal_utani_egyes_kezdete_encoder_ertek = 0;
						kereszt_vonal_utani_egyes_vege_encoder_ertek = 0;
						kereszt_vonal_utani_egyes_mostani_encoder_ertek = 0;
						kereszt_vonal_utani_egyes_hossza = 0;
						egyes_vonal_volt_a_kereszt_utan = 0;
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
			/* Egy módja a vasúti átjáró elõtti megállásnak, hogy
			 * A keresztvonal érzékelése után kikapcsolom a motort
			 */
/**********************************************************************/
			if(state_of_robot == VASUTI_ATJARO_KOVETKEZIK){
				meg_volt_a_kereszt_vonal = 1;
			}

			if(state_of_robot == VASUTI_ATJARO_KETSZER_ATHALADTUNK)
			{


				if(!masodik_athaladas_utani_kereszt_meg_volt)
				{
					masodik_athaladas_utani_kereszt_meg_volt = 1;
					BT_UART_SendString("2 Á U K\r\n");
					hanyszor_volt_kereszt_vonal = 0;
					kereszt_vonal_van = 0;
					elobb_kereszt_vonal_volt = 0;
					kereszt_vonal_utani_egyes_kezdete_encoder_ertek = 0;
					kereszt_vonal_utani_egyes_vege_encoder_ertek = 0;
					kereszt_vonal_utani_egyes_mostani_encoder_ertek = 0;
					kereszt_vonal_utani_egyes_hossza = 0;
					egyes_vonal_volt_a_kereszt_utan = 0;
				}
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
						if(state_of_robot == JUST_GOING){
							state_of_robot = GYOZELEM;
						}

					}
				}

			}
		}
}



/**************************************************************/
/* Dupla keresztvonal felismerés segédváltozók  */
uint8_t elso_kereszt_vonal = 0;
uint8_t masodik_kereszt_vonal = 0;
uint8_t dk_k_v_volt = 0;
uint8_t dk_k_v_hanyszor = 0;
uint8_t most_kereszt_vonalon_vagyok = 0;
int32_t elso_kereszt_vonal_encoder_ertek = 0;
int32_t dk_jelenlegi_encoder_ertek = 0;
int32_t elso_kereszt_vonaltol_mert_tavolsag = 0;


void dupla_keresztvonal_felismeres()
{


	if(!most_kereszt_vonalon_vagyok)
	{
		if(vil_ledek_szama() >= 30)
			{
				if(!dk_k_v_volt)
				{
					dk_k_v_volt = 1;
				} else
				{
					dk_k_v_hanyszor++;

					if(dk_k_v_hanyszor >= 1)
					{
						most_kereszt_vonalon_vagyok = 1;
						if(!elso_kereszt_vonal)
						{
							elso_kereszt_vonal = 1;
							elso_kereszt_vonal_encoder_ertek = get_encoder_counter();
						} else
						{
							masodik_kereszt_vonal = 1;

							if(state_of_robot == JUST_GOING)
							{
								state_of_robot = VASUTI_ATJARO_KOVETKEZIK;
								dk_k_v_volt = 0;
								dk_k_v_hanyszor = 0;
								elso_kereszt_vonal = 0;

							} else if(state_of_robot == VASUTI_ATJARO_KETSZER_ATHALADTUNK)
							{
								state_of_robot = JUST_GOING;
								dk_k_v_volt = 0;
								dk_k_v_hanyszor = 0;
								elso_kereszt_vonal = 0;
							}
						}
					}
				}
			} else
			{
				dk_k_v_volt = 0;
				dk_k_v_hanyszor = 0;
			}
	}
	else {
		if(vil_ledek_szama() <= 10)
		{
			most_kereszt_vonalon_vagyok = 0;
		}
	}



	if(elso_kereszt_vonal)
	{
		dk_jelenlegi_encoder_ertek = get_encoder_counter();
		elso_kereszt_vonaltol_mert_tavolsag = (elso_kereszt_vonal_encoder_ertek - dk_jelenlegi_encoder_ertek)*ENCODER_VALUE_TO_MM;
		if(elso_kereszt_vonaltol_mert_tavolsag >= 100)
		{
			if(state_of_robot == JUST_GOING)
			{
				state_of_robot = GYOZELEM;
			}
			dk_k_v_volt = 0;
			dk_k_v_hanyszor = 0;
			elso_kereszt_vonal = 0;
		}
	}



}



/**************************************************************/

/* fal felismerés segédváltozók  */
uint8_t jobb_oldalon_fal_van = 0;
int32_t jobb_oldali_fal_kezdete_encoder_ertek;
int32_t jobb_oldali_fal_mostani_encoder_ertek;
int32_t jobb_oldali_fal_hossza = 0;

uint8_t bal_oldalon_fal_van = 0;
int32_t bal_oldali_fal_kezdete_encoder_ertek;
int32_t bal_oldali_fal_mostani_encoder_ertek;
int32_t bal_oldali_fal_hossza = 0;

uint8_t hanyszor_lattam_dupla_falat = 0;
uint8_t elobb_dupla_fal_volt = 0;

uint8_t hanyszor_lattam_jobb_falat = 0;
uint8_t elobb_jobb_fal_volt = 0;

uint8_t hanyszor_lattam_bal_falat = 0;
uint8_t elobb_bal_fal_volt = 0;


uint8_t utca_sarok_utani_dupla_fal_van = 0;





/**************************************************************/

void fal_felismeres(){

//Lekezelve: JUST_GOING állapotban futhat le
	if(meg_nem_volt_utca_sarok_elso_fal){
		if(jobb_oldali_sharp_szenzor <= 250 && bal_oldali_sharp_szenzor <= 250)
		{

			if(!elobb_dupla_fal_volt)
			{
				elobb_dupla_fal_volt = 1;
			} else {
				hanyszor_lattam_dupla_falat++;
			}

			if(hanyszor_lattam_dupla_falat >= 4)
			{
				state_of_robot = UTCA_SAROK_DUPLA_FAL;

				elobb_dupla_fal_volt = 0;
				hanyszor_lattam_dupla_falat = 0;
				meg_nem_volt_utca_sarok_elso_fal = 0;
			}


		}
		else {
			elobb_dupla_fal_volt = 0;
			hanyszor_lattam_dupla_falat = 0;
		}
	}

//Lekezelve: Egyértelmû mikor futhat
	if(state_of_robot == UTCA_SAROK_DUPLA_FAL)
	{

		if(jobb_oldali_sharp_szenzor >= 300 && bal_oldali_sharp_szenzor >= 300){

			if(start_kapu_fala_meg_volt){
				state_of_robot = UTCA_SAROK_DUPLA_FAL_UTANI_SZUNET;
			} else {
				start_kapu_fala_meg_volt = 1;
				state_of_robot = JUST_GOING;
				meg_nem_volt_utca_sarok_elso_fal = 1;
			}
		}
	}

//Lekezelve: Egyértelmû mikor futhat
	if(state_of_robot == UTCA_SAROK_DUPLA_FAL_UTANI_SZUNET)
	{
		if(jobb_oldali_sharp_szenzor <= 250 && bal_oldali_sharp_szenzor >= 300){
			state_of_robot = UTCA_SAROK_MASODIK_FAL_JOBB;
		}

		if(jobb_oldali_sharp_szenzor >= 300 && bal_oldali_sharp_szenzor <= 250){
			state_of_robot = UTCA_SAROK_MASODIK_FAL_BAL;
		}
	}

//Lekezelve: Egyértelmû mikor futhat
	if(state_of_robot == JUST_GOING && state_of_robot != UTCA_SAROK_BAL_TOLATAS_VEGE && state_of_robot != UTCA_SAROK_JOBB_TOLATAS_VEGE)
	{
		if(jobb_oldali_sharp_szenzor <= 200 && bal_oldali_sharp_szenzor >= 300){

			if(!jobb_oldalon_fal_van)
			{
				if(!elobb_jobb_fal_volt)
				{
					elobb_jobb_fal_volt = 1;
				} else {
					hanyszor_lattam_jobb_falat++;
				}

				if(hanyszor_lattam_jobb_falat >= 4)
				{
					jobb_oldalon_fal_van = 1;
					jobb_oldali_fal_kezdete_encoder_ertek = get_encoder_counter();
					elobb_jobb_fal_volt = 0;
				}

			} else {
				jobb_oldali_fal_mostani_encoder_ertek = get_encoder_counter();
				jobb_oldali_fal_hossza = (jobb_oldali_fal_kezdete_encoder_ertek - jobb_oldali_fal_mostani_encoder_ertek)*ENCODER_VALUE_TO_MM;

				itoa((int)jobb_oldali_fal_hossza, buf10, 10);
				BT_UART_SendString(buf10);
				if(jobb_oldali_fal_hossza >= 50){
					state_of_robot = KONVOJ_KOVETKEZIK_JELZES_A_JOBB_OLDALON;
				}
			}
		} else
		{
			elobb_jobb_fal_volt = 0;
			hanyszor_lattam_jobb_falat = 0;
			jobb_oldalon_fal_van = 0;
		}

		if(jobb_oldali_sharp_szenzor >= 300 && bal_oldali_sharp_szenzor <= 200){


			if(!bal_oldalon_fal_van)
			{

				if(!elobb_bal_fal_volt)
				{
					elobb_bal_fal_volt = 1;
				} else {
					hanyszor_lattam_bal_falat++;
				}

				if(hanyszor_lattam_bal_falat >= 4)
				{
					bal_oldalon_fal_van = 1;
					elobb_bal_fal_volt = 0;
					bal_oldali_fal_kezdete_encoder_ertek = get_encoder_counter();

				}
			} else {
				bal_oldali_fal_mostani_encoder_ertek = get_encoder_counter();
				bal_oldali_fal_hossza = (bal_oldali_fal_kezdete_encoder_ertek - bal_oldali_fal_mostani_encoder_ertek)*ENCODER_VALUE_TO_MM;

				if(bal_oldali_fal_hossza >= 50){
					state_of_robot = KONVOJ_KOVETKEZIK_JELZES_A_BAL_OLDALON;
				}
			}
		} else {
			elobb_bal_fal_volt = 0;
			hanyszor_lattam_bal_falat = 0;
			bal_oldalon_fal_van = 0;
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
			BT_UART_SendString("N M K\r\n");
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

uint8_t sebesseg_tarto_counter_konvoj = 0;
float integrator_ertek_konvoj = 0;
float speed_diff_konvoj  = 0;



void sebesseg_szabalyzas_konvojhoz(){
	if(sebesseg_tarto_counter_konvoj > 5){
		speed_diff_konvoj = wanted_speed_konvoj - speed_of_drogon;

		integrator_ertek_konvoj += speed_diff_konvoj;


/*
		BT_UART_SendString("I:  ");
		itoa((int)(integrator_ertek_konvoj*10), buf10, 10);
		BT_UART_SendString(buf10);
		BT_UART_SendString("\r\n");

		BT_UART_SendString("SD: ");
		itoa((int)(speed_diff_konvoj*1000), buf10, 10);
		BT_UART_SendString(buf10);
		BT_UART_SendString("\r\n");
*/

		motor_value = 6480 + (int)(speed_diff_konvoj*KP_speed_konvoj) + (int)(KI_speed_konvoj*integrator_ertek_konvoj);

		if(motor_value > GYARI_MOTOR_COUNTER_MAX){
			motor_value = GYARI_MOTOR_COUNTER_MAX;
		} else if(motor_value < GYARI_MOTOR_COUNTER_KOZEP){
			motor_value = GYARI_MOTOR_COUNTER_KOZEP;
		}
		set_gyari_motor_compare_value(motor_value);

	sebesseg_tarto_counter_konvoj = 0;
	} else {
	sebesseg_tarto_counter_konvoj++;
	}
}

void konvoj_kovetes_sebesseg_allitas()
{
	if(p_elso <= 9){
		wanted_speed_konvoj = elulso_sharp_szenzor*0.00053f;
		if(elulso_sharp_szenzor <= 300){
			wanted_speed = 0.0f;
		} else{
			if(wanted_speed_konvoj < 0.7f)
			{
				wanted_speed_konvoj = 0.7f;
			}
		}

/*
		KP_kormany = KP_fast;
		KD_kormany = KD_fast;
*/
	} else {
		wanted_speed_konvoj = 0.9f;

/*
		KP_kormany = 1200;
		KD_kormany = 900;
*/

	}



	BT_UART_SendString("S: ");
	itoa(elulso_sharp_szenzor, buf10, 10);
	BT_UART_SendString(buf10);
	BT_UART_SendString("\r\n");
}

void utca_sarok_jobb_elofeldolgozas(){
	uint8_t csucs_kezdete = 0;
	uint8_t cnt = 0;
	uint8_t m;
	uint8_t egy_gyucsot_talaltam = 0;
	for(m = 0; m < 32; m++){

		if(!egy_gyucsot_talaltam)
		{
			if(!csucs_kezdete){
				if(adcAdatok_elso[m] != 0){
					csucs_kezdete = 1;

				}
			} else {
				if(adcAdatok_elso[m] == 0){
					egy_gyucsot_talaltam = 1;
				}
			}
		} else
		{
			adcAdatok_elso[m] = 0;
		}

	}
}


void utca_sarok_bal_elofeldolgozas(){
	uint8_t csucs_kezdete = 0;
	uint8_t cnt = 0;
	uint8_t m;
	uint8_t egy_gyucsot_talaltam = 0;
	for(m = 32; m > 0; m--){

		if(!egy_gyucsot_talaltam)
		{
			if(!csucs_kezdete){
				if(adcAdatok_elso[m-1] != 0){
					csucs_kezdete = 1;

				}
			} else {
				if(adcAdatok_elso[m-1] == 0){
					egy_gyucsot_talaltam = 1;
				}
			}
		} else
		{
			adcAdatok_elso[m-1] = 0;
		}

	}
}


/*******************************************************/
/* Fékezés segédváltozói  */
uint8_t lefekeztunk = 0;
/*******************************************************/


uint8_t fekez()
{

	if(!fek_megvartuk_a_kozepet)
	{

		if(!fek_varjuk_meg_a_kozep_erteket){
			if(!fek_varjuk_meg_a_hatra_erteket){
				fek_varjuk_meg_a_hatra_erteket = 1;
				set_gyari_motor_compare_value(5800);
			} else {

				if(fek_hatra_ido_milisec >= 50){
					fek_varjuk_meg_a_kozep_erteket = 1;
					set_gyari_motor_compare_value(6200);
					fek_megvartuk_a_hatrat = 1;
				}
			}
		} else {
			if(fek_kozep_ido_milisec >= 50){
				fek_megvartuk_a_kozepet = 1;
				set_gyari_motor_compare_value(4800);
			}
		}
	} else {


		if(speed_of_drogon <= 0.0f)
		{
			set_gyari_motor_compare_value(6200);
			lefekeztunk = 1;
		} else
		{
			set_gyari_motor_compare_value(4800);
		}

	}


	return lefekeztunk;
}


