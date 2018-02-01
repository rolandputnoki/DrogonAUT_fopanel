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

//SPI fogadási flag
uint8_t data_received = 1;

uint16_t KP_kormany = 0;
uint16_t KD_kormany = 0;


//Szabályozás

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

uint8_t sebesseg_tarto_counter = 0;

float speed_diff = 0;

int main()
{

	KP_kormany = KP_slow;
	KD_kormany = KD_slow;

	// HAL_Init, System_Clock_config és hardware inicializáció
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



char bt_adc_value[10];





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


//Orientáció
float delta_orient = 0;





//Hosszú 3as vonal és szaggatott 3as vonal felismerés

//mm mértékegységben
uint16_t kivant_tavolsag = 200;



uint32_t meres_kezdeti_encoder_ertek = 0;
uint32_t meres_mostani_encoder_ertek = 0;
uint32_t meres_megtett_tavolsag = 0;
uint32_t meresen_beluli_sorszam = 0;
uint16_t ket_meres_kozotti_tavolsagok_sorrendben[100];
uint8_t meresek_3as_vonal_szama_sorrendben[100];
uint16_t meres_harom_vonal_szamlalo = 0;
uint8_t eloszor_egyes_aztan_vegig_harmas = 0;
char buf10[10];

uint8_t vil_led = 0;

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

		pd_value = KP_kormany*p_elso + KD_kormany*D;
		prev_pos = p_elso;


		szervo_value = DIGIT_SZ_KOZEP + (int16_t)pd_value;


		set_compare_value_digit_szervo(szervo_value);
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

/****************************************************/
			/* DRÓN ÁLLAPOTAI */
/****************************************************/
		case DRONE_KOVETKEZIK:
			BT_UART_SendString("DRONE köv\r\n");
			wanted_speed = (elulso_sharp_szenzor - 450)*0.00057f;
			if(!speed_of_drogon || elulso_sharp_szenzor <= 450){
					wanted_speed = 0.0f;
					state_of_robot = DRONE_ELOTT_ALLUNK;


			}
			break;
		case DRONE_ELOTT_ALLUNK:
			BT_UART_SendString("DRONE all\r\n");

			itoa(elulso_sharp_szenzor, buf10, 10);
			BT_UART_SendString(buf10);
			BT_UART_SendString("\r\n");
			if(elulso_sharp_szenzor >= 1200){
				state_of_robot = DRONE_FELSZALLT;
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
			/* KÖRFORGALOM ÁLLAPOTAI */
/****************************************************/

		case UTCA_SAROK_DUPLA_FAL:
			BT_UART_SendString("DUPLA FAL");
			BT_UART_SendString("\r\n");

			break;

		case UTCA_SAROK_DUPLA_FAL_UTANI_SZUNET:
			BT_UART_SendString("DUPLA FAL U SZ");
			BT_UART_SendString("\r\n");

			break;

		case UTCA_SAROK_MASODIK_FAL_JOBB:
			BT_UART_SendString("2. J");
			BT_UART_SendString("\r\n");
			set_gyari_motor_compare_value(5800);
			while(1){

			}
			break;

		case UTCA_SAROK_MASODIK_FAL_BAL:
			BT_UART_SendString("2. B");
			BT_UART_SendString("\r\n");
			set_gyari_motor_compare_value(5800);
			while(1){

			}
			break;

		case UTCA_SAROK_MASODIK_FAL_UTANI_SZUNET:
			BT_UART_SendString("2. SZÜN");
			BT_UART_SendString("\r\n");
			break;


		case HORDO_KOVETKEZIK:
			BT_UART_SendString("HORDÓ\r\n");
			set_gyari_motor_compare_value(6200);
			while(1){

			}
			break;

		case KONVOJ_KOVETKEZIK_JELZES_A_JOBB_OLDALON:
			BT_UART_SendString("KONV J\r\n");
			set_gyari_motor_compare_value(6200);
			while(1){

			}
			break;

		case KONVOJ_KOVETKEZIK_JELZES_A_BAL_OLDALON:
			BT_UART_SendString("KONV B\r\n");
			set_gyari_motor_compare_value(6200);
			while(1){

			}

			break;

/****************************************************/
			/* KÖRFORGALOM ÁLLAPOTAI */
/****************************************************/
		case KORFORGALOM_KOVETKEZIK:
			BT_UART_SendString("KÖRFORGALOM\r\n");
			set_gyari_motor_compare_value(5800);
/*
			while(1){

			}
*/
			break;

		case KORFORG_JOBBRA_1:
			set_gyari_motor_compare_value(5800);
			BT_UART_SendString("K J 1\r\n");
			break;

		case KORFORG_JOBBRA_2:
			set_gyari_motor_compare_value(5800);
			BT_UART_SendString("K J 2\r\n");
			break;

		case KORFORG_JOBBRA_3:
			set_gyari_motor_compare_value(5800);
			BT_UART_SendString("K J 3\r\n");
			break;

		case KORFORG_BALRA_1:
			set_gyari_motor_compare_value(5800);
			BT_UART_SendString("K B 1\r\n");
			break;

		case KORFORG_BALRA_2:
			set_gyari_motor_compare_value(5800);
			BT_UART_SendString("K B 2\r\n");
			break;

		case KORFORG_BALRA_3:
			set_gyari_motor_compare_value(5800);
			BT_UART_SendString("K B 3\r\n");
			break;



		case VASUTI_ATJARO_KOVETKEZIK:
			set_gyari_motor_compare_value(5800);
			BT_UART_SendString("HUHUHU");
			while(1);
			break;

		case GYOZELEM:
			set_gyari_motor_compare_value(5800);
			BT_UART_SendString("GYOZELEM");
			while(1);

			break;

		default:

			break;
		}


/************************************************************************/

		if(!meg_jott_a_start_kapu_jele){

		} else {
			sebesseg_szabalyzas();
		}

		fal_felismeres();
		kereszt_vonal_felismeres(vil_ledek_szama());
		jelzes_felismeres(vonalak_szama());
		korforgalom_jelzes_felismeres();
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

/***************************************************************/
/****************** KÖRFORGALOM ************************/

	if(vonal_szam == 0)
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
				state_of_robot = KORFORGALOM_KOVETKEZIK;
			}
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

/* Keresztvonal felismerés segédváltozók  */

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
					state_of_robot = VASUTI_ATJARO_KOVETKEZIK;
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
						state_of_robot = GYOZELEM;
					}
				}

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

	if(state_of_robot == UTCA_SAROK_MASODIK_FAL_JOBB){

		if(jobb_oldali_sharp_szenzor >= 300 && bal_oldali_sharp_szenzor >= 300){
			state_of_robot = UTCA_SAROK_MASODIK_FAL_UTANI_SZUNET;
		}
	}

	if(state_of_robot != UTCA_SAROK_DUPLA_FAL_UTANI_SZUNET && state_of_robot != UTCA_SAROK_DUPLA_FAL && state_of_robot != UTCA_SAROK_MASODIK_FAL_JOBB && state_of_robot != UTCA_SAROK_MASODIK_FAL_BAL){
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

uint8_t csucsok_szama(){
	uint8_t csucs_kezdete = 0;
	uint8_t cnt = 0;
	uint16_t m;
	uint8_t kezdes = 1;



	for(m = 0; m < ONE_CYCLE_MEASERES; m++){

		if(kezdes){
			if(vonalak_szama_sorrendben[m] == 1){
				kezdes = 0;
			}
		}

		else {
			if(!csucs_kezdete){
				if(vonalak_szama_sorrendben[m] == 3){
					csucs_kezdete = 1;
				}
			} else {
				if(vonalak_szama_sorrendben[m] == 1){
					csucs_kezdete = 0;
					cnt++;
				}
			}
		}


	}
/*
	if(cnt == 0 && csucs_kezdete == 1){
		return 10;
	} else {

	*/
		return cnt;

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

