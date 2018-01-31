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


			BT_UART_SendString("J:  ");
			itoa((int)(jobb_oldali_sharp_szenzor), buffer, 10);
			BT_UART_SendString(buffer);
			BT_UART_SendString("\r\n");

			BT_UART_SendString("B:");
			itoa((int)(bal_oldali_sharp_szenzor), buffer, 10);
			BT_UART_SendString(buffer);
			BT_UART_SendString("\r\n");


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

/************************************************************************/
/************************************************************************/

		switch(state_of_robot){


		case DRONE_KOVETKEZIK:

			wanted_speed = (elulso_sharp_szenzor - 450)*0.00057f;
			if(!speed_of_drogon || elulso_sharp_szenzor <= 450){
					wanted_speed = 0.0f;
					state_of_robot = DRONE_ELOTT_ALLUNK;


			}
			break;
		case DRONE_ELOTT_ALLUNK:


			itoa(elulso_sharp_szenzor, buf10, 10);
			BT_UART_SendString(buf10);
			BT_UART_SendString("\r\n");
			if(elulso_sharp_szenzor >= 1200){
				state_of_robot = DRONE_FELSZALLT;
			}
			break;
		case DRONE_FELSZALLT:

			start_milisec_szamlalo = 1;
			if(milisec_szamlalo > 2100){
				wanted_speed = 0.9f;
				state_of_robot = JUST_GOING;
			}
			break;
		case JUST_GOING:

			break;

		default:

			break;
		}


/************************************************************************/

		if(!meg_jott_a_start_kapu_jele){

		} else {
			sebesseg_szabalyzas();
		}

		jelzes_felismeres(vonalak_szama());
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



/***************************/
//Jelzes felismerés segédváltozók
uint8_t hanyszor_volt_3_vonal = 0;
uint8_t harmas_vonal_van = 0;
uint8_t elobb_3_vonal_volt = 0;
int32_t harmas_vonal_kezdete_encoder_ertek = 0;
int32_t harmas_vonal_vege_encoder_ertek = 0;
int32_t harmas_vonal_mostani_encoder_ertek = 0;
int32_t harmas_vonal_hossza = 0;

void jelzes_felismeres(uint8_t vonal_szam){
	if(vonal_szam == 3){
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

	} else if(vonal_szam == 1){

	}

	if(harmas_vonal_van){
		harmas_vonal_vege_encoder_ertek = get_encoder_counter();
		harmas_vonal_hossza = (harmas_vonal_kezdete_encoder_ertek - harmas_vonal_vege_encoder_ertek)*ENCODER_VALUE_TO_MM;

		if(harmas_vonal_hossza > 110){
//			set_gyari_motor_compare_value(6200);
			state_of_robot = DRONE_KOVETKEZIK;
		}
		if(vonal_szam == 1){
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

uint8_t csikok_szama(uint32_t hossz){
	uint8_t csucs_kezdete = 0;
	uint8_t cnt = 0;
	uint32_t m;
	uint8_t kezdetben_egyes = 0;
	uint8_t most_fekete = 0;
	uint8_t vegig_fekete = 0;
	for(m = 0; m < hossz; m++){


		//Megnézem, hogy az elsõ 3as vagy egyes vonal
		if(meresek_3as_vonal_szama_sorrendben[0] == 0){

			kezdetben_egyes = 1;
		} else {

			kezdetben_egyes = 0;
		}


		//Ha kezdetben egyes vonal volt, akkor megvizsgálom, hogy 3as lesz-e
		//Azon belül, ha 3as lett, azt várom, hogy végig 3as legyen.
		if(kezdetben_egyes){
			if(meresek_3as_vonal_szama_sorrendben[m] == 1 && most_fekete == 0){
				most_fekete = 1;
				vegig_fekete = 1;
			}

			if(most_fekete){
				if(meresek_3as_vonal_szama_sorrendben[m] == 0){
					vegig_fekete = 0;
					kezdetben_egyes = 0;
				}
			}
		}


		if(!csucs_kezdete){
			if(meresek_3as_vonal_szama_sorrendben[m] != 0){
				csucs_kezdete = 1;

			}
		} else {
			if(meresek_3as_vonal_szama_sorrendben[m] == 0){
				csucs_kezdete = 0;
				cnt++;
			}
		}
	}

	if(vegig_fekete){
		eloszor_egyes_aztan_vegig_harmas = 1;
	}

	if(meresek_3as_vonal_szama_sorrendben[0] == 1){

		cnt--;
	} else {

	}

	return cnt;
}


void change_state(){
	uint8_t csucsok = csucsok_szama();

	if(csucsok >= 1 && csucsok < 10){
		state_of_robot = GYORSIT;
	}

	char bufi[10];
	itoa(csucsok, bufi, 10);
	BT_UART_SendString(bufi);
	BT_UART_SendString("\r\n");

	if(meresi_cikluson_beluli_3as_vonal_szam == ONE_CYCLE_MEASERES){
		state_of_robot = LASSIT;
	}

	itoa(meresi_cikluson_beluli_3as_vonal_szam, bufi, 10);
	BT_UART_SendString(bufi);
	BT_UART_SendString("\r\n");
}

void sebesseg_szabalyzas(void){
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

