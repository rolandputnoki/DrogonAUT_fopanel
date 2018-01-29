#include "bsp.h"
#include "szervo_pwm.h"
#include "motor_pwm.h"
#include "uart_communication.h"
#include "bsp.h"
#include "adc.h"
#include "dma.h"

#include "sharp_hosszu.h"
#include "sharp_rovid.h"

//SPI1
SPI_HandleTypeDef spi;

uint32_t adc_eredmeny = 0;

//SPI fogadási flag
uint8_t data_received = 1;



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







//Hány vonalt érzékelünk
uint8_t vonalak = 0;



//QT-s házi: robot állapot
struct RobotState rs;
uint16_t sharp1;

uint32_t valami;
uint32_t * sharp2 = &valami;
uint16_t mag_dec = 0;
uint32_t encoder_value = 0;




float wanted_speed = 1.0f;

uint8_t sebesseg_tarto_counter = 0;

float speed_diff = 0;
int main(){



	// HAL_Init, System_Clock_config és hardware inicializáció
    init_all();                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             	init_all();
    HAL_Delay(2000);



	char buffer[10];
	HAL_ADC_Start_DMA(&hadc3, &adc_eredmeny, 1);


	while(1)
	{

//		set_compare_value_digit_szervo()

		if(new_cycle){
			ciklus();
/*
			mag_dec = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);

//			HAL_Delay(20);

			encoder_value = get_encoder_counter();

	//		itoa(capture_ertek, buffer, 10);
			BT_UART_SendString("Sharp:  ");
			itoa(adc_eredmeny, buffer, 10);
			BT_UART_SendString(buffer);
			BT_UART_SendString("\r\n");
			*/
/*
			itoa(encoder_value, buffer, 10);
			BT_UART_SendString(buffer);
			BT_UART_SendString("\r\n");
*/




			if(sebesseg_tarto_counter > 10){
				speed_diff = wanted_speed - speed_of_drogon;

				motor_value += (int)(speed_diff*10);

				if(motor_value > GYARI_MOTOR_COUNTER_MAX){
					motor_value = GYARI_MOTOR_COUNTER_MAX;
				} else if(motor_value < GYARI_MOTOR_COUNTER_KOZEP){
					motor_value = GYARI_MOTOR_COUNTER_KOZEP;
				}
				set_gyari_motor_compare_value(motor_value);

/*
				itoa(adc_eredmeny , buffer, 10);
				BT_UART_SendString("AD érték:  ");
				BT_UART_SendString(buffer);
				BT_UART_SendString("\r\n");

				itoa(sharp_tomb_rovid[adc_eredmeny] , buffer, 10);
				BT_UART_SendString("Távolság mm-ben:  ");
				BT_UART_SendString(buffer);
				BT_UART_SendString("\r\n");
*/
				sebesseg_tarto_counter = 0;
			} else {
				sebesseg_tarto_counter++;
			}

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


#define ONE_CYCLE_MEASERES 20
uint8_t harom_vonal_cnt = 0;
uint8_t measure_cnt = 0;

uint8_t just_measured_three_full_lines = 0;
uint8_t just_measured_one_line = 0;
uint8_t just_measured_three_dashed_lines = 0;
uint8_t accelerate = 0;

//Milyen gyorsan változtatjuk a motor feszültségét
uint16_t cycle_counter_motor = 0;

//Állapotok: Indul, Gyorsít, Lassít,

uint8_t eppen_indulunk = 1;

uint16_t harom_vonalas_szamlalo = 0;
uint8_t kanyar_parameterek = 1;

char bt_adc_value[10];





//
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

//Orientáció
float delta_orient = 0;

void ciklus(){

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

/*
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

		pd_value = KP_slow*p_elso + KD_slow*D;
		prev_pos = p_elso;
		szervo_value = DIGIT_SZ_KOZEP + (int16_t)pd_value;
*/


//Szabályzás a pozíció és a vonal orientációja alapján
	/*
		if(prev_pos > 14 || prev_pos < -14){
			p_atmenet = p_elso;
			if(p_atmenet < 0){
				p_atmenet *= -1;
			}
			if(p_atmenet < 3){
				p_elso = prev_pos;
			}
		}
*/
		p_elso_in_mm = (2*p_elso + 1)*ElSO_KORR_MM;
		p_hatso_in_mm = (2*p_hatso + 1)*HATSO_KORR_MM;

		delta_orient = atan2((p_elso_in_mm - p_hatso_in_mm),L_SENSORS)*RADIAN_TO_DEGREE_CONV;


		pd_value = 400*p_elso + 1100*(int)delta_orient;


		szervo_value = DIGIT_SZ_KOZEP + (int16_t)pd_value;
		set_compare_value_digit_szervo(szervo_value);
		new_cycle = 0;

		char bufi[10];
		itoa((int)(szervo_value), bufi, 10);
		BT_UART_SendString(bufi);
		BT_UART_SendString("\r\n");
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


