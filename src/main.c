#include "bsp.h"
#include "szervo_pwm.h"
#include "motor_pwm.h"
#include "uart_communication.h"
#include "bsp.h"
#include "adc.h"
#include "dma.h"

//SPI1
SPI_HandleTypeDef spi;

uint32_t adc_eredmeny = 0;

//SPI fogadási flag
uint8_t data_received = 1;



//Szabályozás

float prev_pos = 0.0f;
float p = 0.0f;
float p_atmenet = 0.0f;
float D = 0.0f;
int32_t p_prev_konv = 0;
float error = 0.0f;
float prev_error = 0.0f;
float setValue = 0.0f;
int16_t pd_value = 0;


//PWM compare értékek állítása
uint16_t szervo_value = DIGIT_SZ_KOZEP;



//
uint16_t adcAdatok[32];
uint16_t adcAdatok_2[32];
uint16_t adcAdatok_buffer[32];

uint16_t adcAdatok_buffer_2[32];
int32_t adcAdatok_sulyozott[32];


//Hány vonalt érzékelünk
uint8_t vonalak = 0;



//QT-s házi: robot állapot
struct RobotState rs;
uint16_t sharp1;

uint32_t valami;
uint32_t * sharp2 = &valami;
uint16_t mag_dec = 0;
uint32_t encoder_value = 0;
int main(){



	// HAL_Init, System_Clock_config és hardware inicializáció
	init_all();




	char buffer[10];





//	HAL_Delay(5000);

	while(1)
	{
//		set_gyari_motor_compare_value(motor_value);
		mag_dec = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);

		HAL_Delay(50);

//		motor_value = 6500;

		encoder_value = get_encoder_counter();

//		itoa(capture_ertek, buffer, 10);
		BT_UART_SendString("Mag_dec:  ");
		itoa(mag_dec, buffer, 10);
		BT_UART_SendString(buffer);
		BT_UART_SendString("\r\n");

		itoa(encoder_value, buffer, 10);
		BT_UART_SendString(buffer);
		BT_UART_SendString("\r\n");


	}





	/*
	HAL_ADC_Start_DMA(&hadc3, &adc_eredmeny, 1);



	while(1){
		if(new_cycle){
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
			ciklus();
			encoder_value = get_encoder_counter();
			send_encoder_values_over_uart();
//			BT_UART_SendString("Hello_Hello_Hello_Hello_Hello_Hello_Hello_Hello_12");
		}
	}

*/

//	Init_gyari_motor_PWM();


	/*
	ADC_Init();
	ConfigureDMA();


//	HAL_ADC_Start_IT(&g_AdcHandle);
	HAL_ADC_Start_DMA(&hadc3, &adc_eredmeny, 1);
	for(;;)
	{



		HAL_Delay(10);

		uint8_t macska = 0;
	}


	*/
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

void ciklus(){
	uint32_t szumma_adc_values = 0;
	int32_t szumma_sulyozott = 0;

	// Adatfogadás kezdete
	data_received = 0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_SPI_Receive_IT(&spi, adcAdatok_buffer, 64);
	while(!data_received);


	data_received = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_SPI_Receive_IT(&spi, adcAdatok_buffer_2, 64);
	while(!data_received);

	// Adatfogadás vége

	if(first_cycle){
		first_cycle = 0;
	} else {

		for(uint8_t n = 0; n<32; n++){
			// Csak a világító ledekhez tartozó AD adatokat tartjuk meg
			if(adcAdatok_buffer[n] <= 2000){

				adcAdatok[n] = 0;
			} else {
				adcAdatok[n] = adcAdatok_buffer[n];
			}

			if(adcAdatok_buffer_2[n] <= 2000){

				adcAdatok_2[n] = 0;
			} else {
				adcAdatok_2[n] = adcAdatok_buffer_2[n];
			}
		}


		//arányos tényezõ számítása
		for(int i = 0; i < 32; i++)
		{
			szumma_adc_values += adcAdatok_2[i];
			adcAdatok_sulyozott[i] = adcAdatok_2[i] * sorszam[i];			//súlyozás
			szumma_sulyozott += adcAdatok_sulyozott[i];
		}

		//Pozíció
		if(szumma_adc_values != 0){
			p = (float)szumma_sulyozott/szumma_adc_values;
		} else {
			p = 0;
		}

		//Pozíció hiba
		error = p-setValue;
		//Differencia az elõzõ pozíció hibától
		D = error-prev_error;


		if(prev_pos > 14 || prev_pos < -14){
			p_atmenet = p;
			if(p_atmenet < 0){
				p_atmenet *= -1;
			}
			if(p_atmenet < 3){
				p = prev_pos;
			}
		}


		pd_value = KP_slow*p + KD_slow*D;

		prev_pos = p;
		szervo_value = DIGIT_SZ_KOZEP + (int16_t)pd_value;
		prev_error = error;
		set_compare_value_digit_szervo(szervo_value);


		new_cycle = 0;

	}


}



uint8_t vonalak_szama(){
	uint8_t csucs_kezdete = 0;
	uint8_t cnt = 0;
	uint8_t m;
	for(m = 0; m < 32; m++){
		if(!csucs_kezdete){
			if(adcAdatok_2[m] != 0){
				csucs_kezdete = 1;

			}
		} else {
			if(adcAdatok_2[m] == 0){
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
		itoa(adcAdatok_buffer[i], buff, 10);
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
		itoa(adcAdatok_buffer_2[i], buff, 10);
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


