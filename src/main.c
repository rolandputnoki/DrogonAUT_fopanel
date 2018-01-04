#include "bsp.h"
#include "szervo_pwm.h"
#include "motor_pwm.h"
#include "uart_communication.h"
#include "bsp.h"


//SPI1
SPI_HandleTypeDef spi;


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
uint16_t adcAdatok_buffer[32];
int32_t adcAdatok_sulyozott[32];


//Hány vonalt érzékelünk
uint8_t vonalak = 0;



//QT-s házi: robot állapot
struct RobotState rs;


int main(){

	motor_value = GYARI_MOTOR_COUNTER_KOZEP;
	// HAL_Init, System_Clock_config és hardware inicializáció
	init_all();


	rs.r_state = 1;
	uint8_t l = 0;
	while(l < 32){
		rs.sensor_values[l] = 15+100*l;
		l++;
	}
	int32_t x,y,z;
	if(!LMS6DS3_Read_Axes(&x,&y,&z)){
		rs.acel_axes_values[0] = x;
		rs.acel_axes_values[1] = y;
		rs.acel_axes_values[2] = z;
	}

	while(1){
		if(new_cycle){
			ciklus();
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

void ciklus(){
	uint32_t szumma_adc_values = 0;
	int32_t szumma_sulyozott = 0;

	// Adatfogadás kezdete
	data_received = 0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_SPI_Receive_IT(&spi, adcAdatok_buffer, 64);
	while(!data_received);

	// Adatfogadás vége

	if(first_cycle){
		first_cycle = 0;
	} else {

		for(uint8_t n = 0; n<32; n++){
			rs.sensor_values[n] = adcAdatok_buffer[n];
			// Csak a világító ledekhez tartozó AD adatokat tartjuk meg
			if(adcAdatok_buffer[n] <= 2000){

				adcAdatok[n] = 0;
			} else {
				adcAdatok[n] = adcAdatok_buffer[n];
			}
		}

		if(vonalak_szama() == 3){
			harom_vonal_cnt++;
		}
		measure_cnt++;

		if(measure_cnt >= ONE_CYCLE_MEASERES){

			if(harom_vonal_cnt >= ONE_CYCLE_MEASERES ){
				harom_vonalas_szamlalo++;
			}
			//Lassítást jelzõ szakaszon vagyunk
			if(harom_vonal_cnt >= ONE_CYCLE_MEASERES && just_measured_three_full_lines == 0 ){
				just_measured_three_full_lines = 1;
				state = 2;
				accelerate = 0;
			}

			//Lassítást jelzõ szakaszon voltunk, és azt várjuk, hogy elhagyjuk teljesen
			if(just_measured_three_full_lines){
				if(harom_vonal_cnt <= 0){
					just_measured_three_full_lines = 0;
				}
			}

			//Gyorsítást jelzõ szakaszon vagyunk
			if(harom_vonal_cnt > 0 && harom_vonal_cnt <  ONE_CYCLE_MEASERES  && just_measured_three_dashed_lines == 0){
				just_measured_three_dashed_lines = 1;
				state = 1;
				kanyar_parameterek = 0;
				accelerate = 1;
			}

			//Gyorsítást jelzõ szakaszon voltunk, és azt várjuk, hogy elhagyjuk
			if(just_measured_three_dashed_lines){
				if(harom_vonal_cnt <= 0){
					just_measured_three_dashed_lines = 0;
				}
			}


			if(harom_vonalas_szamlalo >= 4){
				kanyar_parameterek = 1;
				harom_vonalas_szamlalo = 0;
			}

			harom_vonal_cnt = 0;
			measure_cnt = 0;
		}

		//
		for(int i = 0; i < 32; i++)				//arányos tényezõ számítása
		{
			szumma_adc_values += adcAdatok[i];
			adcAdatok_sulyozott[i] = adcAdatok[i] * sorszam[i];			//súlyozás
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

/*
		if(state == 2 || state == 0){
			pd_value = KP_slow*p + KD_slow*D;
		}
		else if(state == 1){
			pd_value = KP_fast*p + KD_fast*D;
		}
*/
		if(kanyar_parameterek){
			pd_value = KP_slow*p + KD_slow*D;
		} else {
			pd_value = KP_fast*p + KD_fast*D;
		}

		prev_pos = p;
		szervo_value = DIGIT_SZ_KOZEP + (int16_t)pd_value;
		prev_error = error;
		set_compare_value_digit_szervo(szervo_value);


		new_cycle = 0;



		cycle_counter_motor++;

		if(eppen_indulunk){
			//Motor feszültség változtatása, hogy ne legyen túl gyors
			if(cycle_counter_motor >= 50){

				//Indulás
				if(state == 0){
					if(motor_value < indulas_max){
						motor_value += 50;
						if(motor_value >= indulas_max){
							motor_value = lassulas_min;
							eppen_indulunk = 0;
						}
					} else {
						motor_value = lassulas_min;
						eppen_indulunk = 0;
					}
				}
				cycle_counter_motor = 0;
			}
		} else {

			if(cycle_counter_motor >= 20){

				//Gyorsítás
				if(state == 1){
					if(motor_value <= gyorsulas_max){
						motor_value += 50;
						if(dummy_flag == 0){
							dummy_flag = 1;
						} else {
							dummy_flag = 0;
						}
					}
					if(motor_value > gyorsulas_max){
						motor_value = gyorsulas_max;
					}
				}
				//Lassítás
				else if(state == 2){
					if(motor_value > lassulas_min){
						motor_value = 6400;
					} else {
						motor_value = lassulas_min;
					}
				}
				cycle_counter_motor = 0;
			}



		}

		set_gyari_motor_compare_value(motor_value);


			send_adc_values_over_bt(adcAdatok_buffer);
			send_adc_values_over_bt(adcAdatok);
			p_konv = (int32_t)(10000*p);
			D_konv = (int32_t)(10000*D);
			p_prev_konv = (int32_t)(10000*prev_pos);
			sprintf(parameter_buffer, "Szervo:%u Motor: %u Állapot: %u P:%d P_prev:%d D:%d KP_s:%u KD_s:%u KP_f:%u KD_f:%u Flag:%u\r\n", szervo_value, motor_value, state, p_konv, p_prev_konv, D_konv, KP_slow, KD_slow, KP_fast, KD_fast, cycle_counter_motor);
			BT_UART_SendString(parameter_buffer);

			rs.szervo_value = szervo_value;
		//	BT_UART_Send_RobotState(rs);



	}

}



uint8_t vonalak_szama(){
	uint8_t csucs_kezdete = 0;
	uint8_t cnt = 0;
	uint8_t m;
	for(m = 0; m < 32; m++){
		if(!csucs_kezdete){
			if(adcAdatok[m] != 0){
				csucs_kezdete = 1;

			}
		} else {
			if(adcAdatok[m] == 0){
				csucs_kezdete = 0;
				cnt++;
			}
		}
	}
	return cnt;
}

