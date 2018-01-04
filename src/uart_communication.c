/*
 * uart_communication.c
 *
 *  Created on: 2017. okt. 27.
 *      Author: Roland
 */



#include "string.h"
#include "uart_communication.h"
#include "motor_pwm.h"

//Fogatodd adatok tárolása
uint8_t lastReceivedNumber = 0;
uint8_t lastReceivedMessage[BT_MESSAGE_SIZE];

//Leírók az UART perifériákhoz
UART_HandleTypeDef bt_huart, rr_huart;

//Leírók az UART konfigurálásához
UART_InitTypeDef bt_iuart, rr_iuart;

uint8_t stop = 0;
uint16_t max_motor_value = 6200;

uint16_t KP_slow = 1300;
uint16_t KD_slow = 900;
uint16_t KP_fast = 500;
uint16_t KD_fast = 100;

uint16_t indulas_max = 6570;
uint16_t gyorsulas_max = 6640;
uint16_t lassulas_min = 6540;
uint8_t state = 0;

uint16_t motor_value;

//int8_t sorszam[32] = {-18,-17,-16,-15,-14,-13,-12,-8,-7,-6,-5,-4,-3,-2,-1,0,0,1,2,3,4,5,6,7,8,12,13,14,15,16,17,18};		//szenzorsorsz�mok a s�lyoz�shoz
int8_t sorszam[32] = {-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1,0,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};		//szenzorsorsz�mok a s�lyoz�shoz


// Egyetlen bájtos adatfogadási buffer a rádió vevőtől érkező számoknak
char rr_rxBuffer;
//Több bájtos adatfogadási buffer a bluetooth modulnak
char bt_rxBuffer[BT_MESSAGE_SIZE];
char bt_rxBuffer2;

uint8_t answer_received, last_received_message_size;
// Ide másoljuk a küldés alatt álló stringet. Az UART_SendString() használja.
#define TXBUFFERSIZE 255
char txBuffer[TXBUFFERSIZE];



/** Inicializálás */
HAL_StatusTypeDef Radio_UART_Init()
{
	//Konfigurálás
	rr_iuart.BaudRate = 115200;
	rr_iuart.HwFlowCtl = UART_HWCONTROL_NONE ;
	rr_iuart.Parity = UART_PARITY_NONE;
	rr_iuart.StopBits = UART_STOPBITS_1;
	rr_iuart.WordLength = UART_WORDLENGTH_8B;
	rr_iuart.OverSampling = UART_OVERSAMPLING_16;
	rr_iuart.Mode = UART_MODE_TX_RX;
	rr_huart.Instance = USART3;
	rr_huart.Init = rr_iuart;
	HAL_UART_Init(&rr_huart);

	HAL_UART_Receive_IT(&rr_huart, &rr_rxBuffer, 1);

	return HAL_OK;
}

HAL_StatusTypeDef BT_UART_Init()
{
	//Konfigurálás
	bt_iuart.BaudRate = 115200;
	bt_iuart.HwFlowCtl = UART_HWCONTROL_NONE ;
	bt_iuart.Parity = UART_PARITY_NONE;
	bt_iuart.StopBits = UART_STOPBITS_1;
	bt_iuart.WordLength = UART_WORDLENGTH_8B;
	bt_iuart.OverSampling = UART_OVERSAMPLING_16;
	bt_iuart.Mode = UART_MODE_TX_RX;
	bt_huart.Instance = USART6;
	bt_huart.Init = bt_iuart;
	HAL_UART_Init(&bt_huart);

//	HAL_UART_Receive_IT(&bt_huart, (uint8_t *)bt_rxBuffer, BT_MESSAGE_SIZE);
	HAL_UART_Receive_IT(&bt_huart, &bt_rxBuffer2, 1);
	return HAL_OK;
}



/** Alacsony szintű inicializálás, a HAL_UART_Init() hívja. */
void HAL_UART_MspInit(UART_HandleTypeDef* handle)
{
	UNUSED(handle);

	if(handle->Instance == USART3)
	{
		__USART3_CLK_ENABLE();


		__GPIOB_CLK_ENABLE();
		GPIO_InitTypeDef pins;
		pins.Speed = GPIO_SPEED_FAST;
		pins.Pin = GPIO_PIN_10;
		pins.Mode = GPIO_MODE_AF_PP;
		pins.Pull = GPIO_NOPULL;
		pins.Alternate = GPIO_AF7_USART3;

		HAL_GPIO_Init(GPIOB, &pins);

		__GPIOC_CLK_ENABLE();
		pins.Speed = GPIO_SPEED_FAST;
		pins.Pin = GPIO_PIN_11;
		pins.Mode = GPIO_MODE_AF_PP;
		pins.Pull = GPIO_NOPULL;
		pins.Alternate = GPIO_AF7_USART3;

		HAL_GPIO_Init(GPIOC, &pins);

		HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_1);

		HAL_NVIC_SetPriority(USART3_IRQn, 1, 1);
		HAL_NVIC_EnableIRQ(USART3_IRQn);
	}
	else if(handle->Instance == USART6)
	{
		__GPIOC_CLK_ENABLE();
		__USART6_CLK_ENABLE();

		GPIO_InitTypeDef pins;
		pins.Speed = GPIO_SPEED_FAST;
		pins.Pin = GPIO_PIN_6 | GPIO_PIN_7;
		pins.Mode = GPIO_MODE_AF_PP;
		pins.Pull = GPIO_NOPULL;
		pins.Alternate = GPIO_AF8_USART6;

		HAL_GPIO_Init(GPIOC, &pins);



		HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(USART6_IRQn);
	}

}


/** String küldése, nem blokkolva. */
HAL_StatusTypeDef BT_UART_SendString(const char *str)
{
	while(bt_huart.gState !=HAL_UART_STATE_READY && bt_huart.gState != HAL_UART_STATE_BUSY_RX)
	{

	}
	strncpy(txBuffer, str, TXBUFFERSIZE);

	HAL_UART_Transmit_IT(&bt_huart, (uint8_t*) txBuffer, strlen(txBuffer));

	return HAL_OK;
}

HAL_StatusTypeDef BT_UART_Send_Bytes(const char * str){
	while(bt_huart.gState !=HAL_UART_STATE_READY && bt_huart.gState != HAL_UART_STATE_BUSY_RX)
	{

	}
	uint8_t index = 0;
	while(index < 10){
		txBuffer[index] = str[index];
		index++;
	}
	HAL_UART_Transmit_IT(&bt_huart, (uint8_t*) txBuffer, 10);

	return HAL_OK;

}


HAL_StatusTypeDef BT_UART_Send_RobotState(struct RobotState rs){
	HAL_StatusTypeDef statusa;
	while(bt_huart.gState !=HAL_UART_STATE_READY && bt_huart.gState != HAL_UART_STATE_BUSY_RX)
	{

	}
	txBuffer[0] = 0;
	txBuffer[1] = 0;
	txBuffer[2] = 0;
	txBuffer[3] = ROBOT_STATE_SIZE;
	uint8_t adc_ind = 0;
	while(adc_ind < 32){
		txBuffer[5 + adc_ind*2] = rs.sensor_values[adc_ind] & 0x00FF;
		txBuffer[4 + adc_ind*2] = (rs.sensor_values[adc_ind] & 0xFF00) >> 8;
		adc_ind++;
	}
	txBuffer[68] = rs.r_state;
	txBuffer[69] = (rs.szervo_value & 0xFF00) >> 8;
	txBuffer[70] = rs.szervo_value & 0x00FF;
	txBuffer[71] = (rs.acel_axes_values[0] & 0xFF000000) >> 24;
	txBuffer[72] = (rs.acel_axes_values[0] & 0x00FF0000) >> 16;
	txBuffer[73] = (rs.acel_axes_values[0] & 0xFF00FF00) >> 8;
	txBuffer[74] = (rs.acel_axes_values[0] & 0xFF0000FF);
	txBuffer[75] = (rs.acel_axes_values[1] & 0xFF000000) >> 24;
	txBuffer[76] = (rs.acel_axes_values[1] & 0x00FF0000) >> 16;
	txBuffer[77] = (rs.acel_axes_values[1] & 0xFF00FF00) >> 8;
	txBuffer[78] = (rs.acel_axes_values[1] & 0xFF0000FF);
	txBuffer[79] = (rs.acel_axes_values[2] & 0xFF000000) >> 24;
	txBuffer[80] = (rs.acel_axes_values[2] & 0x00FF0000) >> 16;
	txBuffer[81] = (rs.acel_axes_values[2] & 0xFF00FF00) >> 8;
	txBuffer[82] = (rs.acel_axes_values[2] & 0xFF0000FF);


	statusa = HAL_UART_Transmit_IT(&bt_huart, (uint8_t*) txBuffer, ROBOT_STATE_SIZE + 4);

	return HAL_OK;

}

HAL_StatusTypeDef BT_UART_Send_uint8_to_Qt(uint8_t value){
	while(bt_huart.gState !=HAL_UART_STATE_READY && bt_huart.gState != HAL_UART_STATE_BUSY_RX)
	{

	}
	txBuffer[0] = 0;
	txBuffer[1] = 0;
	txBuffer[2] = 0;
	txBuffer[3] = 1 + 4;
	txBuffer[4] = value;
	HAL_UART_Transmit_IT(&bt_huart, (uint8_t*) txBuffer, 5);

	return HAL_OK;
}

HAL_StatusTypeDef BT_UART_Send_to_qt(const char * str) {
	while(bt_huart.gState !=HAL_UART_STATE_READY && bt_huart.gState != HAL_UART_STATE_BUSY_RX)
	{

	}
	uint8_t length = strlen(str);
	txBuffer[0] = 0;
	txBuffer[1] = 0;
	txBuffer[2] = 0;
	txBuffer[3] = length + 4;
/*
	txBuffer[4] = 0;
	txBuffer[5] = 0;
	txBuffer[6] = 0;
	txBuffer[7] = length - 8;
	*/
	strncpy(txBuffer + 4, str, TXBUFFERSIZE);

	HAL_UART_Transmit_IT(&bt_huart, (uint8_t*) txBuffer, length+4);

	return HAL_OK;
}

uint8_t size = 0;
uint8_t comma_received = 0;

uint8_t command_bit_counter = 0;
uint8_t space_counter = 0;
char command_buffer[20];
char command_buffer_sorszam[150];
/** Callback függvény, mely sikeres adatfogadás végét jelzi. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *handle)
{
	if(handle->Instance == USART6)
	{

		/*
		uint8_t i = 0;
		while(i < BT_MESSAGE_SIZE){
			lastReceivedMessage[i] = bt_rxBuffer[i];
			i++;
		}

		HAL_UART_Receive_IT(&bt_huart, (uint8_t *)bt_rxBuffer, BT_MESSAGE_SIZE);
		*/
		/*
		if(is_connection_up()){
			if(bt_rxBuffer2 == '\n'){
				size = 0;
				HAL_UART_Receive_IT(&bt_huart, &bt_rxBuffer2, 1);
				BT_message_receive_handler();
			} else{
				lastReceivedMessage[size] = bt_rxBuffer2;
				size++;
				last_received_message_size++;
			}
		} else {
			if(bt_rxBuffer2 == '\n'){
				answer_received = 1;
				size = 0;
			} else{
				lastReceivedMessage[size] = bt_rxBuffer2;
				size++;
				last_received_message_size++;

			}

		}
		*/

		if(is_connection_up()){
			if(!comma_received){
				if(bt_rxBuffer2 == ','){
					comma_received = 1;
				}
			}else {
				if(bt_rxBuffer2 == ';'){
					command_bit_counter = 0;

					comma_received = 0;

					if(space_counter == 1){
						sscanf(command_buffer, "%u %u", &KP_slow, &KD_slow);
						for(uint8_t j = 0; j<20; j++){
							command_buffer[j] = 0;
						}
					} else if(space_counter == 31){
						sscanf(command_buffer_sorszam, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
								sorszam,
								sorszam +1,
								sorszam +2,
								sorszam +3,
								sorszam +4,
								sorszam +5,
								sorszam +6,
								sorszam +7,
								sorszam +8,
								sorszam +9,
								sorszam +10,
								sorszam +11,
								sorszam +12,
								sorszam +13,
								sorszam +14,
								sorszam +15,
								sorszam +16,
								sorszam +17,
								sorszam +18,
								sorszam +19,
								sorszam +20,
								sorszam +21,
								sorszam +22,
								sorszam +23,
								sorszam +24,
								sorszam +25,
								sorszam +26,
								sorszam +27,
								sorszam +28,
								sorszam +29,
								sorszam +30,
								sorszam +31
						);
						for(uint8_t j = 0; j<20; j++){
							command_buffer_sorszam[j] = 0;
						}
					}
					else if(space_counter == 2){
						//set_gyari_motor_compare_value(GYARI_MOTOR_COUNTER_KOZEP);
						//stop = 1;
						max_motor_value -=20;
					} else if(space_counter == 3){
//						sscanf(command_buffer, "%u %u %u %u", &KP_fast, &KD_fast, &KP_slow, &);
					} else if(space_counter == 4){
						state = 0;
						sscanf(command_buffer, "%u %u %u %u", &KP_slow, &KD_slow, &KP_fast, &KD_fast);
						for(uint8_t j = 0; j<20; j++){
							command_buffer[j] = 0;
						}
					}
					space_counter = 0;
					uint8_t valami = 1;


				} else {

					if(bt_rxBuffer2 == 32){
						space_counter++;
					}
					command_buffer[command_bit_counter] = bt_rxBuffer2;
					command_bit_counter++;
				}
			}

		} else {
			if(bt_rxBuffer2 == '\n'){
				size = 0;
			} else{
				lastReceivedMessage[size] = bt_rxBuffer2;
				size++;
			}
		}


		HAL_UART_Receive_IT(&bt_huart, &bt_rxBuffer2, 1);

	} else if(handle->Instance == USART3)
	{
		lastReceivedNumber = rr_rxBuffer;
		if(lastReceivedNumber == '0'){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		}
		HAL_UART_Receive_IT(&rr_huart, &rr_rxBuffer, 1);
	}

}


/** Interrupt kezelő függvények. */
void USART3_IRQHandler(void)
{
	HAL_UART_IRQHandler(&rr_huart);
}

void USART6_IRQHandler(void)
{
	HAL_UART_IRQHandler(&bt_huart);
}


void send_adc_values_over_bt(uint16_t *adc_values){
	char tomb[200];
	sprintf(tomb,"%u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u\r\n",
			adc_values[0],
			adc_values[1],
			adc_values[2],
			adc_values[3],
			adc_values[4],
			adc_values[5],
			adc_values[6],
			adc_values[7],
			adc_values[8],
			adc_values[9],
			adc_values[10],
			adc_values[11],
			adc_values[12],
			adc_values[13],
			adc_values[14],
			adc_values[15],
			adc_values[16],
			adc_values[17],
			adc_values[18],
			adc_values[19],
			adc_values[20],
			adc_values[21],
			adc_values[22],
			adc_values[23],
			adc_values[24],
			adc_values[25],
			adc_values[26],
			adc_values[27],
			adc_values[28],
			adc_values[29],
			adc_values[30],
			adc_values[31]
			);
	BT_UART_SendString(tomb);
}





