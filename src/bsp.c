/*
 * bsp.c
 *
 *  Created on: 2017. okt. 30.
 *      Author: Roland
 */

#include <szervo_pwm.h>
#include "bsp.h"


volatile int sys_delay = 0;
uint8_t new_cycle = 0;


/** Globï¿½lis funkciï¿½k inicializï¿½lï¿½sa. */


TIM_HandleTypeDef Tim7Handle;


void init_all(){
	HAL_Init(); 			//init HAL library
	SystemClock_Config(); 	//set system clock

	//Szervo indítás
	Init_AF_szervo();
	Init_Digit_Szervo_PWM();

	//Gyári motor indítás
	InitAF_gyari_motor();
//	Init_gyari_motor_PWM();

	//Motor input capture
	set_gy_rv_af_motor();
	Init_input_capture_motor();

	//Gyorsulásmérõ
	I2C_Init(0);
	LSM6DS3_Init2();

	SPI1_init();

	/* Bluetooth bekapcsolása */
	/* AutoReconnect engedélyezve van, így rögtön csatlakozik */
	BT_init_pins();
	BT_UART_Init();

	//Sharp szenzorok méréséhez az ADC csatornák inicializációja
//	ADC_Init();
//	DMA_Init();
	//
	init_pin_to_analyser();

	//A mérés ciklusidejét határozza meg
	init_cuklus_timer();
}

void init_cuklus_timer(void)
{

	//Tim7 84MHz
	__TIM7_CLK_ENABLE();

	Tim7Handle.Instance = TIM7;
	Tim7Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	Tim7Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	Tim7Handle.Init.Prescaler = 6;
	Tim7Handle.Init.Period = 52499;
	Tim7Handle.State = HAL_TIM_STATE_RESET;
	HAL_TIM_Base_Init(&Tim7Handle);
	HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM7_IRQn);
	HAL_TIM_Base_Start_IT(&Tim7Handle);
}



void TIM7_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&Tim7Handle);
}






/** us szï¿½mï¿½ mikroszekundum vï¿½rakozï¿½s (blokkolva). */
void Sys_DelayUs(int us)
{
	sys_delay = us;
	while(sys_delay);
}

/** ms szï¿½mï¿½ milliszekundum vï¿½rakozï¿½s (blokkolva). */
void Sys_DelayMs(int ms)
{
	sys_delay = ms*1000;
	while(sys_delay);
}

/* ----------------- Megszakï¿½tï¿½skezelï¿½ ï¿½s callback fï¿½ggvï¿½nyek ----------------- */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *handle)
{
	UNUSED(handle);

	if (sys_delay>0)
	{
		sys_delay--;
	}

	if (handle->Instance == TIM7)
		{
			new_cycle = 1;
		}
}


/*

void TIM4_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&Tim4Handle);
}

*/

void init_LED2(void)
{
		__GPIOA_CLK_ENABLE();
		GPIO_InitTypeDef GPIO_InitStructure;
		// Configure pin in output push/pull mode
		GPIO_InitStructure.Pin = GPIO_PIN_5;
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
		GPIO_InitStructure.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

void init_pin_to_analyser(){
	__GPIOA_CLK_ENABLE();
	__GPIOC_CLK_ENABLE();


	GPIO_InitTypeDef GPIO_InitStructure;
	// Configure pin in output push/pull mode
	GPIO_InitStructure.Pin = GPIO_PIN_12;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);


	GPIO_InitStructure.Pin = GPIO_PIN_12;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);

}

void init_user_button(void)
{
	__GPIOC_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Pin = GPIO_PIN_13;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
}



/* System Clock Configuration */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the
     device is clocked below the maximum system frequency (see datasheet). */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}
