/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "main.h"
#define TIMER_ISR_TIMING 0

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);

static void MX_TIM1_Init(void);

uint16_t timer_count_value = 1600 - 1; //this is for 100us
uint8_t LAST_READ = 0x00; //data immediately obtained after startup: 0x00000001
__vo uint32_t cmd_data;
__vo uint32_t gyro_data;
int16_t sensor_stat;
int16_t min_ = 0, max_ = -100;
__vo float pos = 0;
__vo float angular_speed = (float) 0.0;
__vo float temp;
__vo float data_at_k = (float) 0.0;
__vo float data_at_k_1 = (float) 0.0;
float time_value_for_integration = 100 * (0.000001) + TIMER_ISR_TIMING;//us
__vo CMD_SNSR_DATA_t mstr_cmd_data = { 0 };

uint8_t reading_after_first_dummy_read = 0;

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	HAL_Init();

	SystemClock_Config();
	MX_GPIO_Init();
	MX_SPI2_Init();
	MX_TIM1_Init();

	//if sensor was not yet started (or)
	//no SPI comm. has been established since startup, then :
	if (LAST_READ == 0x00)
		LAST_READ = ADXRS453_startUpSequence();

	mstr_cmd_data.cmd_bits.DATA_ACCESS = SET;
	mstr_cmd_data.cmd_bits.CHK = RESET;
	mstr_cmd_data.cmd_bits.P = 0b1 & ADXRS453_findParityBit(mstr_cmd_data.cmd);

	pos = 0;
	/*	while (1)
	 {
	 sensor_stat 	= ADXRS453_cmd_sensorData(mstr_cmd_data.cmd);
	 if(sensor_stat > max_)
	 max_ = sensor_stat;
	 if(sensor_stat < min_)
	 min_ = sensor_stat;
	 temp =  ADXRS453_decode_sensor_Data(&sensor_stat) * ((float)0.001);

	 //if(sensor_stat < -150 || sensor_stat > 0)
	 pos += temp;
	 HAL_Delay(1);
	 SET_BIT((hspi2.Instance->CR2), (1<<7) );
	 }*/

	//the sensor init was done
	//now we do a dummy read so that data will latch t=0
	sensor_stat 	= ADXRS453_cmd_sensorData(mstr_cmd_data.cmd);
	angular_speed = ADXRS453_decode_sensor_Data(&sensor_stat);
	reading_after_first_dummy_read = 0b1;
	//working of timer

	HAL_TIM_Base_Start_IT(&htim1);

	//this while loop is must => otherwise before the timer interrupt generation we will
	//have  a chip reset as the main function caller will get control of uC
	while (1) {//can be an empty loop as timer is interrupt is going on!
		/*HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		HAL_Delay(100);*/
	}
	//this following line is required to stop the timer phrl.!!!
	//HAL_TIM_Base_Stop_IT(&htim1);
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 192;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 8;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {
	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0; //=> internally for /1 , it adds 1 to this prescaler value
	htim1.Init.Period = 65535;

	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pins : SPI2_SCK_Pin SPI2_MISO_Pin SPI2_MOSI_Pin SPI2_NSS */
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*LED pins*/
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = RESET;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}
//end of MX_GPIO_Init(void)

//timer interrupt handler
void TIM1_UP_TIM10_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim1) {
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);

	//sensor_stat 	= ADXRS453_cmd_sensorData(mstr_cmd_data.cmd)   + GYRO_SENSOR_DATA_OFFSET;
	/*	sensor_stat 	= ADXRS453_cmd_sensorData(mstr_cmd_data.cmd);
	 if(sensor_stat > max_)
	 max_ = sensor_stat;
	 if(sensor_stat < min_)
	 min_ = sensor_stat;*/

	/*
	 if (reading_after_first_dummy_read == 0b1)
	 {
	 //after the 1st dummy read at main code => this following read would be the data 2y us late
	 //so store in data @ k-1 sample
	 data_at_k = ADXRS453_decode_sensor_Data(ADXRS453_cmd_sensorData(mstr_cmd_data.cmd));

	 }
	 data_at_k_1 = data_at_k;
	 data_at_k = ADXRS453_decode_sensor_Data(ADXRS453_cmd_sensorData(mstr_cmd_data.cmd));
	 //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, ON);
	 //temp = ( ADXRS453_decode_sensor_Data(&sensor_stat))* ((float)0.000001);
	 //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, OFF);

	 if(sensor_stat < -150 || sensor_stat > -40)
	 pos += temp;
	 //HAL_Delay(1);
	 //SET_BIT((hspi2.Instance->CR2), (1<<7) ); => we don't need an SPI interrupt!
	 //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);
	 */
	if (reading_after_first_dummy_read == 1) {
		reading_after_first_dummy_read = 0;
		data_at_k_1 = 0;
	} else {
		data_at_k_1 = data_at_k;
	}
	sensor_stat = ADXRS453_cmd_sensorData(mstr_cmd_data.cmd);
	data_at_k = ADXRS453_decode_sensor_Data(&sensor_stat);

	if(sensor_stat < -150 || sensor_stat > -40)
		pos += ((data_at_k + data_at_k_1) / 2) * time_value_for_integration;

}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
