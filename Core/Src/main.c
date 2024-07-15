/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>		//for string_copy
#include <stdio.h>		//for sprintf
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static const uint8_t TMP102_ADDR = 0x48 << 1; //default address OF TMP
static const uint8_t TEMPERATURE_REG = 0x00;	//TEMPERATURE_REGISTER of TMP

static const uint8_t CJMCU_ADDR = 0x18 << 1; //default address of CJMCU
static const uint8_t AMBIENT_TEMPERATURE_REGISTER = 0x05; //0000 0101
// variables for HCSR01 sensor
#define TRIG_PIN GPIO_PIN_9
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_8
#define ECHO_PORT GPIOA
uint32_t pMillis, a;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t Distance  = 0;  // cm
char buffer[40];

uint16_t read_value_0_CJMCU;
uint16_t read_value_1_CJMCU;
uint16_t read_value_FIXED_0CJMCU;
float read_value_FIXED_1CJMCU;
uint8_t five_bits_of_readed_first_byte_CJMCU;

uint8_t buffer_get_TEMP[2];
uint16_t read_value_0_TMP102;
uint16_t read_value_FIXED_0_TMP102;
uint16_t read_value_BUF_0_INV_TMP102;
uint16_t read_value_BUF_1_INV_TMP102;
uint16_t read_value_inv_TMP102;
uint16_t value_pos_neg_TMP;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	HAL_StatusTypeDef ret;	//return status

		float temp_in_c_all_CJMCU, temp_in_c_negative_CJMCU, temp_in_c_all_TMP;
		unsigned short temp_c1_CJMCU, temp_c_TMP;
		unsigned char sign_CJMCU, sign_TMP;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

//------------------HCSR01-------------------------



  HAL_TIM_Base_Start(&htim1);
      HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

  pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
  // wait for the echo pin to go high
  while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 10 >  HAL_GetTick());
  Value1 = __HAL_TIM_GET_COUNTER (&htim1);

  pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
  // wait for the echo pin to go low
  while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
  Value2 = __HAL_TIM_GET_COUNTER (&htim1);

  Distance = (Value2-Value1)* 0.034/2;
  HAL_Delay(100);

	 sprintf(buffer, "Distance %d \r\n", Distance);
			 HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), HAL_MAX_DELAY);

			 if(Distance > 40)
			 {
				 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
			 }
			  if(Distance < 30){
				 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
			 }
			 if(Distance < 15){
				 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
			     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);}

//-----------------TIME---------------------------

			 sprintf(buffer, "elapsed time %d s\r\n",pMillis/1000);
			  HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), HAL_MAX_DELAY);
//--------------------------

			 // Tell CJMCU that we want to read from the temperature register
			 	buffer_get_TEMP[0] = AMBIENT_TEMPERATURE_REGISTER;
			 	ret = HAL_I2C_Master_Transmit(&hi2c1, CJMCU_ADDR, buffer_get_TEMP, 1,HAL_MAX_DELAY);
			 			if (ret != HAL_OK) {
			 				strcpy((char*) buffer, "Error Tx\r\n");
			 					} else {

			 						// Read 2 bytes from the temperature register
			 						ret = HAL_I2C_Master_Receive(&hi2c1, CJMCU_ADDR, buffer_get_TEMP, 2,
			 						HAL_MAX_DELAY);
			 						if (ret != HAL_OK) {
			 							strcpy((char*) buffer, "Error Rx\r\n");
			 						} else {

			 							//Read the bytes
			 							read_value_0_CJMCU = ((int16_t) buffer_get_TEMP[0] & ~(1 << 4) & ~(1 << 5)& ~(1 << 6) & ~(1 << 7));//because value is in first 4 bits and second byte
			 							read_value_1_CJMCU = (int16_t) (buffer_get_TEMP[1]);

			 							;
			 							//create new variable because if we dont do it, program will do some errors and I can explain it
			 							read_value_FIXED_0CJMCU = read_value_0_CJMCU;
			 							read_value_FIXED_1CJMCU = read_value_1_CJMCU;

			 							//to read temperature is positive or negative
			 							five_bits_of_readed_first_byte_CJMCU = (buffer_get_TEMP[0] & ~(1 << 6)	& ~(1 << 7) & ~(1 << 8));

			 							// If temperature will be negative it will be  over 0x0F by datasheet
			 							if (five_bits_of_readed_first_byte_CJMCU > 15) {
			 								sign_CJMCU = '-';

			 								//Convert the temperature data by datasheet (p.25)
			 								temp_in_c_negative_CJMCU = (read_value_FIXED_0CJMCU * 16)+ (read_value_FIXED_1CJMCU / 16);

			 								temp_in_c_all_CJMCU = 256 - temp_in_c_negative_CJMCU;
			 							} else {
			 								sign_CJMCU = '+';
			 								//Convert the temperature data by datasheet (p.25)
			 								temp_in_c_all_CJMCU = (read_value_FIXED_0CJMCU * 16)	+ (read_value_FIXED_1CJMCU / 16);

			 							}}}
			 							// Convert temperature to decimal format
			 							temp_c1_CJMCU = temp_in_c_all_CJMCU * 100;


			 							sprintf(buffer, "CJMCU  %c%d.%d  C\r\n", (sign_CJMCU),
			 									(temp_c1_CJMCU / 100), (temp_c1_CJMCU % 100));
			 							HAL_UART_Transmit(&huart2,(uint8_t*) buffer, sizeof(buffer),
			 									HAL_MAX_DELAY);


	/* ------------------TMP*/


			 										 							// Tell TMP102 that we want to read from the temperature register
			 										 									buffer_get_TEMP[0] = TEMPERATURE_REG;
			 										 									ret = HAL_I2C_Master_Transmit(&hi2c1, TMP102_ADDR, buffer_get_TEMP, 1,
			 										 											HAL_MAX_DELAY);
			 										 									if (ret != HAL_OK) {
			 										 										strcpy((char*) buffer, "Error Tx\r\n");
			 										 									} else {

			 										 										// Read 2 bytes from the temperature register
			 										 										ret = HAL_I2C_Master_Receive(&hi2c1, TMP102_ADDR, buffer_get_TEMP, 2,
			 										 												HAL_MAX_DELAY);
			 										 										if (ret != HAL_OK) {
			 										 											strcpy((char*) buffer, "Error Rx\r\n");
			 										 										} else {

			 										 											//Combine the bytes
			 										 											read_value_0_TMP102 = ((int16_t) buffer_get_TEMP[0] << 4)	| (int16_t) (buffer_get_TEMP[1] >> 4);

			 										 											//create new varible because if we dont do it, program will do some errors and I can explain it
			 										 											read_value_FIXED_0_TMP102 = read_value_0_TMP102;

			 										 											//if temperrature is negative, we must invert reading bytes,
			 										 											//so we invert bytes and then do combine
			 										 											read_value_BUF_0_INV_TMP102 =(int16_t) 0xFF ^ (buffer_get_TEMP[0]);
			 										 											read_value_BUF_1_INV_TMP102=(int16_t)0xFF^(buffer_get_TEMP[1] );

			 										 											//combine inverted bytes
			 										 											read_value_inv_TMP102 = (read_value_BUF_0_INV_TMP102 << 4 | read_value_BUF_1_INV_TMP102 >> 4);


			 										 											// If temperature will be negative it will be  over 0x7FF or 2047 by datasheet
			 										 											if (read_value_0_TMP102 > 2047) {
			 										 												sign_TMP = '-';
			 										 												value_pos_neg_TMP = read_value_inv_TMP102 + 1;
			 										 											}else {
			 										 												sign_TMP = '+';
			 										 												value_pos_neg_TMP = read_value_FIXED_0_TMP102;
			 										 											}
			 										 											// Convert to float temperature value (Celsius)
			 										 											temp_in_c_all_TMP = value_pos_neg_TMP * 0.0625;

			 										 											// Convert temperature to decimal format
			 										 											temp_c_TMP = temp_in_c_all_TMP * 100;
			 										 											sprintf(buffer, "TMP102 %c%d.%d  C\r\n", (sign_TMP), (temp_c_TMP / 100),(temp_c_TMP % 100));
			 										 											HAL_UART_Transmit(&huart2, buffer, sizeof(buffer),
			 										 													HAL_MAX_DELAY);

			 										 										}
			 										 									}
				sprintf(buffer, "                               \r\n ");  //just for clear buffer
				sprintf(buffer, "                               \r\n ");  //just for clear buffer
				HAL_UART_Transmit(&huart2,(uint8_t*) buffer, sizeof(buffer),HAL_MAX_DELAY);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|LD2_Pin|GPIO_PIN_9
                          |GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 LD2_Pin PA9
                           PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|LD2_Pin|GPIO_PIN_9
                          |GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
