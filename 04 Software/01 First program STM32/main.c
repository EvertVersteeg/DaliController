/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dali.h"
#include "debug.h"
#include <stdio.h>
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
void test(void);
void splitAdd(long input, uint8_t highbyte, uint8_t middlebyte, uint8_t lowbyte);
void initialisation(void);
void busTest(void);
uint16_t maxResponseLevel(void);
uint16_t minResponseLevel(void);
void delay_us(uint16_t us);
void transmit(uint8_t cmd1, uint8_t cmd2);
void sendByte(uint8_t b);
void sendBit(int b);
void sendZero(void);
void sendOne(void);
void scanShortAdd();
uint8_t recieve();

uint16_t CURRENT_TIMER4_COUNT =0;
uint16_t currentMillis = 0;
uint16_t systemMillis = 0;

uint8_t rx_Data[8];
#define BROADCAST_DP 		0b11111110				// Broadcast DIRECT ARC Power Control, BASE ADDRESS
#define OFF_DP 				0b00000000				//0 OFF, repeat=no, answer=no
#define UP_DP  				0b00000001				//1 UP, repeat=no, answer=no
#define DOWN_DP  			0b00000010				//2 DOWN, repeat=no, answer=no
#define STEP_DOWN_DP  		0b00001000				//4 STEP DOWN, repeat=no, answer=no
#define ON_AND_STEP_UP_DP  	0b00001000				//8 ON AND STEP UP, repeat=no, answer=no


#define BROADCAST_C 		0b11111111				// Broadcast INDIRECT (C), BASE ADDRESS
#define Address_A0			0b00000001				// Address A0, dimmable light, level 86 to 256
#define Address_A1			0b00000011				// Address A1, not dimmable light
#define Address_A2			0b00000101				// Address A2, not dimmable light

#define OFF_C 				0b00000000				//0 OFF, repeat=no, answer=no
#define UP_C  				0b00000001				//1 UP, repeat=no, answer=no
#define DOWN_C  			0b00000010				//2 DOWN, repeat=no, answer=no

#define STEP_UP_C  			0b00000011				//3 STEP UP, repeat=no, answer=no
#define STEP_DOWN_C  		0b00000100				//4 STEP DOWN, repeat=no, answer=no


#define RECALL_MAX_LEVEL_C 	0b00000101				//5 RECALL MAX LEVEL (Flash), repeat=no, answer=no
#define ON_C 				0b00000101

#define ON_AND_STEP_UP_C  	0b00001000				//8 ON AND STEP UP, repeat=no, answer=no
#define QUERY_STATUS		0b10010000				//144 Query Status, repeat=no, answer=yes
#define RESET 				0b00100000				//22 Reset, repeat=yes, answer=no



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  // HULPFUNCTIONS
  //test();
  //busTest();
  //initialisation();
  //msgMode = true;
  //scanShortAdd();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);		//set uitgang standaard hoog bij start
  printf("Start\r\n");
  msgMode = true;
  getResponse = false;
  uint8_t direction = 1;                                   	// up is 1, down is 0
  uint8_t dim_level = 86;									// Dimlevel 86 (0x56) .. 254 (0xFE)

  // SET to default, lights of dim level = low
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_Delay(20);
  transmit(Address_A0, OFF_C);
  HAL_Delay(20);
  transmit(Address_A1, OFF_C);
  HAL_Delay(20);
  transmit(Address_A2, OFF_C);
  HAL_Delay(20);
  transmit(Address_A0, OFF_C);
  HAL_Delay(20);
  transmit(Address_A0, ON_AND_STEP_UP_C);
  HAL_Delay(20);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_Delay(20);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  // READ BUTTONS
	  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == 0)								// Button 1 (PC0) is pressed
	  {
		  HAL_Delay(20);
		  transmit(Address_A1, ON_AND_STEP_UP_C);								// Light A1 => ON
		  HAL_Delay(20);
	  }
	  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 0)								// Button 2 (PC1) is pressed
	  {
		  HAL_Delay(20);
		  transmit(Address_A1, OFF_C);											// Light A1 => OFF
		  HAL_Delay(20);
	  }

	  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == 0)								// Button 3 (PC2) is pressed
	  {
		  HAL_Delay(20);
		  transmit(Address_A2, ON_AND_STEP_UP_C);								// Light A2 => ON
		  HAL_Delay(20);
	  }
	  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == 0)								// Button 4 (PC3) is pressed
	  {
		  HAL_Delay(20);
		  transmit(Address_A2, OFF_C);											// Light A2 => OFF
		  HAL_Delay(20);
	  }

	  // SET DIM LEVEL / DIRECTION

	  if (direction == 1)
	  {
		  dim_level = dim_level + 1;
		  transmit(Address_A0, STEP_UP_C);
		  HAL_Delay(20);
		  if (dim_level == 254)
		  {
			  direction = 0;
		  }
	  } else if (direction == 0)
	  {
		  dim_level= dim_level - 1;
		  transmit(Address_A0, STEP_DOWN_C);
		  HAL_Delay(20);
		  if (dim_level == 86)
		  {
			  direction = 1;
		  }
	  }
	  printf("direction : %d \n", direction);
	  printf("dim level : %d \n", dim_level);

	  // SET LEDS according to DIM LEVEL

	  if (dim_level < 107)
		  {
			  // LEDS OFF
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		  }

	  if ( (dim_level >= 107) & (dim_level < 125) )
		  {
		  	  // LED 1 ON
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		  }

	  if ( (dim_level >= 125) & (dim_level < 143) )
	  	  {
		  	  // LED 1 and 2 ON
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	  	  }

	  if ( (dim_level >= 143) & (dim_level < 161) )
	  	  {
		  	  // LED 1 to 3 ON
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	  		HAL_Delay(20);
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	  		HAL_Delay(20);
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	  		HAL_Delay(20);
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	  		HAL_Delay(20);
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	  		HAL_Delay(20);
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	  		HAL_Delay(20);
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	  		HAL_Delay(20);
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	  		HAL_Delay(20);
	  	  }
	  if ( (dim_level >= 161) & (dim_level < 179) )
	  	  {
	  		  	// LED 1 to 4 ON
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	  	  	}
	  if ( (dim_level >= 179) & (dim_level < 197) )
	  	  {
	  		  	// LED 1 to 5 ON
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	  	  	}
	  if ( (dim_level >= 197) & (dim_level < 215) )
	  	  {
	  		  	// LED 1 to 6 ON
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	  	  	}
	  if ( (dim_level >= 215) & (dim_level < 233) )
	  	  {
	  		  	// LED 1 to 7 ON
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	  	  	}
	  if ( dim_level >= 233 )
	  	  {
	  		  	// LED 1 to 8 ON
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	  	  	}

		 //HAL_Delay(2000);

		//HAL_ADC_Start(&hadc1);
		 //value_adc = HAL_ADC_GetValue(&hadc1);
		 //printf ("Analog input = %d\r\n", value_adc);


		 //delay_us(delay2);

		 //HAL_ADC_Start(&hadc1);
		 //value_adc = HAL_ADC_GetValue(&hadc1);
		 //printf ("Analog input = %d\r\n", value_adc);

		 HAL_Delay(50);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 80-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC6 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void initialisation(void)
{
	const int delaytime = 10; //ms
	long low_longadd = 0x000000;
	long high_longadd = 0xFFFFFF;
	long input;
	long longadd = (long)(low_longadd + high_longadd) / 2;
	uint8_t highbyte;
	uint8_t middlebyte;
	uint8_t lowbyte;
	uint8_t short_add = 0;
	//uint8_t cmd2;
	HAL_Delay(delaytime);
	transmit(BROADCAST_C, RESET);
	HAL_Delay(delaytime);
	transmit(BROADCAST_C, RESET);
	HAL_Delay(delaytime);
	transmit(BROADCAST_C, OFF_C);
	HAL_Delay(delaytime);
	transmit(0b10100101, 0b00000000); //initialise
	HAL_Delay(delaytime);
	transmit(0b10100101, 0b00000000); //initialise
	HAL_Delay(delaytime);
	transmit(0b10100111, 0b00000000); //randomise
	HAL_Delay(delaytime);
	transmit(0b10100111, 0b00000000); //randomise
	if (msgMode) {
		printf ("Searching for long addresses:\r\n");
		}
	while (longadd <= 0xFFFFFF - 2 && short_add <= 64) {
		while ((high_longadd - low_longadd) > 1) {

			//splitAdd(longadd, highbyte, middlebyte, lowbyte); //divide 24bit adress into three 8bit adresses
			input = longadd;
			highbyte = input >> 16;
			middlebyte = input >> 8;
			lowbyte = input;
			HAL_Delay(delaytime);
			transmit(0b10110001, highbyte); //search HB
			HAL_Delay(delaytime);
			transmit(0b10110011, middlebyte); //search MB
			HAL_Delay(delaytime);
			transmit(0b10110101, lowbyte); //search LB
			HAL_Delay(delaytime);
			transmit(0b10101001, 0b00000000); //compare

			if (minResponseLevel() > analogLevel)
			{
				low_longadd = longadd;
			}
			else
			{
				high_longadd = longadd;
			}

			longadd = (low_longadd + high_longadd) / 2; //center

			if (msgMode) {
				printf ("BIN: \r\n");
				printf("longadd = %d\r\n", (longadd + 1));
				printf (" \r\n");
				//printf ("DEC: ");
				//printf (longadd + 1, DEC);
				//printf (" ");
				//printf ("HEX: ");
				//printf (longadd + 1, HEX);
				printf("longadd = %p\r\n", (longadd + 1));
				//printf ();
			}
			else {
				//printf (longadd + 1);
				printf("longadd = %d\r\n", (longadd + 1));
			}
		} // second while


		if (high_longadd != 0xFFFFFF)
		{
			//splitAdd(longadd + 1, highbyte, middlebyte, lowbyte);
			input = longadd;
			highbyte = input >> 16;
			middlebyte = input >> 8;
			lowbyte = input;

			transmit(0b10110001, highbyte); //search HB
			HAL_Delay(delaytime);
			transmit(0b10110011, middlebyte); //search MB
			HAL_Delay(delaytime);
			transmit(0b10110101, lowbyte); //search LB
			HAL_Delay(delaytime);
			transmit(0b10110111, 1 + (short_add << 1)); //program short adress
			HAL_Delay(delaytime);
			transmit(0b10101011, 0b00000000); //withdraw
			HAL_Delay(delaytime);
			transmit((1 + (short_add << 1)), ON_C);
			HAL_Delay(1000);
			transmit(1 + (short_add << 1), OFF_C);
			HAL_Delay(delaytime);
			short_add++;

			if (msgMode) {
			printf ("Assigning a short address/r/n");
			}

			high_longadd = 0xFFFFFF;
			longadd = (low_longadd + high_longadd) / 2;

		}
		else {
			if (msgMode) {
				printf ("End/r/n");
			}
		}
	} // first while


	transmit(0b10100001, 0b00000000);  //terminate
	transmit(BROADCAST_C, ON_C);  //broadcast on
}




void splitAdd(long input, uint8_t highbyte, uint8_t middlebyte, uint8_t lowbyte)
{
	highbyte = input >> 16;
	middlebyte = input >> 8;
	lowbyte = input;
}

void busTest(void)
{
	//Luminaries must turn on and turn off. If not, check connection.
	printf("Start bus test\r\n");
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	transmit(BROADCAST_C, OFF_C);  //0x00 FE
	printf ("Lampen 2s uit\r\n");
	HAL_Delay(2000);
	transmit(BROADCAST_C, ON_AND_STEP_UP_C);
	printf ("Lampen 2s aan\r\n");  //0x00 DE
	HAL_Delay(2000);
	transmit(BROADCAST_C, OFF_C);
	printf ("Lampen 2s uit\r\n");
	HAL_Delay(2000);
	printf("Einde aansturing lampen\r\n");

	//Receive response from luminaries: max and min level
	//transmit(BROADCAST_C, QUERY_STATUS);
	maxLevel = maxResponseLevel();
	printf("maxResponseLevel = %d\r\n", maxLevel); //0x00 EC EB
	//transmit(BROADCAST_C, QUERY_STATUS);
	minLevel = minResponseLevel();
	printf("minResponseLevel = %d\r\n", minLevel); //0x00 EC EB

	analogLevel = (uint16_t)(maxLevel + minLevel) / 2;
	printf("analogLevel = %d\r\n", analogLevel);
	printf("Einde bus test\r\n");

}

void test(void)
{
	uint8_t number = 0x0000000;
	uint8_t number2 = 0x00000000;
	uint8_t i= 0;
	for (i = 0; i<64; i++){
		HAL_Delay(100);
		number2 = (number << 1) + 1;
		transmit(number2,  QUERY_STATUS);
		HAL_Delay(100);
	}
	HAL_Delay(100);
	transmit(number,  QUERY_STATUS);

}

uint16_t maxResponseLevel(void)
{
	const uint8_t dalistep = 40; //us
	HAL_ADC_Start(&hadc1);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	uint16_t rxmax = 0;
	uint32_t dalidata;
	uint32_t idalistep;
	transmit(BROADCAST_C, QUERY_STATUS);
	for (idalistep = 0; idalistep < daliTimeout; idalistep = idalistep + dalistep) {

		dalidata = HAL_ADC_GetValue(&hadc1);

		if (dalidata > rxmax) {
			rxmax = dalidata;
			};
		delay_us(dalistep);
		}
		return rxmax;
}

uint16_t minResponseLevel(void)
{
	const uint8_t dalistep = 40; //us
	uint16_t rxmin = 4095;
	uint16_t dalidata;
	uint32_t idalistep;
	transmit(BROADCAST_C, QUERY_STATUS);
	for (idalistep = 0; idalistep < daliTimeout; idalistep = idalistep + dalistep) {
		HAL_ADC_Start(&hadc1);
		dalidata = HAL_ADC_GetValue(&hadc1);

		if (dalidata < rxmin) {
			rxmin = dalidata;
			};
		delay_us(dalistep);
		}
		return rxmin;
}

/**
  * @brief Setup us delay function with timer 4
  * @retval None
  */
void delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim4,0);  // set the counter value a 0
	__HAL_TIM_ENABLE(&htim4);
	while (__HAL_TIM_GET_COUNTER(&htim4) < us);  // wait for the counter to reach the us input in the parameter
}

uint32_t micros(void)
{
    systemMillis = __HAL_TIM_GET_COUNTER(&htim4);



     //systemMillis = systemMillis + currentMillis;
     //__HAL_TIM_SET_COUNTER (&htim4, 0);

    return systemMillis;
}

void transmit(uint8_t cmd1, uint8_t cmd2) // transmit commands to DALI bus (address byte, command byte)
{
	sendBit(1);
	sendByte(cmd1);
	sendByte(cmd2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);		//digitalWrite(TxPin, HIGH); Set signaal weer hoog wanneer verzonden (STOP)
}


void sendByte(uint8_t b)
{
	for (int i = 7; i >= 0; i--)
	{
		sendBit((b >> i) & 1);
	}
}


void sendBit(int b)
{
 if (b) {
		sendOne();
	}
	else {
		sendZero();
	}
}


void sendZero(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);		//digitalWrite(TxPin, HIGH);
	delay_us(delay2);											//delayMicroseconds(delay2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);		//digitalWrite(TxPin, LOW);
	delay_us(delay1);											//delayMicroseconds(delay1);
}

void sendOne(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);		//digitalWrite(TxPin, LOW);
	delay_us(delay2); 											//delayMicroseconds(delay2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); 		//digitalWrite(TxPin, HIGH);
	delay_us(delay1);											//delayMicroseconds(delay1);
}

void scanShortAdd()
{
	const uint8_t delayTime = 10;
	const uint8_t start_ind_adres = 0;
	//const uint8_t finish_ind_adres =127;
	uint8_t add_byte;
	uint8_t device_short_add;

	transmit(BROADCAST_C, OFF_C);
	delay_us(delayTime);
	if(msgMode)
	{
		printf ("Short addresses:\r\n");
	}
	for (device_short_add = start_ind_adres; device_short_add <=63; device_short_add++)
	{
		add_byte = 1 + (device_short_add << 1);
		transmit(add_byte, 0xA1);    //1010 0001 ?? A1 moet zijn 91
		uint32_t response = recieve();
		if(getResponse){
			transmit(add_byte, OFF_C);
			HAL_Delay(1000);
			transmit(add_byte, ON_AND_STEP_UP_C);
			HAL_Delay(1000);
			transmit(add_byte, OFF_C);
			HAL_Delay(1000);
		}else {response = 0;}
		HAL_Delay(500);
	}

}

uint8_t recieve()
{

	bool previousLogicLevel = 1;
	bool currentLogicLevel = 1;
	uint8_t arrLength = 20;
	uint8_t timeArray[arrLength];
	uint8_t i = 0;
	uint8_t k = 0;
	bool logicLevelArray[arrLength];
	uint32_t response = 0;
	getResponse=false;

	__HAL_TIM_SET_COUNTER(&htim4,0);  // set the counter value a 0
	__HAL_TIM_ENABLE(&htim4);
	uint32_t startFuncTime = micros();

	while (micros() - startFuncTime < daliTimeout && i < arrLength)
	{
		HAL_ADC_Start(&hadc1);
		uint32_t value = HAL_ADC_GetValue(&hadc1);
		//printf("value = %ld\r\n", value);
		if(value > analogLevel){currentLogicLevel = 1;}else{currentLogicLevel = 0;}
		if (previousLogicLevel != currentLogicLevel){
			timeArray[i] = micros() - startFuncTime;
			logicLevelArray[i] = currentLogicLevel;
			previousLogicLevel = currentLogicLevel;
			getResponse = true;
		}
		arrLength = i;

		//decoding to manchester
		for (i = 0; i < arrLength - 1; i++) {
			if ((timeArray[i + 1] - timeArray[i]) > 0.75 * period) {
				for (k = arrLength; k > i; k--) {
					timeArray[k] = timeArray[k - 1];
					logicLevelArray[k] = logicLevelArray[k - 1];
				}
				arrLength++;
				timeArray[i + 1] = (timeArray[i] + timeArray[i + 2]) / 2;
				logicLevelArray[i + 1] = logicLevelArray[i];
			}

		}
	}
	printf("response = %ld\r\n", response);
	return response;
}



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

