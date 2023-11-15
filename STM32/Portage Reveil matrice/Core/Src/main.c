/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "ws2812.h"
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

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t angle = 0;
static uint8_t Heures_brt = 0;
static uint8_t Heures_U = 4;
static uint8_t Heures_D = 0;
static uint8_t Minutes_brt = 0;
static uint8_t Minutes_U = 7;
static uint8_t Minutes_D = 2;
uint8_t Cli = 0;

uint8_t interrupteur1_OLD = 0;
uint8_t interrupteur2_OLD = 0;
uint8_t interrupteur3_OLD = 0;
uint8_t interrupteur4_OLD = 0;

static uint8_t Rx_data[19];
uint16_t testData = 0;
uint8_t Alarm = 0;

uint16_t ui16_step = 0;
uint16_t ui16_loop = 0;

uint8_t ui8_Rx_2 = 0;
uint8_t ui8_Rx_3 = 0;
uint8_t ui8_Rx_11 = 0;
uint8_t ui8_Rx_12 = 0;

uint16_t ReadADC = 0;



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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  ws2812_start();
  HAL_UART_Receive_IT(&huart1, Rx_data, 19);

	uint8_t H =0;
	uint16_t FctLum=255;
	ImageData* pacManSprite;
	IndexedImageData* IndexedSprite;
	// Déclarez une instance de Canvas
	Canvas myCanvas;
	// Initialisez la structure Canvas
	myCanvas.numCols = NUM_COLS;
	myCanvas.numRows = NUM_ROWS;
	// Allouez de la mémoire pour les pixels
	myCanvas.pixels = malloc(sizeof(Pixel) * NUM_COLS * NUM_ROWS);
	// Utilisez memset pour initialiser le tableau à zéro
	memset(myCanvas.pixels, 0, sizeof(Pixel) * NUM_COLS * NUM_ROWS);

	// Vous pouvez maintenant utiliser myCanvas et les pixels initialisé


	/* start ADC */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		HAL_ADC_Start(&hadc1);
		//HAL_ADC_PollForConversion(&hadc1, 1000);
	  ReadADC = HAL_ADC_GetValue(&hadc1);

	 // HAL_ADC_Stop(&hadc1);

	  if (Alarm){
		  IndexedSprite = &NotPickleRickIndexed;
		  drawIndexedImage(IndexedSprite, (ui16_loop/2)%98, 1, 1, &myCanvas);
	  }

	  else{
		  FctLum = ReadADC/20 + 50;
	  	  setCanvasColor(&myCanvas, (Pixel){51*FctLum/255, 0*FctLum/255, 77*FctLum/255});
	  	  //drawRectangle(&myCanvas, 19, 5, 1, 1, (Pixel){0,0,0}, (Pixel){0,0,0});

	  	  // display temps en BCD
	  	  displayBCD(&myCanvas, 1, 4, Heures_D, 4, FctLum);
		  displayBCD(&myCanvas, 1, 3, Heures_D, 4, FctLum);
		  displayBCD(&myCanvas, 1, 2, Heures_D, 4, FctLum);

		  displayBCD(&myCanvas, 6, 4, Heures_U, 4, FctLum);
		  displayBCD(&myCanvas, 6, 3, Heures_U, 4, FctLum);
		  displayBCD(&myCanvas, 6, 2, Heures_U, 4, FctLum);

		  displayBCD(&myCanvas, 11, 4, Minutes_D, 4, FctLum);
		  displayBCD(&myCanvas, 11, 3, Minutes_D, 4, FctLum);
		  displayBCD(&myCanvas, 11, 2, Minutes_D, 4, FctLum);

		  displayBCD(&myCanvas, 16, 4, Minutes_U, 4, FctLum);
		  displayBCD(&myCanvas, 16, 3, Minutes_U, 4, FctLum);
		  displayBCD(&myCanvas, 16, 2, Minutes_U, 4, FctLum);



		  //autre
		  // displayBCD(&myCanvas, 1, 3, ui8_Rx_2, 8, FctLum);
		  // displayBCD(&myCanvas, 10, 3, ui8_Rx_3, 8, FctLum);

		  // displayBCD(&myCanvas, 6, 4, ui8_Rx_11, 8, FctLum);
		  // displayBCD(&myCanvas, 6, 1, ui8_Rx_12, 8, FctLum);


		  // displayBCD(&myCanvas, 2, 3, ReadADC, 16, FctLum);


		  IndexedSprite = &BadApple_2bit_7dot5fps;
		  drawIndexedImage(IndexedSprite, (ui16_loop/5), 1, 1, &myCanvas);

/*
		  	  	  for(uint8_t diag=1; diag<=23; diag++){
		  	  		  		  colorDiagonal(&myCanvas, HSVtoPixel((H + (diag* 255 / 23))%255 , MAX_LUX), diag);
		  	    	  }

		  	  	  if (H >= 255){
		  	  		  		  H=0;

		  	  		  	  }
		  	  		  	  else{
		  	  		  		  H++;
		  	  		  	  }
		  */
	  }









	 // else {







		  if (ui16_loop <= IndexedSprite->FrameAmount *2)	ui16_loop++;
		  else ui16_loop = 0;


	  	  sendCanvas(&myCanvas);



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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 250000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA2_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/******************************************************************************
* Fonction 	HAL_UART_RxCpltCallback											  *
*		Prototype	:void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart)*
*																		 	  *
*   Description des paramètres:												  *
*			UART qu'on utilise												  *
*																			  *
*		Inclus :	main.c													  *
*																			  *
*	  Description															  *
* Interruption qui va mettre dans des variables la réception UART comme le	  *
* temps										 								  *
******************************************************************************/

void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart)
{

	if(&huart1 == huart)
	{
		testData = Rx_data[3];		//Stock data fonction
		Heures_brt = Rx_data[4];	//Stock le data des heures, "brt" = brute
		Minutes_brt = Rx_data[5];	//Stock le data des minutes

		ui8_Rx_2 = Rx_data[2];
		ui8_Rx_3 = Rx_data[3];
		ui8_Rx_11 = Rx_data[11];
		ui8_Rx_12 = Rx_data[12];

		Alarm = (testData >> 3) &0x01;		// Alarm Status

		Heures_U = Heures_brt & 0x0F;	//Traite le data pour avoir l'Unité des Heures

		Heures_D = (Heures_brt & 0xF0) >> 4;	//Traite le data pour avoir la Dizaine des Heures

		Minutes_U = Minutes_brt & 0x0F;	//Traite le data pour avoir l'Unité des Minutes

		Minutes_D = (Minutes_brt & 0xF0) >> 4;	//Traite le data pour avoir la Dizaine des Minutes

		if(HAL_UART_Receive_IT(&huart1, Rx_data, 19) == HAL_ERROR)	//Réception d'UART lors d'une erreur
		{
			HAL_UART_Receive_IT(&huart1, Rx_data, 19);
		}
	}
}

/******************************************************************************
* Fonction 	HAL_UART_ErrorCallback											  *
*		Prototype	:void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart)*
*																		 	  *
*   Description des paramètres:												  *
*			UART qu'on utilise												  *
*																			  *
*		Inclus :	main.c													  *
*																			  *
*	  Description															  *
* Interruption lorsqu'il y a un problème dans la transmission UART qui permet *
* de refaire une réception									 				  *
******************************************************************************/

void HAL_UART_ErrorCallback (UART_HandleTypeDef * huart)
{
	if(&huart1 == huart)
	{
		HAL_UART_Receive_IT(&huart1, Rx_data, 19);
	}
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
