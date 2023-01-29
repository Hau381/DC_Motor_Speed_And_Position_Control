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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MODE_X4 1
#define MODE_X1 0
#define MODE_V 0
#define MODE_P 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;

/* USER CODE BEGIN PV */
uint8_t u8_Rx_Data[6],u8_Tx_Data[3] , status = MODE_X4, test, state , mode = MODE_V;
int16_t enc_cnt =0,enc_cnt_pre =0, s_enc_cnt =0,e,pid,rate,s_rate,pulse1,pid1,Kp1=10,Ki1,Kd1; 
float Kp =10, Ki=10,Kd=0,Ts=0.005;
uint16_t x = 999; 


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Enc_mode(uint8_t mode);
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
  MX_DMA_Init();
  MX_TIM2_Init(); // Tim 2 encoder
  MX_TIM8_Init(); // Tim 8 PWM
  MX_UART4_Init();
  MX_TIM1_Init(); // Tim 1 PID loop
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1); 
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1|TIM_CHANNEL_2);
	//HAL_TIM_Base_Start_IT(&htim1);
	HAL_UART_Receive_DMA(&huart4,u8_Rx_Data,6);	
	//HAL_UART_Transmit_DMA(&huart4,u8_Tx_Data,9);
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//Enc_mode(MODE_X1);
	u8_Tx_Data[2] = 0xFF;
	//HAL_UART_Transmit_DMA(&huart4,u8_Tx_Data,402);
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 499;   // 10ms 1680*1000/168Mhz = 10 ms // 499 => 5ms
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1679;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF; // 65535
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */
	
  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 83; //168Mhz/ 84*x = 20kHZ
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = x;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// TIM1 interup 0.005s
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim -> Instance == TIM1){
	
	
	//enc_cnt =1000;
	if (mode == MODE_P)
	{
	enc_cnt  = __HAL_TIM_GetCounter(&htim2);
	e = s_enc_cnt - enc_cnt;

	static float u=0,u1 = 0 ,e1 = 0,e2 =0;
	u= u1 + Kp*(e-e1) + Ki*Ts*(e+e1)/2 + Kd*(e-2*e1+e2)/Ts;
	pid = u;
	if (u -pid>0.5) pid++;

	if (pid > x){
	pid = x;
	}
	else if (pid < -x){
	pid =-x;
	}
	e2= e1;
	e1 = e;
	u1=pid;
	if (pid >=0){
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, pid);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,0);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,1);
	}
	else{
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, -pid);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,0);
	}
	
		u8_Tx_Data[0] = enc_cnt>>8;
		u8_Tx_Data[1] = enc_cnt;
		u8_Tx_Data[2] = 0xFF;
	
	HAL_UART_Transmit_DMA(&huart4,u8_Tx_Data,3);
	}
		else 
	{
		enc_cnt_pre = enc_cnt;
		enc_cnt = __HAL_TIM_GetCounter(&htim2);
		pulse1 = enc_cnt - enc_cnt_pre;
			if ( pulse1>6300)
			{
				pulse1 = pulse1- 65536;
			}
			if (pulse1 < -6300)
			{
				pulse1 = pulse1+ 65536;
			}
			rate = pulse1*3000/374;
		if (s_rate !=0)
		{
			
			/* Compute PID Output for mortor1*/
			int e1 = s_rate -rate;
			static float u1_1 = 0 , u1 =0,e1_1 = 0,e2_1 =0;
			u1= u1_1 + Kp*(e1-e1_1) + Ki*Ts*(e1_1+e1)/2 + Kd*(e1-2*e1_1+e2_1)/Ts;
			pid1 = u1;
			if (u1 -pid1>0.5) pid1++;
			
			e2_1 = e1_1;
			e1_1 = e1;
			u1_1=pid1;
			
			
			if (pid1 >= 0)
			{
			if (pid1 > 999) pid1 =999;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pid1);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,0);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,1);	
			}
			else
			{
			if (pid1 < - 999) pid1 = - 999;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, - pid1);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,1);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,0);
			}
		}
		else  /* Stop motor 1*/
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,0);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,0);
		}
		u8_Tx_Data[0] = rate>>8;
		u8_Tx_Data[1] = rate;
		u8_Tx_Data[2] = 0xFE;
	
		HAL_UART_Transmit_DMA(&huart4,u8_Tx_Data,3);
	}
	}

}
// UART4 RX_interup
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (u8_Rx_Data[5] != '#' ){
		//  Error
	}
	else{
	  char* ptr ;
		
		switch (u8_Rx_Data[0]){
			case('P') :
				// update KP
			  for (int i =0; i < 4 ; i++)
				{
					ptr = (char*)(&Kp);
					*(ptr+i) =  u8_Rx_Data[i+1];
				}
				break ;
			case ('I'):
				// update Ki
			for (int i =0; i < 4 ; i++)
				{
					ptr = (char*)(&Ki);
					*(ptr+i) =  u8_Rx_Data[i+1];
				}
				break ;
			case ('D'):
				// Update Kd
			for (int i =0; i < 4 ; i++)
				{
					ptr = (char*)(&Kd);
					*(ptr+i) =  u8_Rx_Data[i+1];
				}
				HAL_TIM_Base_Start_IT(&htim1);
				break ;
			case ('S'):
				if ( u8_Rx_Data[4] == 'P')
				{
				if (mode != MODE_P){
				Enc_mode(!status);	
				mode =  MODE_P;}
				ptr = (char*)(&s_enc_cnt);
				*ptr = u8_Rx_Data[1];
				*(ptr+1) = u8_Rx_Data[2];
					
				if (u8_Rx_Data[3] == '1')
				{
					Enc_mode(MODE_X1);
				}
				else if (u8_Rx_Data[3] =='4')
				{
					Enc_mode(MODE_X4);
				}
			}
				else if ( u8_Rx_Data[4] == 'V'){
				mode = MODE_V;
				ptr = (char*)(&s_rate);
				*ptr = u8_Rx_Data[1];
				*(ptr+1) = u8_Rx_Data[2];
				}
				break ;	
			default:
				//Stop
				HAL_TIM_Base_Stop_IT(&htim1);
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
		}
	}
}
// Change mode encoder
void Enc_mode(uint8_t mode){
	TIM_Encoder_InitTypeDef sConfig = {0};
	if (mode != status){
	if (mode){
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	htim2.Init.Prescaler = 0;
	}
	else{
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	htim2.Init.Prescaler = 2;
	}
	htim2.Instance = TIM2;
  
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF; // 65535
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
	HAL_TIM_Encoder_Stop(&htim2,TIM_CHANNEL_1|TIM_CHANNEL_2);
	HAL_TIM_Encoder_DeInit(&htim2);
	HAL_TIM_Encoder_Init(&htim2,&sConfig);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1|TIM_CHANNEL_2);
	status =mode;
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
