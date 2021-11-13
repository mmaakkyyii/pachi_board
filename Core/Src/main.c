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
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define LED_NUM (13)

void delay_ns(uint16_t ns_count)
{

	//one clock takes 15.625 ns @64MHz
	//ex) t[ns]->  ns_count= t/15.625
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0 on
	while (__HAL_TIM_GET_COUNTER(&htim1) < ns_count){
		;  // wait for the counter to reach the us input in the parameter
	}
}

void HAL_GPIO_WritePin_wo_assert(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{

  if (PinState != GPIO_PIN_RESET)
  {
    GPIOx->BSRR = (uint32_t)GPIO_Pin;
  }
  else
  {
    GPIOx->BRR = (uint32_t)GPIO_Pin;
  }
}

void On(){
	HAL_GPIO_WritePin_wo_assert(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
}
void Off(){
	HAL_GPIO_WritePin_wo_assert(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
}
int a=10;
int b=30;
int count=0;
void code(int i){
	if(i==0){
		//On();
		__disable_irq();
	    GPIOA->BSRR = (uint32_t)GPIO_PIN_12;
	    //delay_ns(a); //300 ns
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");

	    //Off();
	    GPIOA->BRR = (uint32_t)GPIO_PIN_12;
		//delay_ns(b);
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");

	    __enable_irq();

	}else if(i==1){
		//On();
		__disable_irq();
	    GPIOA->BSRR = (uint32_t)GPIO_PIN_12;
		//delay_ns(b);
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");

	    asm("nop");
	    //Off();
	    GPIOA->BRR = (uint32_t)GPIO_PIN_12;
		//delay_ns(a);
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
	    asm("nop");
		__enable_irq();


	}else if(i==-1){
		//Off();
	    GPIOA->BRR = (uint32_t)GPIO_PIN_12;
	    //HAL_Delay(1);
	    for(int i=0;i<100;i++)delay_ns(6400);

	}else{
	    GPIOA->BRR = (uint32_t)GPIO_PIN_12;

	}
}

char RGB[LED_NUM][3];
void Show(){
	code(-1);

	for(int id=0;id<LED_NUM;id++){
		code((RGB[id][0]>>7)&1);
		code((RGB[id][0]>>6)&1);
		code((RGB[id][0]>>5)&1);
		code((RGB[id][0]>>4)&1);
		code((RGB[id][0]>>3)&1);
		code((RGB[id][0]>>2)&1);
		code((RGB[id][0]>>1)&1);
		code((RGB[id][0]>>0)&1);

		code((RGB[id][1]>>7)&1);
		code((RGB[id][1]>>6)&1);
		code((RGB[id][1]>>5)&1);
		code((RGB[id][1]>>4)&1);
		code((RGB[id][1]>>3)&1);
		code((RGB[id][1]>>2)&1);
		code((RGB[id][1]>>1)&1);
		code((RGB[id][1]>>0)&1);

		code((RGB[id][2]>>7)&1);
		code((RGB[id][2]>>6)&1);
		code((RGB[id][2]>>5)&1);
		code((RGB[id][2]>>4)&1);
		code((RGB[id][2]>>3)&1);
		code((RGB[id][2]>>2)&1);
		code((RGB[id][2]>>1)&1);
		code((RGB[id][2]>>0)&1);

	}
	//code(-1);

}
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  //while(1){}
  Off();
  code(-1);
  for(int id=0;id<LED_NUM;id++){
	  RGB[id][0]=0;
	  RGB[id][1]=0;
	  RGB[id][2]=0;
  }
  Show();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  int t=0;
	  int tt=0;
	  while(1){

		  for(int id=0;id<LED_NUM;id+=1){
//			  for(int i=0;i<LED_NUM;i++){
//				  RGB[i][0]=0;
//				  RGB[i][1]=0;
//				  RGB[i][2]=0;


				  if(t<100){
					  RGB[id][0]=t /1.0;
					  RGB[id][1]=(100-t) /1.0;
					  RGB[id][2]=0;
				  }else if(t<200){
					  RGB[id][0]=(100-(t-100) ) /1.0;
					  RGB[id][1]=0;
					  RGB[id][2]=(t-100)/1.0;
				  }else if(t<300){
					  RGB[id][0]=0;
					  RGB[id][1]=(t-200)/1.0;
					  RGB[id][2]=(100-(t-200))/1.0;
				  }else{
					  t=0;
				  }

			  }
			  Show();

			  //HAL_Delay(1);
			  //Delay(1000);


		  //HAL_Delay(50);

		  tt++;
		  if(tt>1){
			  tt=0;
			  t+=1;
		  }

	  }
	  for(int r=50;r<100;r+=5){
		  for(int id=0;id<LED_NUM;id++){
			  for(int i=0;i<LED_NUM;i++){
				  RGB[i][0]=0;
				  RGB[i][1]=0;
				  RGB[i][2]=0;
			  }
			  RGB[id][0]=r;
			  RGB[id][1]=r;
			  RGB[id][2]=0;
			  Show();
			  //HAL_Delay(1);
		  }
	  }
	  for(int r=50;r<100;r+=5){
		  for(int id=0;id<LED_NUM;id++){
			  for(int i=0;i<LED_NUM;i++){
				  RGB[i][0]=0;
				  RGB[i][1]=0;
				  RGB[i][2]=0;
			  }
			  RGB[id][0]=r;
			  RGB[id][1]=0;
			  RGB[id][2]=r;
			  Show();
			  //HAL_Delay(1);
		  }
	  }
	  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  htim1.Init.Prescaler = 0;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
