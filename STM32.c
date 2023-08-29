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

#include "string.h"
#include "stdio.h" 
#include "stdlib.h"
#include "stdbool.h"
#include <time.h>
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define pi 3.14159

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

int16_t r_CountValue=0, l_CountValue=0;
int16_t r_set_speed=0, r_max_speed=332;
int16_t l_set_speed=0, l_max_speed=302;
char c_Rxdata[1]="x", s_Rxdata[9]="00000000", r_Rxdata[4]="0000", l_Rxdata[4]="0000";
float sample_time=0.01;
float r_cur_speed=0.0, r_kp=0.083, r_ki=0.842, r_kd=0.0004;
float l_cur_speed=0.0, l_kp=0.0843, l_ki=0.8063, l_kd=0.000395;
unsigned char State1r, State1l;
uint8_t PreviousState1, PreviousState2;
uint8_t data_RX[20];
uint8_t data_TX[20]="Hello";
int t=0, r_rpm=0, l_rpm=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
	// CHANNEL A, Motor Righ
	
	State1r = (State1r<<1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
	State1r = (State1r<<1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
	State1r = State1r&0x03;
	switch (State1r) {
		case 0:
			if(PreviousState1==1) r_CountValue++;
			else r_CountValue--;
		break;
		case 1:
			if(PreviousState1==3) r_CountValue++;
			else r_CountValue--;
		break;
		case 2:
			if(PreviousState1==0) r_CountValue++;
			else r_CountValue--;
		break;
		case 3:
			if(PreviousState1==2) r_CountValue++;
			else r_CountValue--;
		break;
		}
	PreviousState1 = State1r;
  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
	// CHANNEL B, Motor Righ
	
	State1r = (State1r<<1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
	State1r = (State1r<<1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
	State1r = State1r&0x03;
	switch (State1r) {
		case 0:
			if(PreviousState1==1) r_CountValue++;
			else r_CountValue--;
		break;
		case 1:
			if(PreviousState1==3) r_CountValue++;
			else r_CountValue--;
		break;
		case 2:
			if(PreviousState1==0) r_CountValue++;
			else r_CountValue--;
		break;
		case 3:
			if(PreviousState1==2) r_CountValue++;
			else r_CountValue--;
		break;
		}
	PreviousState1 = State1r;
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
	// CHANNEL A, Motor Left
	
	State1l = (State1l<<1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
	State1l = (State1l<<1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
	State1l = State1l&0x03;
	switch (State1l) {
		case 0:
			if(PreviousState2==1) l_CountValue++;
			else l_CountValue--;
		break;
		case 1:
			if(PreviousState2==3) l_CountValue++;
			else l_CountValue--;
		break;
		case 2:
			if(PreviousState2==0) l_CountValue++;
			else l_CountValue--;
		break;
		case 3:
			if(PreviousState2==2) l_CountValue++;
			else l_CountValue--;
		break;
		}
	PreviousState2 = State1l;
  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
	// CHANNEL B, Motor Left
	
	State1l = (State1l<<1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
	State1l = (State1l<<1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
	State1l = State1l&0x03;
	switch (State1l) {
		case 0:
			if(PreviousState2==1) l_CountValue++;
			else l_CountValue--;
		break;
		case 1:
			if(PreviousState2==3) l_CountValue++;
			else l_CountValue--;
		break;
		case 2:
			if(PreviousState2==0) l_CountValue++;
			else l_CountValue--;
		break;
		case 3:
			if(PreviousState2==2) l_CountValue++;
			else l_CountValue--;
		break;
		}
	PreviousState2 = State1l;
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function PID Of Motor Right.
  */
float right_pid(float r_set_speed, float r_cur_speed)
{
    static float r_err_p = 0;
    static float r_ui_p = 0;
    float r_err, r_up, r_ui, r_ud, r_out;
	
    if(r_set_speed > 0)
    {
        r_err= (float)r_set_speed - r_cur_speed;
    }
    if(r_set_speed < 0)
    {
        r_err = (float)(-1*r_set_speed) - (-1*r_cur_speed);
    }
		if(r_set_speed == 0)
		{
				r_err = - abs((int16_t)(0 - r_cur_speed));
		}
	
    r_up = r_kp*r_err;
    r_ud = r_kd*(r_err - r_err_p) / sample_time;
    r_ui = r_ui_p + r_ki*r_err*sample_time;
	
    r_err_p = r_err;
    r_ui_p = r_ui;

    r_out = r_up + r_ui + r_ud;

    if(r_out > r_max_speed)
    {
        r_out = r_max_speed;
    }
    
    else if(r_out < 0.0)
    {
        r_out = 0.0;
    }
    return r_out;
}

/**
  * @brief This function PID Of Motor Right.
  */
float left_pid(float l_set_speed, float l_cur_speed)
{
    static float l_err_p = 0;
    static float l_ui_p = 0;
    float l_err, l_up, l_ui, l_ud, l_out;
	
    if(l_set_speed > 0)
    {
        l_err = (float)l_set_speed - l_cur_speed;
    }
    if(l_set_speed < 0)
    {
        l_err = (float)(-1*l_set_speed) - (-1*l_cur_speed);
    }
		if(l_set_speed == 0)
    {
        l_err = - abs((int16_t)(0 - l_cur_speed));
    }
	
    l_up = l_kp*l_err;
    l_ud = l_kd*(l_err - l_err_p) / sample_time;
    l_ui = l_ui_p + l_ki*l_err * sample_time;
	
    l_err_p = l_err;
    l_ui_p = l_ui;

    l_out = l_up + l_ui + l_ud;

    if(l_out > l_max_speed)
    {
        l_out = l_max_speed;
    }
    
    else if(l_out < 0.0)
    {
        l_out = 0.0;
    }
    return l_out;
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
	
	if(huart->Instance == USART1)
	{
		if(c_Rxdata[0] == 't')
		{
			HAL_UART_Receive(&huart1,(uint8_t*)&s_Rxdata,8,20);
			r_Rxdata[0] = s_Rxdata[0];
			r_Rxdata[1] = s_Rxdata[1];
			r_Rxdata[2] = s_Rxdata[2];
			r_Rxdata[3] = s_Rxdata[3];
			l_Rxdata[0] = s_Rxdata[4];
			l_Rxdata[1] = s_Rxdata[5];
			l_Rxdata[2] = s_Rxdata[6];
			l_Rxdata[3] = s_Rxdata[7];
			s_Rxdata[4] = 0;
			r_set_speed = atoi(s_Rxdata);
			l_set_speed = atoi(l_Rxdata);
			c_Rxdata[0] = 'x';
		}
		HAL_UART_Receive_IT(&huart1,(uint8_t*)&c_Rxdata,1);
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//printf("OK \n");
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);	// khoi tao timer 2 PWM
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);	// khoi tao timer 3 PWM
	HAL_TIM_Base_Start_IT(&htim1);						// khoi tao timer 1
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); //r_dir+
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); //l_dir+
	
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, 0); //r_pwm+
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, 0); //l_pwm+
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_UART_Receive_IT(&huart1,(uint8_t*)&c_Rxdata,1);
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
//		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, 30);
//		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, 22);
//		HAL_Delay(5000);
//		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, 0);
//		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, 0);
//		HAL_Delay(1000);
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
//		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, 30);
//		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, 22);
//		HAL_Delay(5000);
//		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, 0);
//		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, 0);
//		HAL_Delay(1000);
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  htim1.Init.Prescaler = 319;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 249;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 124;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 124;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {	// ngat timer 1 tinh van toc
	
	if(htim->Instance==TIM1)	// ngat timer 1:	10ms
	{
		r_cur_speed=r_CountValue*60*2.0/(1983.0*sample_time);
		l_cur_speed=l_CountValue*60*2.0/(1983.0*sample_time);
		l_CountValue=0;
		r_CountValue=0;
		
		if(r_set_speed > 0)
		{	
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); //r_dir+
		}
		if(r_set_speed < 0) 
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); //r_dir+
		}
		if(l_set_speed > 0)
		{	
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); //l_dir+
		}
		if(l_set_speed < 0) 
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); //l_dir+
		}
		
		r_rpm = abs((int)(r_set_speed*120.0/r_max_speed)) + 13.0;
		l_rpm = abs((int)(l_set_speed*112.0/l_max_speed)) + 5.0;
		if(r_rpm >= 85){r_rpm = 85;}
		if(l_rpm >= 76){l_rpm = 76;}
		
    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, r_rpm);
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, l_rpm);
		
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
