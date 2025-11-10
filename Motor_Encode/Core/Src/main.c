/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include<math.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>

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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
int32_t Encoder_Overflow_Count = 0;
int32_t Last_Total_Count = 0;
int32_t Total_Pulse_Count = 0;//编码器变量
#define ENCODER_PPR 500//编码器每转脉冲数
#define GEAR_RATIO  34 //减速比
#define ENCODER_MULTIPLIER 4 //编码器倍频
float Motor_Shaft_Angle = 0.0f;//电机轴角度变量
float Motor_Shaft_Revolutions = 0.0f;//电机轴圈数变量
float Motor_Speed_RPM = 0.0f;//电机转速变量
uint8_t vofa_buffer[100];
float vofa_data[4];//vofa数据数组
#define SPEED_ALPHA 0.2f//速度滤波系数
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void Calculate_Motor_Angle(void);
void Calculate_Motor_Speed(void);
void VOFA_SendHeader(void);
void VOFA_SendAngleData(float angle, float revolutions, float speed, float time);
//void TIM6_IRQHandler(void);

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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);//启动编码器
  HAL_TIM_Base_Start_IT(&htim3);//启动定时器3中断
  HAL_TIM_Base_Start_IT(&htim6);//启动定时器6中断
  HAL_UART_Init(&huart1);//初始化串口1
  VOFA_SendHeader();//初始化vofa协议
  printf("STM32 Encoder Angle Reader Start\r\n");
  printf("Encoder PPR: %d, Gear Ratio: %d, Multiplier: %d\r\n", ENCODER_PPR, GEAR_RATIO, ENCODER_MULTIPLIER);//初始化后测试一下
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    static uint32_t last_send_time = 0;
    if (HAL_GetTick() - last_send_time >= 50)//50ms间隙
    {
      float current_time =(float)HAL_GetTick()/1000.0f;
      VOFA_SendAngleData(Motor_Shaft_Angle,Motor_Shaft_Revolutions,Motor_Speed_RPM,current_time);//发送数据
      last_send_time = HAL_GetTick();//更新时间
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7199;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Calculate_Motor_Angle(void)
{
  static int32_t last_total_pulse = 0;
  int16_t current_counter =__HAL_TIM_GET_COUNTER(&htim3);//获取当前计数值
  int32_t current_total = Encoder_Overflow_Count * 65536 + current_counter;//计算总脉冲数
  int32_t pulse_delta = current_total - last_total_pulse;//计算与上次的脉冲差值
  if(abs(pulse_delta)<10000){
    Total_Pulse_Count += pulse_delta;
  }//更新总脉冲数，滤除异常跳变
  last_total_pulse = current_total;//更新上次总脉冲数
  float pulses_per_revolution = ENCODER_PPR * GEAR_RATIO * ENCODER_MULTIPLIER;//计算每转总脉冲数
  Motor_Shaft_Revolutions = (float)Total_Pulse_Count / pulses_per_revolution;//计算电机轴圈数
  Motor_Shaft_Angle = fmod(Motor_Shaft_Revolutions*360.0f,360.0f);//计算电机轴角度
  if(Motor_Shaft_Angle < 0)
  {
    Motor_Shaft_Angle += 360.0f;
  }//确保角度为正值
}
void Calculate_Motor_Speed(void)
{
  static int32_t last_pulse_count = 0;
  static uint32_t last_time = 0;
  uint32_t current_time = HAL_GetTick();//获取当前时间（毫秒）
  uint32_t time_delta = current_time - last_time;//计算时间差
  if(time_delta >= 10){
    int32_t pulse_delta = Total_Pulse_Count - last_pulse_count;//计算脉冲差值
    float pulses_per_revolution = ENCODER_PPR * GEAR_RATIO * ENCODER_MULTIPLIER;//计算每转总脉冲数
    float raw_speed = (float)pulse_delta / pulses_per_revolution * (60000.0f / (float)time_delta);//计算原始速度（RPM）
    static float filtered_speed = 0.0f;
    filtered_speed = SPEED_ALPHA * raw_speed + (1 - SPEED_ALPHA) * filtered_speed;//应用低通滤波
    Motor_Speed_RPM = filtered_speed;//更新电机速度
    last_pulse_count = Total_Pulse_Count;//更新上次脉冲数
    last_time = current_time;//更新上次时间
  }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM3)//编码器计数器溢出中断
  {
    if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
    {
      Encoder_Overflow_Count--;
    }
    else
    {
      Encoder_Overflow_Count++;
    }
  }
}
void VOFA_SendAngleData(float angle,float revolutions,float speed,float time)
{
  vofa_data[0]=angle;
  vofa_data[1]=revolutions;
  vofa_data[2]=speed;
  vofa_data[3]=time;
  int len=sprintf((char *)vofa_buffer,"Angle:%.2f,Revolutions:%.2f,Speed:%.2f,Time:%.2f\r\n",
                  vofa_data[0],vofa_data[1],vofa_data[2],vofa_data[3]);
  HAL_UART_Transmit_DMA(&huart1,vofa_buffer,len);
}//发送数据函数
void VOFA_SendHeader(void)
{
  char header[]="---VOFA MOTOR ENCODER DATA---\r\n";
  HAL_UART_Transmit_DMA(&huart1,(uint8_t *)header,strlen(header));
}
/*void TIM6_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim6);
  Calculate_Motor_Angle();
  Calculate_Motor_Speed();
}
void VOFA_SendAngleData(float angle,float revolutions,float speed,float time)
{
  vofa_data[0]=angle;
  vofa_data[1]=revolutions;
  vofa_data[2]=speed;
  vofa_data[3]=time;
  int len=sprintf((char *)vofa_buffer,"Angle:%.2f,Revolutions:%.2f,Speed:%.2f,Time:%.2f\r\n",
                  vofa_data[0],vofa_data[1],vofa_data[2],vofa_data[3]);
  HAL_UART_Transmit_DMA(&huart1,vofa_buffer,len);
}//发送数据函数
void VOFA_SendHeader(void)
{
  char header[]="---VOFA MOTOR ENCODER DATA---\r\n";
  HAL_UART_Transmit_DMA(&huart1,(uint8_t *)header,strlen(header));
}*/
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
