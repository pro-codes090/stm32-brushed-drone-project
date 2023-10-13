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
#include "RC_Filters.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include "mpu6050_driver.h"
#include "lora.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {

	int16_t Roll ;
	int16_t Pitch ;
	uint16_t Throtle ;
	int16_t Yaw ;

}transmitter_chanels_t ;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_ALLOWED_THROTTLE_INPUT 950
#define MIN_ALLOWED_THROTTLE_INPUT 512
#define MAX_ALLOWED_YAW_INPUT      60
#define MAX_ALLOWED_PITCH_INPUT    60
#define MAX_ALLOWED_ROLL_INPUT     60

#define MIN_ALLOWED_YAW_INPUT     -60
#define MIN_ALLOWED_PITCH_INPUT   -60
#define MIN_ALLOWED_ROLL_INPUT    -60

#define PID_OUT_CONVERTER 0.5f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
MPU_Accl_Val_t Accl_Data ;
MPU_Gyro_Val_t Gyro_Data ;
MPU_Gyro_calib_t Gyro_Calib;

// lora related varaibles
lora_pins_t lora_pins;		// Structure variable for lora pins
lora_t lora;				// Structure variable for lora
uint8_t ret = 0  ;			// captures the return value from the functions in lora lib
uint8_t buff [15] = {0} ; //buffer to accumulate all the data
// raw transmitter received values
transmitter_chanels_t recived_channels = {0} ;

pidController_t yaw_pidController = {0};
pidController_t pitch_pidController = {0};
pidController_t roll_pidController = {0};

// filter variables
filter_t lowpass = {0} ;
// calculated motor output varaibles
uint16_t m1_out = 0 ;
uint16_t m2_out = 0 ;
uint16_t m3_out = 0 ;
uint16_t m4_out = 0 ;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void fc_powerup();
void config_wireless();
void rcv_channel();
void config_motors() ;
void config_gyro() ;
void wait_for_pair() ;
void check_if_under_range() ;
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
  HAL_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

 fc_powerup();
 config_gyro();
 config_motors();
 config_wireless();
 wait_for_pair();
 pid_init(&roll_pidController);
 pid_init(&yaw_pidController);
 pid_init(&pitch_pidController);
 // here we are having 50hz cutoff freq and 0.002 sec of sampling time i.e 2ms
 filter_init(&lowpass, 50, 0.002 ) ;
 yaw_pidController.p_gain 				= 10  ;
 yaw_pidController.i_gain               = 0 ;
 yaw_pidController.d_gain               = 0 ;
 yaw_pidController.filter_sampling_time = 0.01  ;
 yaw_pidController.sampling_time        = 0.001 ;
 yaw_pidController.limitMax             = 250   ;
 yaw_pidController.limitMin             = -250  ;
 yaw_pidController.limitMaxInt          = 300;
 yaw_pidController.limitMinInt          = -300;

 roll_pidController.p_gain 				 = 10;
 roll_pidController.i_gain               = 0 ;
 roll_pidController.d_gain               = 0 ;
 roll_pidController.filter_sampling_time = 0.01  ;
 roll_pidController.sampling_time        = 0.001 ;
 roll_pidController.limitMax             =  250 ;
 roll_pidController.limitMin             = -250 ;
 roll_pidController.limitMaxInt          =  300 ;
 roll_pidController.limitMinInt          = -300 ;

 pitch_pidController.p_gain 			  = 10  ;
 pitch_pidController.i_gain               = 0 ;
 pitch_pidController.d_gain               = 0 ;
 pitch_pidController.filter_sampling_time = 0.01  ;
 pitch_pidController.sampling_time        = 0.001 ;
 pitch_pidController.limitMax             =  250 ;
 pitch_pidController.limitMin             = -250 ;
 pitch_pidController.limitMaxInt          =  300;
 pitch_pidController.limitMinInt          = -300;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  rcv_channel();
	  get_gyro(&hi2c1, &Gyro_Data, &Gyro_Calib) ;
	  // low pass all the gyro data
	  Gyro_Data.pitch =  filter_update(&lowpass, Gyro_Data.pitch ) ;
	  Gyro_Data.yaw =  filter_update(&lowpass, Gyro_Data.yaw) ;
	  Gyro_Data.roll =  filter_update(&lowpass, Gyro_Data.roll) ;

	  printf(" %f  , %f  ,  %f \r" , Gyro_Data.pitch , Gyro_Data.yaw , Gyro_Data.roll ) ;

	  // limit the throttle channel to have some margins for pid corrections
	  if (recived_channels.Throtle > MAX_ALLOWED_THROTTLE_INPUT) {
		  recived_channels.Throtle = MAX_ALLOWED_THROTTLE_INPUT ;
	     }
	  else if (recived_channels.Throtle < MIN_ALLOWED_THROTTLE_INPUT) {
			  recived_channels.Throtle = 0 ;
		}

	  // limit the yaw channel to have some margins for pid corrections
	  if (recived_channels.Yaw > MAX_ALLOWED_YAW_INPUT) {
		  recived_channels.Yaw = MAX_ALLOWED_YAW_INPUT;
	     }
	  else if (recived_channels.Yaw < MIN_ALLOWED_YAW_INPUT) {
	      recived_channels.Yaw = MIN_ALLOWED_YAW_INPUT;
		}

	  // limit the roll channel to have some margins for pid corrections
	  if (recived_channels.Roll > MAX_ALLOWED_ROLL_INPUT) {
		  recived_channels.Roll = MAX_ALLOWED_ROLL_INPUT ;
	     }
	  else if (recived_channels.Roll < MIN_ALLOWED_ROLL_INPUT) {
		  recived_channels.Roll= MIN_ALLOWED_ROLL_INPUT;
		}

	  // limit the pitch channel to have some margins for pid corrections
	  if (recived_channels.Pitch > MAX_ALLOWED_PITCH_INPUT) {
		  recived_channels.Pitch = MAX_ALLOWED_PITCH_INPUT ;
	     }
	  else if (recived_channels.Pitch < MIN_ALLOWED_PITCH_INPUT) {
		  recived_channels.Pitch = MIN_ALLOWED_PITCH_INPUT;
		}

	  pid_update(&pitch_pidController,recived_channels.Pitch ,Gyro_Data.pitch ) ;
	  pid_update(&yaw_pidController,recived_channels.Yaw   ,Gyro_Data.yaw ) ;
	  pid_update(&roll_pidController,recived_channels.Roll  ,Gyro_Data.roll ) ;

	  m1_out = (recived_channels.Throtle - pitch_pidController.out - yaw_pidController.out - roll_pidController.out )*PID_OUT_CONVERTER;
	  m2_out = (recived_channels.Throtle + pitch_pidController.out + yaw_pidController.out - roll_pidController.out )*PID_OUT_CONVERTER;
	  m3_out = (recived_channels.Throtle + pitch_pidController.out - yaw_pidController.out + roll_pidController.out )*PID_OUT_CONVERTER;
	  m4_out = (recived_channels.Throtle - pitch_pidController.out + yaw_pidController.out + roll_pidController.out )*PID_OUT_CONVERTER;

	  __HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_1, m1_out) ;
	  __HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_2, m2_out) ;
	  __HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_3, m3_out) ;
	  __HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_4, m4_out) ;


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 32000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 6400;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 500;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 64;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 160;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 80;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 40;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 20;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 32000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 16000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void  fc_powerup(){
// turn the white and red led ON for a bit
	__HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_3 ,0);
	__HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_4 ,0);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) ;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) ;
	HAL_Delay(1000) ;
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3) ;
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4) ;

	__HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_3 ,32000);
	__HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_4 ,32000);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) ;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) ;
	HAL_Delay(1000) ;
	HAL_Delay(1000) ;
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3) ;
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4) ;

}

void config_motors() {

   __HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_1,0) ;
   __HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_2,0) ;
   __HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_3,0) ;
   __HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_4,0) ;

   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

   HAL_Delay(1000) ;
   HAL_Delay(1000) ;

}
void config_gyro() {
   Self_test_mpu6050(&hi2c1) ;
   Mpu6050_Init(&hi2c1) ;			//initalise gyroscope
   gyro_calibrate(&hi2c1,  &Gyro_Calib);

   get_gyro(&hi2c1, &Gyro_Data, &Gyro_Calib) ;
   get_Accl(&hi2c1, &Accl_Data) ;
//    take 200 readings from the imu
   for (uint8_t i = 0 ; i < 200 ; i++){
	     printf(" %0.2lf , %0.2lf , %0.2lf ,%0.2lf , %0.2lf , %0.2lf \r" , Gyro_Data.pitch , Gyro_Data.roll, Gyro_Data.yaw , Accl_Data.pitch , Accl_Data.roll, Accl_Data.yaw);
	     }

}
void wait_for_pair() {
uint8_t Key [4] = {0xAA ,0xBB , 0xCC ,0xDD };
uint8_t ret = 0 ;
uint8_t temp_buff [4] = {0} ;

while(1){

  ret = lora_prasePacket(&lora);
  if(ret){
	uint8_t i=0;
	while( i <  4){
	temp_buff[i] = lora_read(&lora);
	i++;
    }
	printf("%x  %x  %x  %x  \n" , temp_buff[0] , temp_buff[1] , temp_buff[2] , temp_buff[3]   ) ;
  }
  if (memcmp(Key , temp_buff , 4) == 0) {
	printf("key matched \n ") ;
	HAL_Delay(1000) ;
	HAL_Delay(1000) ;

  break ;
  }
  printf("looking for pair ") ;
 }

printf("done paring  \n") ;
HAL_Delay(1000) ;

}

void check_if_under_range() {

}
void config_wireless(){
	lora_pins.dio0.port  = LORA_DIO0_PORT;
	lora_pins.dio0.pin   = LORA_DIO0_PIN;
	lora_pins.nss.port   = LORA_SS_PORT;	// NSS pin to which port is connected
	lora_pins.nss.pin    = LORA_SS_PIN;		// NSS pin to which pin is connected
	lora_pins.reset.port = LORA_RESET_PORT;	// RESET pin to which port is connected
	lora_pins.reset.pin  = LORA_RESET_PIN;	// RESET pin to which pin is connected
	lora_pins.spi  			 = &hspi1;
	lora.pin = &lora_pins;
	lora.frequency = FREQ_433MHZ;	// 433MHZ Frequency

	while(lora_init(&lora)){										// Initialize the lora module
	printf("init Failed \n");
	HAL_Delay(1000);
	}
	printf("init success \n");

}

void rcv_channel(){

	  ret = lora_prasePacket(&lora);
	  if(ret){
		uint8_t i=0;
		while( i <  sizeof(recived_channels)){
		buff[i] = lora_read(&lora);
		i++;
	     }
	  }

	recived_channels.Roll    = buff[1] << 8 | buff[0] ;
	recived_channels.Pitch   = buff[3] << 8 | buff[2] ;
	recived_channels.Throtle = buff[5] << 8 | buff[4] ;
	recived_channels.Yaw     = buff[7] << 8 | buff[6] ;


}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

// for pid calculations
}

void __io_putchar(int ch) {
//	ITM_SendChar(ch) ;
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
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
