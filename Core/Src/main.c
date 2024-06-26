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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	VOID,
	BLUE_STEP1,
	BLUE_STEP2,
	BLUE_STEP3,
	BLUE_STEP4,
	BLUE_STORAGE,
	BLUE_FIND_BALL,
	BLUE_FACING_SILO,
	BLUE_FIND_SILO,
	RED_STEP1,
	RED_STEP2,
	RED_STEP3,
	RED_STEP4,
	RED_STORAGE,
	RED_FIND_BALL,
	RED_FACING_SILO,
	RED_FIND_SILO,
	TES
} movingState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile int counter1 = 0, counter2 = 0, counter3 = 0;
volatile int counterIN1 = 0, counterIN2 = 0, counterIN3 = 0, counterIN4 = 0;

/*   IMU Sensor MPU6050   */
uint8_t receive[50];
uint32_t rxIndex = 0;
uint32_t dataIndex = 0;
double sensorData[3];

/*   CAMERA   */
/*
 * Data from camera:
 * index 0 -> Ball Distance
 * index 1 -> Ball Angle
 * index 2 -> Ball Existence
 * index 3 -> Silo 1 Distance
 * index 4 -> Silo 1 Angle
 * index 5 -> Silo 2 Distance
 * index 6 -> Silo 2 Angle
 * index 7 -> Silo 3 Distance
 * index 8 -> Silo 3 Angle
 * index 9 -> Silo 4 Distance
 * index 10 -> Silo 4 Angle
 * index 11 -> Silo 5 Distance
 * index 12 -> Silo 5 Angle
 *
 * Default distance for silo if its not detected is 9999
 * Default angle for silo if its not detected is 999
 */
uint8_t receiveCAM[50];
uint32_t indexCAM = 0;
uint32_t dataindexCAM = 0;
int camera[13];

/*   Arduino Mega   */
/*
 * Data from Arduino Mega:
 * index 0 -> Front left Distance
 * index 1 -> Front right Distance
 * index 2 -> Left Distance
 * index 3 -> Color (0 if blue else otherwise)
 * index 4 -> Proximity
 */
uint8_t receiveMEGA[50];
uint32_t indexMEGA = 0;
uint32_t dataindexMEGA = 0;
int sensorMEGA[5];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
movingState mode = VOID;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// ENCODER
	if((GPIO_Pin == EB_1_Pin) && (HAL_GPIO_ReadPin(EB_1_GPIO_Port, EB_1_Pin) == GPIO_PIN_SET))
	{
		HAL_GPIO_ReadPin(EA_1_GPIO_Port, EA_1_Pin) ? counter1-- : counter1++;
		__HAL_GPIO_EXTI_CLEAR_IT(EB_1_Pin);
	}
	else if((GPIO_Pin == EB_2_Pin) && (HAL_GPIO_ReadPin(EB_2_GPIO_Port, EB_2_Pin) == GPIO_PIN_SET))
	{
		HAL_GPIO_ReadPin(EA_2_GPIO_Port, EA_2_Pin) ? counter2-- : counter2++;
		__HAL_GPIO_EXTI_CLEAR_IT(EB_2_Pin);
	}
	else if((GPIO_Pin == EB_3_Pin) && (HAL_GPIO_ReadPin(EB_3_GPIO_Port, EB_3_Pin) == GPIO_PIN_SET))
	{
		HAL_GPIO_ReadPin(EA_3_GPIO_Port, EA_3_Pin) ? counter3-- : counter3++;
		__HAL_GPIO_EXTI_CLEAR_IT(EB_3_Pin);
	}
	else if((GPIO_Pin == EinB_1_Pin) && (HAL_GPIO_ReadPin(EinB_1_GPIO_Port, EinB_1_Pin) == GPIO_PIN_SET))
	{
		HAL_GPIO_ReadPin(EinA_1_GPIO_Port, EinA_1_Pin) ? counterIN1++ : counterIN1--;
		__HAL_GPIO_EXTI_CLEAR_IT(EinB_1_Pin);
	}
	else if((GPIO_Pin == EinB_2_Pin) && (HAL_GPIO_ReadPin(EinB_2_GPIO_Port, EinB_2_Pin) == GPIO_PIN_SET))
	{
		HAL_GPIO_ReadPin(EinA_2_GPIO_Port, EinA_2_Pin) ? counterIN2-- : counterIN2++;
		__HAL_GPIO_EXTI_CLEAR_IT(EinB_2_Pin);
	}
	else if((GPIO_Pin == EinB_3_Pin) && (HAL_GPIO_ReadPin(EinB_3_GPIO_Port, EinB_3_Pin) == GPIO_PIN_SET))
	{
		HAL_GPIO_ReadPin(EinA_3_GPIO_Port, EinA_3_Pin) ? counterIN3++ : counterIN3--;
		__HAL_GPIO_EXTI_CLEAR_IT(EinB_3_Pin);
	}
	else if((GPIO_Pin == EinB_4_Pin) && (HAL_GPIO_ReadPin(EinB_4_GPIO_Port, EinB_4_Pin) == GPIO_PIN_SET))
	{
		HAL_GPIO_ReadPin(EinA_4_GPIO_Port, EinA_4_Pin) ? counterIN4-- : counterIN4++;
		__HAL_GPIO_EXTI_CLEAR_IT(EinB_4_Pin);
	}

	// BUTTON
	else if((GPIO_Pin == Button_1_Pin) && (HAL_GPIO_ReadPin(Button_1_GPIO_Port, Button_1_Pin) == GPIO_PIN_RESET))
	{
		mode = mode + BLUE_STEP1;
		if(mode > BLUE_STEP1)
		{
			mode = VOID;
		}
		__HAL_GPIO_EXTI_CLEAR_IT(Button_1_Pin);
	}
	else if((GPIO_Pin == Button_2_Pin) && (HAL_GPIO_ReadPin(Button_2_GPIO_Port, Button_2_Pin) == GPIO_PIN_RESET))
	{
		mode = mode + BLUE_STEP1;
		if(mode > BLUE_STEP1)
		{
			mode = VOID;
		}
		__HAL_GPIO_EXTI_CLEAR_IT(Button_2_Pin);
	}
	else if((GPIO_Pin == Button_3_Pin) && (HAL_GPIO_ReadPin(Button_3_GPIO_Port, Button_3_Pin) == GPIO_PIN_RESET))
	{
		mode = mode + BLUE_STORAGE;
		if(mode > BLUE_STORAGE)
		{
			mode = VOID;
		}
		__HAL_GPIO_EXTI_CLEAR_IT(Button_3_Pin);
	}
	else if((GPIO_Pin == Button_4_Pin) && (HAL_GPIO_ReadPin(Button_4_GPIO_Port, Button_4_Pin) == GPIO_PIN_RESET))
	{
		mode = mode + BLUE_STORAGE;
		if(mode > BLUE_STORAGE)
		{
			mode = VOID;
		}
		__HAL_GPIO_EXTI_CLEAR_IT(Button_4_Pin);
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
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  if(HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
	  Error_Handler();
  }
  if(HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
	  Error_Handler();
  }
  if(HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3) != HAL_OK)
  {
	  Error_Handler();
  }
  if(HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4) != HAL_OK)
  {
	  Error_Handler();
  }
  if(HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1) != HAL_OK)
  {
	  Error_Handler();
  }
  if(HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_2) != HAL_OK)
  {
	  Error_Handler();
  }
  if(HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_3) != HAL_OK)
  {
	  Error_Handler();
  }
  if(HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_4) != HAL_OK)
  {
	  Error_Handler();
  }
  if(HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3) != HAL_OK)
  {
	  Error_Handler();
  }
  if(HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4) != HAL_OK)
  {
	  Error_Handler();
  }

  if(HAL_UART_Receive_IT(&huart1, receive, 1) != HAL_OK)
  {
	  Error_Handler();
  }
  if(HAL_UART_Receive_IT(&huart2, receiveCAM, 1) != HAL_OK)
  {
	  Error_Handler();
  }
  if(HAL_UART_Receive_IT(&huart3, receiveMEGA, 1) != HAL_OK)
  {
	  Error_Handler();
  }

  initializeSilos();

  EKF blue_step1 = {0.0, 6700.0, 0.0};
  EKF blue_step2 = {4200.0, 6700.0, 0.0};
  EKF blue_step3 = {4200.0, 9500.0, 0.0};
  EKF blue_step4 = {4200.0, 9500.0, -90.0};
  EKF blue_storage = {-2600.0, 0.0, 0.0};
  EKF blue_facing_silo_EKF = {0.0, 0.0, 90.0};
  robotPosition blue_facing_silo = {0.0, 0.0, 90.0};

  EKF red_step1 = {0.0, 6700.0, 0.0};
  EKF red_step2 = {-3500.0, 6200.0, 0.0};
  EKF red_step3 = {-3500.0, 9500.0, 0.0};
  EKF red_step4 = {-3500.0, 9500.0, 90.0};
  EKF red_storage = {-900.0, 0.0, 0.0};
  EKF red_facing_silo_EKF = {0.0, 0.0, 90.0};
  robotPosition red_facing_silo = {0.0, 0.0, 90.0};

  EKF coba[4] = {
		  {0.0, 6400.0, 0.0},
		  {3500.0, 6400.0, 0.0},
		  {3500.0, 9500.0, 0.0},
		  {3500.0, 9500.0, -90.0}
  };
  EKF tes = {0.0, 2000.0, 0.0};
  robotPosition nyoba_internal = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6700.0};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  lcd_init();
	  EKF position = extendedKalmanFilter();
	  robotPosition positionExtenal = odometry();
//	  displayKalman(position);
	  displayPosition(positionExtenal, in_global);
//	  displayCounter();

	  int frontLeftDistance = sensorMEGA[0];
	  int frontRightDistance = sensorMEGA[1];
	  int tolerance = 400;

	  bool blue_step1_check = atTargetPosition(blue_step1, position, tolerance, 1);
	  bool blue_step2_check = atTargetPosition(blue_step2, position, tolerance, 1);
	  bool blue_step3_check = atTargetPosition(blue_step3, position, tolerance, 1);
	  bool blue_step4_check = atTargetPosition(blue_step4, position, tolerance, 1);
	  bool blue_storage_check = atTargetPosition(blue_storage, position, tolerance+500, 1);

	  bool blue_facing_silo_check = atTargetExternal(blue_facing_silo, positionExtenal, tolerance+500, 1);

	  bool red_step1_check = atTargetPosition(red_step1, position, tolerance, 1);
	  bool red_step2_check = atTargetPosition(red_step2, position, tolerance, 1);
	  bool red_step3_check = atTargetPosition(red_step3, position, tolerance, 1);
	  bool red_step4_check = atTargetPosition(red_step4, position, tolerance, 1);
	  bool red_storage_check = atTargetPosition(red_storage, position, tolerance+500, 1);
	  bool red_facing_silo_check = atTargetExternal(red_facing_silo, positionExtenal, tolerance+500, 1);

	  switch(mode)
	  {
	  case BLUE_STEP1:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		  PID_Internal(nyoba_internal, 1.3, 0.0, 0.0, 2.0, 0.8, 4000);
//		  PID_moveToCoordinate(coba, 1.0, 0.0, 0.0, 1.0, tolerance, 3500);
//		  PID_KFtocoordinate(blue_step1, 1.3, 0.0, 0.0, 2.5, 0.8, 4000);
//		  if(blue_step1_check)
//		  {
//			  mode = BLUE_STEP2;
//		  }
		  break;
	  case BLUE_STEP2:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		  PID_KFtocoordinate(blue_step2, 2.4, 0.0, 0.0, 2.5, 0.8, 4000);
		  if(blue_step2_check)
		  {
			  mode = BLUE_STEP3;
		  }
		  break;
	  case BLUE_STEP3:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		  PID_KFtocoordinate(blue_step3, 1.8, 0.0, 0.0, 2.0, 0.8, 5500);
		  if(blue_step3_check)
		  {
			  mode = BLUE_STEP4;
		  }
		  break;
	  case BLUE_STEP4:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		  PID_KFtocoordinate(blue_step4, 2.0, 0.0, 0.0, 1.0, 0.8, 3000);
		  if(blue_step4_check)
		  {
			  mode = VOID;
		  }
		  break;
	  case BLUE_STORAGE:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		  setMotorSpeed(1, 0);
		  setMotorSpeed(2, 0);
		  setMotorSpeed(7, 0);
		  PID_KFtocoordinate(blue_storage, 1.3, 0.0, 0.0, 1.4, 0.6, 3000);
		  if(blue_storage_check)
		  {
			  mode = BLUE_FIND_BALL;
		  }
		  break;
	  case BLUE_FIND_BALL:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		  findAndTakeBall();
		  if(sensorMEGA[4] == 0)
		  {
			  mode = BLUE_FACING_SILO;
		  }
		  break;
	  case BLUE_FACING_SILO:
		  setMotorSpeed(1, 0);
		  setMotorSpeed(2, 0);
		  setMotorSpeed(7, 0);
		  servo_write(126);
		  PID_KFtocoordinate(blue_facing_silo_EKF, 1.5, 0.0, 0.0, 1.2, 0.6, 3000);
		  if(blue_facing_silo_check)
		  {
			  mode = BLUE_FIND_SILO;
		  }
		  break;
	  case BLUE_FIND_SILO:
		  placeBallInSilo(blue_facing_silo, 1.2, 0.0, 0.0, 1.1);
		  if((frontLeftDistance > 0 && frontLeftDistance <= 10) || (frontRightDistance > 0 && frontRightDistance <= 10))
		  {
			  Inverse_Kinematics(0, 0, 0);
			  setMotorSpeed(1, -800);
			  setMotorSpeed(2, -1000);
			  setMotorSpeed(7, -1200);
			  HAL_Delay(3000);
			  mode = BLUE_STORAGE;
		  }
		  break;
	  case RED_STEP1:
		  PID_KFtocoordinate(red_step1, 1.0, 0.0, 0.0, 1.0, 0.8, 5000);
		  if(red_step1_check)
		  {
			  mode = RED_STEP2;
		  }
		  break;
	  case RED_STEP2:
		  PID_KFtocoordinate(red_step2, 1.0, 0.0, 0.0, 1.0, 0.8, 5000);
		  if(red_step2_check)
		  {
			  mode = RED_STEP3;
		  }
		  break;
	  case RED_STEP3:
		  PID_KFtocoordinate(red_step3, 1.0, 0.0, 0.0, 1.0, 0.8, 5000);
		  if(red_step3_check)
		  {
			  mode = RED_STEP4;
		  }
		  break;
	  case RED_STEP4:
		  PID_KFtocoordinate(red_step4, 1.0, 0.0, 0.0, 1.0, 0.8, 5000);
		  if(red_step4_check)
		  {
			  mode = RED_STORAGE;
		  }
		  break;
	  case RED_STORAGE:
		  setMotorSpeed(1, 0);
		  setMotorSpeed(2, 0);
		  setMotorSpeed(7, 0);
		  PID_KFtocoordinate(red_storage, 1.3, 0.0, 0.0, 1.4, 0.6, 5000);
		  if(red_storage_check)
		  {
			  mode = RED_FIND_BALL;
		  }
		  break;
	  case RED_FIND_BALL:
		  findAndTakeBall();
		  if(sensorMEGA[4] == 0)
		  {
			  mode = RED_FACING_SILO;
		  }
		  break;
	  case RED_FACING_SILO:
		  setMotorSpeed(1, 0);
		  setMotorSpeed(2, 0);
		  setMotorSpeed(7, 0);
		  servo_write(126);
		  PID_KFtocoordinate(red_facing_silo_EKF, 1.5, 0.0, 0.0, 1.2, 0.7, 5000);
		  if(red_facing_silo_check)
		  {
			  mode = RED_FIND_SILO;
		  }
		  break;
	  case RED_FIND_SILO:
		  placeBallInSilo(red_facing_silo, 1.2, 0.0, 0.0, 1.1);
		  if((frontLeftDistance > 0 && frontLeftDistance <= 10) || (frontRightDistance > 0 && frontRightDistance <= 10))
		  {
			  Inverse_Kinematics(0, 0, 0);
			  setMotorSpeed(1, -800);
			  setMotorSpeed(2, -1000);
			  setMotorSpeed(7, -1200);
			  HAL_Delay(3000);
			  mode = RED_STORAGE;
		  }
		  break;
	  case TES:
		  PID_KFtocoordinate(tes, 1.0, 0.0, 0.0, 1.0, 0.8, 3000);
		  break;
	  default:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		  Inverse_Kinematics(0, 0, 0);
		  setMotorSpeed(1, 0);
		  setMotorSpeed(2, 0);
		  setMotorSpeed(7, 0);
		  break;
	  }
	  lcd_clear();
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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 20;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1680-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 20;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 8000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2
                          |GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE6 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EA_2_Pin EinA_1_Pin */
  GPIO_InitStruct.Pin = EA_2_Pin|EinA_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EB_2_Pin EinB_1_Pin */
  GPIO_InitStruct.Pin = EB_2_Pin|EinB_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EinB_2_Pin EinB_4_Pin */
  GPIO_InitStruct.Pin = EinB_2_Pin|EinB_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EinA_2_Pin EinA_4_Pin EA_1_Pin */
  GPIO_InitStruct.Pin = EinA_2_Pin|EinA_4_Pin|EA_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_1_Pin Button_2_Pin Button_3_Pin */
  GPIO_InitStruct.Pin = Button_1_Pin|Button_2_Pin|Button_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : EinA_3_Pin */
  GPIO_InitStruct.Pin = EinA_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EinA_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EinB_3_Pin */
  GPIO_InitStruct.Pin = EinB_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EinB_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_4_Pin */
  GPIO_InitStruct.Pin = Button_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Button_4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 PD3
                           PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : EB_1_Pin EB_3_Pin */
  GPIO_InitStruct.Pin = EB_1_Pin|EB_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : EA_3_Pin */
  GPIO_InitStruct.Pin = EA_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EA_3_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		if(receive[rxIndex] == '\r' || receive[rxIndex] == '\n')
		{
			receive[rxIndex] = '\0';
			char *token = strtok((char *)receive, ",");
			dataIndex = 0;
			while(token != NULL)
			{
				sensorData[dataIndex++] = atof(token);
				token = strtok(NULL, ",");
			}
			memset(receive, 0, sizeof(receive));
			rxIndex = 0;
			if(HAL_UART_Receive_IT(&huart1, receive, 1) != HAL_OK)
			{
				Error_Handler();
			}
		}
		else
		{
			rxIndex++;
			if(HAL_UART_Receive_IT(&huart1, receive + rxIndex, 1) != HAL_OK)
			{
				Error_Handler();
			}
		}
	}
	if(huart->Instance == USART2)
	{
		if(receiveCAM[indexCAM] == '\r' || receiveCAM[indexCAM] == '\n')
		{
			receiveCAM[indexCAM] = '\0';
			char *token = strtok((char *)receiveCAM, ",");
			dataindexCAM = 0;
			while(token != NULL)
			{
				camera[dataindexCAM++] = atoi(token);
				token = strtok(NULL, ",");
			}
			memset(receiveCAM, 0, sizeof(receiveCAM));
			indexCAM = 0;
			if(HAL_UART_Receive_IT(&huart2, receiveCAM, 1) != HAL_OK)
			{
				Error_Handler();
			}
		}
		else
		{
			indexCAM++;
			if(HAL_UART_Receive_IT(&huart2, receiveCAM + indexCAM, 1) != HAL_OK)
			{
				Error_Handler();
			}
		}
	}
	if(huart->Instance == USART3)
	{
		if(receiveMEGA[indexMEGA] == '\r' || receiveMEGA[indexMEGA] == '\n')
		{
			receiveMEGA[indexMEGA] = '\0';
			char *token = strtok((char *)receiveMEGA, ",");
			dataindexMEGA = 0;
			while(token != NULL)
			{
				sensorMEGA[dataindexMEGA++] = atoi(token);
				token = strtok(NULL, ",");
			}
			memset(receiveMEGA, 0, sizeof(receiveMEGA));
			indexMEGA = 0;
			if(HAL_UART_Receive_IT(&huart3, receiveMEGA, 1) != HAL_OK)
			{
				Error_Handler();
			}
		}
		else
		{
			indexMEGA++;
			if(HAL_UART_Receive_IT(&huart3, receiveMEGA + indexMEGA, 1) != HAL_OK)
			{
				Error_Handler();
			}
		}
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
	  Inverse_Kinematics(0, 0, 0);
	  setMotorSpeed(1, 0);
	  setMotorSpeed(2, 0);
	  setMotorSpeed(7, 0);
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
