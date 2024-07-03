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

	RED_STEP,
	RED_STORAGE,
	RED_FIND_BALL,
	RED_FACING_SILO,
	RED_FIND_SILO,
	RED_RETRY,
	RED_RETRY_STORAGE,
	RED_RETRY_FIND_BALL,
	RED_RETRY_FACING_SILO,
	RED_RETRY_FIND_SILO,

	BLUE_STEP,
	BLUE_STORAGE,
	BLUE_FIND_BALL,
	BLUE_FACING_SILO,
	BLUE_FIND_SILO,
	BLUE_RETRY,
	BLUE_RETRY_STORAGE,
	BLUE_RETRY_FIND_BALL,
	BLUE_RETRY_FACING_SILO,
	BLUE_RETRY_FIND_SILO,

	TES,
	TES2
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
 * index 2 -> Ball Existence -> Diubah jadi cek bola di dalam robot (0 gak ada bola, 1 ada bola)
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
 * index 13 -> Silo 1 is full?
 * index 14 -> Silo 2 is full?
 * index 15 -> Silo 3 is full?
 * index 16 -> Silo 4 is full?
 * index 17 -> Silo 5 is full?
 *
 * Default distance for silo if its not detected is 9999
 * Default angle for silo if its not detected is 999
 */
uint8_t receiveCAM[50];
uint32_t indexCAM = 0;
uint32_t dataindexCAM = 0;
int camera[18];

/*   Arduino Mega   */
/*
 * Data from Arduino Mega:
 * index 0 -> Front left Distance
 * index 1 -> Front right Distance
 * index 2 -> Left Distance
 * index 3 -> Proximity
 */
uint8_t receiveMEGA[50];
uint32_t indexMEGA = 0;
uint32_t dataindexMEGA = 0;
int sensorMEGA[4];
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
char buffMode[10];

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
		mode = mode + 1;
		if(mode > 22)
		{
			mode = VOID;
		}
		__HAL_GPIO_EXTI_CLEAR_IT(Button_1_Pin);
	}
	else if((GPIO_Pin == Button_2_Pin) && (HAL_GPIO_ReadPin(Button_2_GPIO_Port, Button_2_Pin) == GPIO_PIN_RESET))
	{
		mode = mode + 1;
		if(mode > 22)
		{
			mode = VOID;
		}
		__HAL_GPIO_EXTI_CLEAR_IT(Button_2_Pin);
	}
	else if((GPIO_Pin == Button_3_Pin) && (HAL_GPIO_ReadPin(Button_3_GPIO_Port, Button_3_Pin) == GPIO_PIN_RESET))
	{
		mode = mode - 1;
		if(mode < 0)
		{
			mode = 22;
		}
		__HAL_GPIO_EXTI_CLEAR_IT(Button_3_Pin);
	}
	else if((GPIO_Pin == Button_4_Pin) && (HAL_GPIO_ReadPin(Button_4_GPIO_Port, Button_4_Pin) == GPIO_PIN_RESET))
	{
		mode = mode + 1;
		if(mode > 22)
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
  void displayMode();
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

  external_global red_step[3] = {
		  {0.0, 6200.0, 0.0},
		  {3750.0, 6200.0, 0.0},
		  {3750.0, 9500.0, 0.0}
  };
  external_global red_storage = {1100.0, 9500.0, 0.0};
  external_global red_silo = {3700.0, 9500.0, 90.0};
  external_global red_throwBall = {1100.0, 8500.0, -179.0};

  external_global redBall[3] = {
		  {1150.0, 9500.0, -90.0},
		  {1150.0, 10500.0, -179.0},
		  {1150.0, 8500.0, 0.0}
  };

  external_global red_retry[4] = {
		  {500.0, 0.0, 0.0},
		  {500.0, 1000.0, 0.0},
		  {3800.0, 1000.0, 0.0},
		  {3800.0, 4300.0, 0.0}
  };
  external_global red_retry_storage = {1100.0, 4300.0, 0.0};
  external_global red_retry_silo = {3750.0, 4300.0, 90.0};
  external_global red_retry_throwBall = {1100.0, 3300.0, -179.0};

  external_global redRetryBall[3] = {
		  {1150.0, 4300.0, -90.0},
		  {1150.0, 5300.0, -179.0},
		  {1150.0, 3300.0, 0.0}
  };



  external_global blue_step[3] = {
		  {0.0, 6200.0, 0.0},
		  {-3750.0, 6200.0, 0.0},
		  {-3750.0, 9500.0, 0.0}
  };
  external_global blue_storage = {-1100.0, 9500.0, 0.0};
  external_global blue_silo = {-3700.0, 9500.0, -90.0};
  external_global blue_throwBall = {-1100.0, 8500.0, -179.0};

  external_global blueBall[3] = {
		  {-1100.0, 9500.0, 90.0},
		  {-1100.0, 10500.0, -179.0},
		  {-1100.0, 8500.0, 0.0}
  };

  external_global blue_retry[4] = {
		  {-500.0, 0.0, 0.0},
		  {-500.0, 1000.0, 0.0},
		  {-3800.0, 1000.0, 0.0},
		  {-3800.0, 4300.0, 0.0}
  };
  external_global blue_retry_storage = {-1150.0, 4300.0, 0.0};
  external_global blue_retry_silo = {-3750.0, 4300.0, -90.0};
  external_global blue_retry_throwBall = {-1100.0, 3300.0, 179.0};

  external_global blueRetryBall[3] = {
		  {-1150.0, 4300.0, 90.0},
		  {-1150.0, 5300.0, -179.0},
		  {-1150.0, 3300.0, 0.0}
  };



  external_global tes = {0.0, 0.0, 0.0};
  /* PID_parameter
	double Kp;
	double Ki;
	double Kd;
	double KpH;
	double smoothingFactor;
	int maxVelocity;
	double xyTolerance;
	double hTolerance;
   */
  PID_parameter red_step_parameters[3] = {
		  {1.25, 0.0, 0.0, 2.5, 0.8, 3000, 200, 2},
		  {1.8, 0.0, 0.0, 3.5, 0.75, 3500, 200, 1},
		  {1.5, 0.0, 0.0, 2.5, 0.8, 3500, 200, 5}
  };

  PID_parameter red_retry_parameters[4] = {
		  {5.0, 0.0, 0.0, 2.5, 0.8, 2500, 200, 2},
		  {2.6, 0.0, 0.0, 2.5, 0.75, 2500, 200, 2},
		  {1.8, 0.0, 0.0, 3.0, 0.75, 3500, 200, 1},
		  {1.5, 0.0, 0.0, 2.8, 0.8, 3500, 200, 5}
  };

  uint16_t red_step_numPoints = sizeof(red_step) / sizeof(red_step[0]);
  uint16_t red_retry_numPoints = sizeof(red_retry) / sizeof(red_retry[0]);



  PID_parameter blue_step_parameters[3] = {
		  {1.25, 0.0, 0.0, 2.5, 0.8, 3000, 200, 2},
		  {1.8, 0.0, 0.0, 3.5, 0.75, 3500, 200, 1},
		  {1.5, 0.0, 0.0, 2.5, 0.8, 3500, 200, 5}
  };

  PID_parameter blue_retry_parameters[4] = {
		  {5.0, 0.0, 0.0, 2.5, 0.8, 2500, 200, 2},
		  {2.6, 0.0, 0.0, 2.5, 0.75, 2500, 200, 2},
		  {1.8, 0.0, 0.0, 3.0, 0.75, 3500, 200, 1},
		  {1.5, 0.0, 0.0, 2.8, 0.8, 3500, 200, 5}
  };

  uint16_t blue_step_numPoints = sizeof(blue_step) / sizeof(blue_step[0]);
  uint16_t blue_retry_numPoints = sizeof(blue_retry) / sizeof(blue_retry[0]);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  lcd_init();

	  int FL_distance = sensorMEGA[0];
	  int FR_distance = sensorMEGA[2];

	  external_global position = odometry_eg();
	  displayMode();
	  display_EG();

//	  displaySilo();
//	  displayCounter();
//	  displayBall();

	  if(fabs(position.x) > 99999 || fabs(position.y) > 99999)
	  {
		  Error_Handler();
	  }

	  bool red_step_check = atTargetEG(red_step[red_step_numPoints-1], position, 400, 5);
	  bool red_storage_check = atTargetEG(red_storage, position, 400, 1);
	  bool red_silo_check = atTargetEG(red_silo, position, 400, 1);

	  bool red_retry_check = atTargetEG(red_retry[red_retry_numPoints-1], position, 400, 5);
	  bool red_retry_storage_check = atTargetEG(red_retry_storage, position, 400, 1);
	  bool red_retry_silo_check = atTargetEG(red_retry_silo, position, 400, 1);


	  bool blue_step_check = atTargetEG(blue_step[blue_step_numPoints-1], position, 400, 5);
	  bool blue_storage_check = atTargetEG(blue_storage, position, 400, 1);
	  bool blue_silo_check = atTargetEG(blue_silo, position, 400, 1);

	  bool blue_retry_check = atTargetEG(blue_retry[blue_retry_numPoints-1], position, 400, 5);
	  bool blue_retry_storage_check = atTargetEG(blue_retry_storage, position, 400, 1);
	  bool blue_retry_silo_check = atTargetEG(blue_retry_silo, position, 400, 1);

	  switch(mode)
	  {
	  case RED_STEP:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

		  PID_moveToCoordinate(red_step, red_step_parameters, red_step_numPoints);
		  if(red_step_check)
		  {
			  mode = RED_STORAGE;
		  }
		  break;

	  case RED_STORAGE:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

		  setMotorSpeed(1, 0);
		  setMotorSpeed(2, 0);
		  setMotorSpeed(7, 0);
		  PID_EG(red_storage, 1.8, 0.0, 0.0, 1.5, 0.7, 3500);
		  if(red_storage_check)
		  {
			  mode = RED_FIND_BALL;
		  }
		  break;

	  case RED_FIND_BALL:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

		  findAndTakeBall(redBall);
//		  if(sensorMEGA[3] == 0)
//		  {
//			  mode = RED_FACING_SILO;
//		  }
		  if(sensorMEGA[3] == 0 && camera[2] == 1)
		  {
			  mode = RED_FACING_SILO;
		  }
		  else if(sensorMEGA[3] == 0 && camera[2] == 0)
		  {
			  throwTheBall(red_throwBall, 1.5, 0.0, 0.0, 1.5);
		  }
		  break;

	  case RED_FACING_SILO:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

		  servo_write(120);
		  PID_EG(red_silo, 1.8, 0.0, 0.0, 1.5, 0.8, 3000);
		  if(red_silo_check)
		  {
			  mode = RED_FIND_SILO;
		  }
		  break;

	  case RED_FIND_SILO:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

		  placeBallInSilo(red_silo, 1.5, 0.0, 0.0, 1.5, 0.7, 2500);
		  if((FL_distance > 0 && FL_distance <= 10) || (FR_distance > 0 && FR_distance <= 10))
		  {
			  setMotorSpeed(1, -2000);
			  setMotorSpeed(2, -2000);
			  setMotorSpeed(7, -2800);
			  HAL_Delay(3000);
			  mode = RED_STORAGE;
		  }
		  break;

	  case RED_RETRY:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

		  PID_moveToCoordinate(red_retry, red_retry_parameters, red_retry_numPoints);
		  if(red_retry_check)
		  {
			  mode = RED_RETRY_STORAGE;
		  }
		  break;

	  case RED_RETRY_STORAGE:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

		  setMotorSpeed(1, 0);
		  setMotorSpeed(2, 0);
		  setMotorSpeed(7, 0);
		  PID_EG(red_retry_storage, 1.8, 0.0, 0.0, 1.5, 0.7, 3500);
		  if(red_retry_storage_check)
		  {
			  mode = RED_RETRY_FIND_BALL;
		  }
		  break;

	  case RED_RETRY_FIND_BALL:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

		  findAndTakeBall(redRetryBall);
//		  if(sensorMEGA[3] == 0)
//		  {
//			  mode = RED_RETRY_FACING_SILO;
//		  }
		  if(sensorMEGA[3] == 0 && camera[2] == 1)
		  {
			  mode = RED_RETRY_FACING_SILO;
		  }
		  else if(sensorMEGA[3] == 0 && camera[2] == 0)
		  {
			  throwTheBall(red_retry_throwBall, 1.5, 0.0, 0.0, 1.5);
		  }
		  break;

	  case RED_RETRY_FACING_SILO:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

		  servo_write(120);
		  PID_EG(red_retry_silo, 1.8, 0.0, 0.0, 1.5, 0.8, 3000);
		  if(red_retry_silo_check)
		  {
			  mode = RED_RETRY_FIND_SILO;
		  }
		  break;

	  case RED_RETRY_FIND_SILO:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

		  placeBallInSilo(red_retry_silo, 1.5, 0.0, 0.0, 1.5, 0.7, 2500);
		  if((FL_distance > 0 && FL_distance <= 10) || (FR_distance > 0 && FR_distance <= 10))
		  {
			  setMotorSpeed(1, -2000);
			  setMotorSpeed(2, -2000);
			  setMotorSpeed(7, -2800);
			  HAL_Delay(3000);
			  mode = RED_RETRY_STORAGE;
		  }
		  break;

	  case BLUE_STEP:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

		  PID_moveToCoordinate(blue_step, blue_step_parameters, blue_step_numPoints);
		  if(blue_step_check)
		  {
			  mode = BLUE_STORAGE;
		  }
		  break;

	  case BLUE_STORAGE:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

		  setMotorSpeed(1, 0);
		  setMotorSpeed(2, 0);
		  setMotorSpeed(7, 0);
		  PID_EG(blue_storage, 1.8, 0.0, 0.0, 1.5, 0.7, 3500);
		  if(blue_storage_check)
		  {
			  mode = BLUE_FIND_BALL;
		  }
		  break;

	  case BLUE_FIND_BALL:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

		  findAndTakeBall(blueBall);
//		  if(sensorMEGA[3] == 0)
//		  {
//			  mode = BLUE_FACING_SILO;
//		  }
		  if(sensorMEGA[3] == 0 && camera[2] == 1)
		  {
			  mode = BLUE_FACING_SILO;
		  }
		  else if(sensorMEGA[3] == 0 && camera[2] == 0)
		  {
			  throwTheBall(blue_throwBall, 1.5, 0.0, 0.0, 1.5);
		  }
		  break;

	  case BLUE_FACING_SILO:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

		  servo_write(120);
		  PID_EG(blue_silo, 1.8, 0.0, 0.0, 1.5, 0.8, 3000);
		  if(blue_silo_check)
		  {
			  mode = BLUE_FIND_SILO;
		  }
		  break;

	  case BLUE_FIND_SILO:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

		  placeBallInSilo(blue_silo, 1.5, 0.0, 0.0, 1.5, 0.7, 2500);
		  if((FL_distance > 0 && FL_distance <= 10) || (FR_distance > 0 && FR_distance <= 10))
		  {
			  setMotorSpeed(1, -2000);
			  setMotorSpeed(2, -2000);
			  setMotorSpeed(7, -2800);
			  HAL_Delay(3000);
			  mode = BLUE_STORAGE;
		  }
		  break;

	  case BLUE_RETRY:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

		  PID_moveToCoordinate(blue_retry, blue_retry_parameters, blue_retry_numPoints);
		  if(blue_retry_check)
		  {
			  mode = BLUE_RETRY_STORAGE;
		  }
		  break;

	  case BLUE_RETRY_STORAGE:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

		  setMotorSpeed(1, 0);
		  setMotorSpeed(2, 0);
		  setMotorSpeed(7, 0);
		  PID_EG(blue_retry_storage, 1.8, 0.0, 0.0, 1.5, 0.7, 3500);
		  if(blue_retry_storage_check)
		  {
			  mode = BLUE_RETRY_FIND_BALL;
		  }
		  break;

	  case BLUE_RETRY_FIND_BALL:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

		  findAndTakeBall(blueRetryBall);
//		  if(sensorMEGA[3] == 0)
//		  {
//			  mode = BLUE_RETRY_FACING_SILO;
//		  }
		  if(sensorMEGA[3] == 0 && camera[2] == 1)
		  {
			  mode = BLUE_RETRY_FACING_SILO;
		  }
		  else if(sensorMEGA[3] == 0 && camera[2] == 0)
		  {
			  throwTheBall(blue_retry_throwBall, 1.5, 0.0, 0.0, 1.5);
		  }
		  break;

	  case BLUE_RETRY_FACING_SILO:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

		  servo_write(120);
		  PID_EG(blue_retry_silo, 1.8, 0.0, 0.0, 1.5, 0.8, 3000);
		  if(blue_retry_silo_check)
		  {
			  mode = BLUE_RETRY_FIND_SILO;
		  }
		  break;

	  case BLUE_RETRY_FIND_SILO:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

		  placeBallInSilo(blue_retry_silo, 1.5, 0.0, 0.0, 1.5, 0.7, 2500);
		  if((FL_distance > 0 && FL_distance <= 10) || (FR_distance > 0 && FR_distance <= 10))
		  {
			  setMotorSpeed(1, -2000);
			  setMotorSpeed(2, -2000);
			  setMotorSpeed(7, -2800);
			  HAL_Delay(3000);
			  mode = BLUE_RETRY_STORAGE;
		  }
		  break;

	  case TES:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

		  placeBallInSilo(tes, 1.5, 0.0, 0.0, 1.5, 0.7, 2500);
		  if((FL_distance > 0 && FL_distance <= 10) || (FR_distance > 0 && FR_distance <= 10))
		  {
			  setMotorSpeed(1, -2000);
			  setMotorSpeed(2, -2000);
			  setMotorSpeed(7, -2800);
			  HAL_Delay(3000);
			  mode = VOID;
		  }
		  break;

	  case TES2:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
		  break;
	  default:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

//		  trying(2500, 0, 0, 0.0, 3.0);
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
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : Button_1_Pin */
  GPIO_InitStruct.Pin = Button_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Button_1_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pins : Button_2_Pin Button_3_Pin */
  GPIO_InitStruct.Pin = Button_2_Pin|Button_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_4_Pin */
  GPIO_InitStruct.Pin = Button_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Button_4_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pins : PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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

void displayMode()
{
	lcd_set_cursor(0, 0);
	switch (mode)
	{
		case RED_STEP:
			lcd_write_string("R STEP");
			break;
		case RED_STORAGE:
			lcd_write_string("R STORAGE");
			break;
		case RED_FIND_BALL:
			lcd_write_string("R FIND BALL");
			break;
		case RED_FACING_SILO:
			lcd_write_string("R FACING SILO");
			break;
		case RED_FIND_SILO:
			lcd_write_string("R FIND SILO");
			break;
		case RED_RETRY:
			lcd_write_string("R R");
			break;
		case RED_RETRY_STORAGE:
			lcd_write_string("R R STORAGE");
			break;
		case RED_RETRY_FIND_BALL:
			lcd_write_string("R R FIND BALL");
			break;
		case RED_RETRY_FACING_SILO:
			lcd_write_string("R R FACING SILO");
			break;
		case RED_RETRY_FIND_SILO:
			lcd_write_string("R R FIND SILO");
			break;
		case BLUE_STEP:
			lcd_write_string("B STEP");
			break;
		case BLUE_STORAGE:
			lcd_write_string("B STORAGE");
			break;
		case BLUE_FIND_BALL:
			lcd_write_string("B FIND BALL");
			break;
		case BLUE_FACING_SILO:
			lcd_write_string("B FACING SILO");
			break;
		case BLUE_FIND_SILO:
			lcd_write_string("B FIND SILO");
			break;
		case BLUE_RETRY:
			lcd_write_string("B R");
			break;
		case BLUE_RETRY_STORAGE:
			lcd_write_string("B R STORAGE");
			break;
		case BLUE_RETRY_FIND_BALL:
			lcd_write_string("B R FIND BALL");
			break;
		case BLUE_RETRY_FACING_SILO:
			lcd_write_string("B R FACING SILO");
			break;
		case BLUE_RETRY_FIND_SILO:
			lcd_write_string("B R FIND SILO");
			break;
		case TES:
			lcd_write_string("TES");
			break;
		case TES2:
			lcd_write_string("TES2");
			break;
		default:
			lcd_write_string("VOID");
			break;
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
