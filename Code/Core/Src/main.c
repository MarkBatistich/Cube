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
#include <stdio.h>
#include "fixmath.h"
//#include <math.h>
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

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}

void resetICM()
{
	uint8_t tx[2] = {W | DEVICE_CONFIG, 0x01};
	HAL_GPIO_WritePin (CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, tx, 2, 1000);
	HAL_GPIO_WritePin (CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
}

void spiSet(uint8_t addr, uint8_t val)
{
	uint8_t tx[2] = {0};
	tx[0] = W | addr;
	tx[1] = val;
	HAL_GPIO_WritePin (CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, tx, 2, SPI_WAIT);
	HAL_GPIO_WritePin (CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

void spiGet(uint8_t addr, uint8_t* rx, uint8_t num)
{
	uint8_t tx = R | addr;
	HAL_GPIO_WritePin (CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &tx, 1, SPI_WAIT);
	HAL_SPI_Receive(&hspi1, rx, num, SPI_WAIT);
	HAL_GPIO_WritePin (CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

void printf16(fix16_t num, int decimals){
	decimals = decimals > 5 ? 5 : decimals;
	char str[14] = {'0'};
	fix16_to_str(num, str, decimals);
	printf("%s", str);
}

void pn(){
	printf("\r\n");
}

void ps(){
	printf(" ");
}

void pc(){
	printf(", ");
}

void brakeOn() {
	HAL_GPIO_WritePin(BRAKE_GPIO_Port, BRAKE_Pin, GPIO_PIN_RESET);
	return;
}

void brakeOff() {
	HAL_GPIO_WritePin(BRAKE_GPIO_Port, BRAKE_Pin, GPIO_PIN_SET);
	return;
}

void waitUntilStop(){
	TIM3->CCR1 = 100;
	brakeOn();
	int val1 = 0;
	int val2 = 1;
	while (val1 != val2){
		val1 = TIM1->CNT;
		HAL_Delay (10);
		val2 = TIM1->CNT;
	}
	brakeOff();
}

fix16_t getBatteryVoltage() {
	HAL_ADC_Start(&hadc1);
	uint32_t raw;
	fix16_t voltage;
	HAL_ADC_PollForConversion(&hadc1, 1000);
	raw = HAL_ADC_GetValue(&hadc1);
	voltage = fix16_mul(fix16_from_int(raw), fix16_from_float(3.3/4095*0.9647)); // Vdd / 2^16 * fudgeFactor
	voltage = fix16_mul(voltage, fix16_from_float(11.07)); // resistor ratio
	voltage = fix16_sub(voltage, fix16_from_float(0.1)); // more fudge factor
	HAL_ADC_Stop(&hadc1);
	return voltage;
}

fix16_t rad2deg(fix16_t num){
	num = fix16_mul(num, fix16_from_int(180));
	num = fix16_div(num, fix16_pi);
	return num;
}

//returns 1 if the number is negative, 0 if positive or 0
int32_t sign(int32_t num){
	return (num < 0);
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
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  //Start timers and ADC
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

  uint8_t rx[6] = { 0 };
  resetICM();
  spiSet(ACCEL_CONFIG0, 0x06);
  spiSet(GYRO_CONFIG0, 0x06);
  spiSet(PWR_MGMT0, 0x0F);
  HAL_Delay(45);

//  fix16_t P = fix16_from_float(32767.999985);
//  fix16_t Q = fix16_atan2(140351, 65536);
//  P = fix16_div(P,Q);
//  Q = fix16_mul(Q, fix16_from_int(2));
//  P = fix16_sqrt(P);

  printf("start\r\n");
  pn();


  //variable definitions
  int16_t pos1 = 0, pos2 = 0, diff = 0;
  fix16_t angle = 0, prevAngle = 0, gyrAngle = 0, accAngle = 0;
  uint32_t start = 0, end = 0;
  uint8_t dir = 1;
  uint32_t pulse = TIM3->CCR1;
  int32_t control = 0, controlSignal = 0;
  int16_t a_i[3] = {0};
  fix16_t a_f[3] = {0};
  int16_t g_i[3] = {0};
  fix16_t g_f[3] = {0};
  fix16_t error = 0, errorInt = 0, errorDiff = 0;
  fix16_t velocity = 0;
  fix16_t controlVelocity = 0;
  fix16_t velocityError = 0, velocityErrorInt = 0, velocityErrorDiff = 0;
  fix16_t prevVelocityError = 0;
  uint8_t firstRun = 1;
  fix16_t angleControl, velocityControl = 0;


  const uint32_t dt = 10;
  const fix16_t tau = fix16_from_int(1);
  const fix16_t alpha = fix16_div(tau, fix16_add(tau, fix16_div(fix16_from_int(dt), fix16_from_int(1000))));
  const fix16_t accFactor = fix16_div(fix16_from_int(16), fix16_from_int(32768));
  const fix16_t gyrFactor = fix16_div(fix16_from_int(2000), fix16_from_int(32768));
  const fix16_t Kp = fix16_from_float(80.00);
  const fix16_t Ki = fix16_from_float(3.000);
  const fix16_t Kd = fix16_from_float(200.00);
  const fix16_t Kp_s = fix16_from_float(10.00);
  const fix16_t Ki_s = fix16_from_float(0.00);
  const fix16_t Kd_s = fix16_from_float(0.00);

  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
  waitUntilStop();
  TIM3->CCR1 = 100;

  spiGet(WHO_AM_I, rx, 1);
  printf("WHO_AM_I: %d\r\n", rx[0]);
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  start = HAL_GetTick();

	  pos1 = pos2;
	  pos2 = TIM1 -> CNT;
	  diff = pos2 - pos1;
	  if (diff < 0) diff += 65536;

	  spiGet(ACCEL_DATA_X1, rx, 6);

	  a_i[0] = (rx[0] << 8) | rx[1];
	  a_i[1] = (rx[2] << 8) | rx[3];
	  a_i[2] = (rx[4] << 8) | rx[5];

	  a_f[0] = fix16_mul(fix16_from_int(a_i[0]), accFactor);
	  a_f[1] = fix16_mul(fix16_from_int(a_i[1]), accFactor);
	  a_f[2] = fix16_mul(fix16_from_int(a_i[2]), accFactor);

	  spiGet(GYRO_DATA_X1, rx, 6);

	  g_i[0] = (rx[0] << 8) | rx[1];
	  g_i[1] = (rx[2] << 8) | rx[3];
	  g_i[2] = (rx[4] << 8) | rx[5];

	  g_f[0] = fix16_mul(fix16_from_int(g_i[0]), gyrFactor);
	  g_f[1] = fix16_mul(fix16_from_int(g_i[1]), gyrFactor);
	  g_f[2] = fix16_mul(fix16_from_int(g_i[2]), gyrFactor);

	  prevAngle = angle;
	  accAngle = fix16_atan2(-a_f[0], -a_f[1]);
	  accAngle = rad2deg(accAngle);
	  accAngle = fix16_sub(accAngle, fix16_from_float(1.5));

	  if (firstRun){
		  gyrAngle = accAngle;
		  angle = accAngle;
	  }

	  gyrAngle = fix16_add(angle, fix16_mul(-g_f[2], fix16_div(fix16_from_int(dt), fix16_from_int(1000))));

	  angle = fix16_add(fix16_mul(alpha, gyrAngle), fix16_mul(fix16_sub(fix16_one, alpha), accAngle));


	  //HAL_GPIO_TogglePin (LED_GPIO_Port, LED_Pin);

	  error = angle;
	  errorDiff = fix16_sub(angle, prevAngle);
	  errorInt += error;
	  prevAngle = angle;

	  velocity = fix16_div(fix16_from_int(diff), fix16_from_int(200));
	  velocity = fix16_mul(velocity, fix16_from_int(1000/dt));

	  velocityError = velocity;
	  velocityErrorDiff = fix16_sub(velocityError, prevVelocityError);
	  velocityErrorInt += velocityError;
	  prevVelocityError = velocityError;



//	  error = angle;
//	  errorDiff = fix16_sub(angle, prevAngle);
//	  errorInt += error;
//	  control = fix16_to_int(fix16_add(fix16_add(fix16_mul(Kp, error), fix16_mul(Ki, errorInt)), fix16_mul(Kd, errorDiff)));
//
//	  control = MAX(MIN(control, 100), -100);
//	  //Ensure motor is stopped before switching directions
////	  if((sign(control) != sign(diff)) && diff != 0) {
////		  waitUntilStop();
////		  dir = !sign(control);
////		  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, dir);
////	  }
//
//	  controlSpeed = fix16_mul(fix16_from_int(control), fix16_from_float(36 / 100.0));
//	  prevvelocityError = velocityError;
//	  velocityError = fix16_sub(controlSpeed, speed);
//	  velocityErrorDiff = fix16_sub(velocityError, prevvelocityError);
//	  velocityErrorInt += velocityError;
//	  controlSignal = fix16_add(fix16_add(fix16_mul(Kp_s, velocityError), fix16_mul(Ki_s, velocityErrorInt)), fix16_mul(Kd_s, velocityErrorDiff));
//
//	  controlSignal = fix16_mul(controlSignal, fix16_from_float(100 / 36.0));
//	  controlSignal = MAX(MIN(controlSignal, fix16_from_int(100)), fix16_from_int(-100));
//	  controlSignal = fix16_to_int(controlSignal);

	  angleControl = fix16_add(fix16_add(fix16_mul(Kp, error), fix16_mul(Ki, errorInt)), fix16_mul(Kd, errorDiff));
	  velocityControl = fix16_add(fix16_add(fix16_mul(Kp_s, velocityError), fix16_mul(Ki_s, velocityErrorInt)), fix16_mul(Kd_s, velocityErrorDiff));

	  control = fix16_add(angleControl, -velocityControl);

	  control = MAX(MIN(control, fix16_from_int(100)), fix16_from_int(-100));
	  controlSignal = fix16_to_int(control);

	  controlVelocity = fix16_mul(control, fix16_from_float(100 / 36.0));

	  if (fix16_sub(fix16_abs(controlVelocity), fix16_abs(velocity)) > 0 && ((sign(controlSignal) == sign(velocity)) || (fix16_abs(velocity) < fix16_from_int(10)))) {
		  pulse = 100 - (controlSignal * (1-2*(controlSignal<0)));
		  brakeOff();
	  } else {
		  pulse = 100;
		  brakeOn();
	  }

	  if (fix16_abs(velocity) < fix16_from_int(10)) {
		  dir = !sign(controlSignal);
		  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, dir);
	  }

	  if (fix16_abs(angle) > fix16_from_int(25)) {
		  pulse = 100;
		  errorInt = 0;
	  }

	  TIM3->CCR1 = pulse;

//	  printf("%d %d %d ", control, pulse, controlSignal);
//	  printf16(speed,2);
//	  ps();
//	  printf16(controlSpeed, 2);
//	  pn();


	  //TIM3->CCR1 = 80;
	  printf("%d, ", HAL_GetTick());
	  printf16(angleControl, 2);
	  pc();
	  printf16(velocityControl, 2);
	  pc();
	  printf16(control, 2);
	  pc();
	  printf16(angle, 2);
	  pc();
	  printf("%d, ", pulse);
	  pn();

	  end = HAL_GetTick();
	  while (end-start < dt){
		  HAL_Delay(0);
		  end = HAL_GetTick();
	  }

	  firstRun = 0;
	  //printf("Tick Time: %d\n", end-start);
	  //pn();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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
  htim3.Init.Prescaler = 8-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BRAKE_Pin|DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BRAKE_Pin DIR_Pin */
  GPIO_InitStruct.Pin = BRAKE_Pin|DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin CS_Pin */
  GPIO_InitStruct.Pin = LED_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

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

