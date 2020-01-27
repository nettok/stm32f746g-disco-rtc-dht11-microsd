/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
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

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_SDMMC1_SD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct {
	uint8_t integralHumidity;
	uint8_t decimalHumidity;
	uint8_t integralTemp;
	uint8_t decimalTemp;
} ClimateEvent;

ClimateEvent climateEvent = {0};

RTC_TimeTypeDef time = {0};
RTC_DateTypeDef date = {0};

FRESULT fRes;    				/* FatFs function common result code */
FATFS SDFatFS;  				/* File system object for SD card logical drive */
FIL MyFile;     				/* File object */
char SDPath[4]; 				/* SD card logical drive path */
char filename[15];				/* YYYYMMDDHH.CSV */
uint8_t writeBuffer[50];
UINT byteswritten;				/* File write counts */

uint8_t persistEvent;			/* Flag to indicate that the event should be persisted */

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	HAL_RTC_GetTime(hrtc, &time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(hrtc, &date, RTC_FORMAT_BIN);

	persistEvent = 1;
}

// DHT11 Code

GPIO_InitTypeDef GPIO_DHT11_InitStruct = {0};

void setDHT11Mode(uint32_t mode) {
  GPIO_DHT11_InitStruct.Pin = DHT11_Pin;
  GPIO_DHT11_InitStruct.Mode = mode;
  GPIO_DHT11_InitStruct.Pull = GPIO_NOPULL;
  GPIO_DHT11_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_DHT11_InitStruct);
}

void setDHT11ToOutputMode() {
  setDHT11Mode(GPIO_MODE_OUTPUT_PP);
}

void setDHT11ToInputMode() {
  setDHT11Mode(GPIO_MODE_INPUT);
}

void setDHT11Pin(GPIO_PinState pinState) {
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, pinState);
}

GPIO_PinState getDHT11Pin() {
  return HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin);
}

uint32_t waitForDHT11PullUp() {
  uint32_t downCycles = 0;
  if (getDHT11Pin() == 1) {
    while (getDHT11Pin() == 1) { /* wait until down first */ }
  }
  while (getDHT11Pin() == 0) { // wait for pull up
    downCycles++;
  }
  return downCycles;
}

uint32_t waitForDHT11PullDown() {
  uint32_t upCycles = 0;
  if (getDHT11Pin() == 0) {
    while (getDHT11Pin() == 0) { /* wait until up first */ }
  }
  while (getDHT11Pin() == 1) { // wait for pull down
	  upCycles++;
  }
  return upCycles;
}

uint8_t readData(uint32_t thresholdCycles) {
  uint8_t i,j;
  for (j=0; j<8; j++) {
    waitForDHT11PullUp();
	uint32_t upCycles = waitForDHT11PullDown();
	if (upCycles > thresholdCycles) {
      i|= (1<<(7-j));	// bit is 1
	} else {
      i&= ~(1<<(7-j));  // bit is 0
	}
  }
  return i;
}

uint8_t getClimate() {
	uint8_t integralHumidityData;
	uint8_t decimalHumidityData;
	uint8_t integralTempData;
	uint8_t decimalTempData;
	uint8_t checkSum;

	// initial state for DHT11
	setDHT11ToInputMode();
	HAL_Delay(1000); // to pass unstable status of the DHT11 sensor

	// start communication
	setDHT11ToOutputMode();
	setDHT11Pin(0);
	HAL_Delay(18);
	setDHT11Pin(1);

	// wait for DHT11 response
	setDHT11ToInputMode();
	waitForDHT11PullUp();
	uint32_t upCycles = waitForDHT11PullDown();

	// upCycles of last pull-down is equivalent to 80 microseconds
	// Use this to interpolate data transmission bit responses knowing that:
	//   - bit 0: 26-28 microseconds
	//   - bit 1: 70 microseconds
	//
	uint32_t thresholdCycles = (upCycles / 2) + (10 * upCycles / 80); // ~50 microseconds

	// start sensor data transmission
	integralHumidityData = readData(thresholdCycles);
	decimalHumidityData = readData(thresholdCycles);
	integralTempData = readData(thresholdCycles);
	decimalTempData = readData(thresholdCycles);
	checkSum = readData(thresholdCycles);

	uint8_t dataSum  = integralHumidityData + decimalHumidityData + integralTempData + decimalTempData;

	if (checkSum != dataSum) {
	  return 0;
	}

	climateEvent.integralTemp = integralTempData;
	climateEvent.decimalTemp = decimalTempData;
	climateEvent.integralHumidity = integralHumidityData;
	climateEvent.decimalHumidity = decimalHumidityData;

	return 1;
}

// microSD code

void writeClimateEventToMicroSD()
{
	// See hack in: BSP_PlatformIsDetected
	fRes = f_mount(&SDFatFS, (TCHAR const*)SDPath, 0);
	if (fRes != FR_OK)
	{
		Error_Handler();
	}

	sprintf(filename, "%04d%02d%02d%02d.CSV", 2000 + date.Year, date.Month, date.Date, time.Hours);

	fRes = f_open(&MyFile, filename, FA_OPEN_APPEND|FA_WRITE);
	if (fRes != FR_OK)
	{
		Error_Handler();
	}

	int length = sprintf((char*)writeBuffer, "%04d-%02d-%02dT%02d:%02d:%02d,%d.%d°C,%d.%d%%\n",
			2000 + date.Year, date.Month, date.Date,
			time.Hours, time.Minutes, time.Seconds,
			climateEvent.integralTemp, climateEvent.decimalTemp,
			climateEvent.integralHumidity, climateEvent.decimalHumidity);

	f_write(&MyFile, writeBuffer, length, &byteswritten);

	f_close(&MyFile);

	f_mount(0, (TCHAR const*)SDPath, 0);
}

// main event handler

void doPersistClimateEvent() {
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
	if (getClimate()) {
		writeClimateEventToMicroSD();
	}
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
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
  MX_RTC_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	HAL_Delay(1000);

	if (persistEvent == 1)
	{
		persistEvent = 0;
		doPersistClimateEvent();
	}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_SDMMC1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{
  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == 0x777) {
	  // If clock has been already initialized, only set the alarm interrupt
	  HAL_RTC_GetAlarm(&hrtc, &sAlarm, RTC_ALARM_A, RTC_FORMAT_BIN);
	  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK) {
		  Error_Handler();
	  }
	  return;
  } else {
	  // Store already-initialized status code in a backup register
	  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x777);
  }

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 22;
  sTime.Minutes = 24;
  sTime.Seconds = 30;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SUNDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 26;
  sDate.Year = 20;
  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A 
  */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY|RTC_ALARMMASK_HOURS
                              |RTC_ALARMMASK_MINUTES;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

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
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PI8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT11_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

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
	while (1) {
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_Delay(500);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
