/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "app_subghz_phy.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32wlxx_hal_gpio_ex.h"
#include "stm32wlxx_hal_i2c.h"
#include "subghz_phy_app.h"
#include "radio.h"
#include "sys_app.h"
#include "utilities_conf.h"
#include "gps.h"
#include <stdint.h>
#include <string.h>
#include <sys/_intsup.h>
#include "usart_if.h"
#include "altimeter.h"
#include "softuart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
  OFF,
  PRE_FLIGHT,
  ARMED,
  IN_FLIGHT,
  RECOVERY
}deviceState_t;

typedef struct __attribute__((packed)){

  char  header[3];
  uint8_t  utc_hour;
  uint8_t  utc_minute;
  uint8_t  utc_seconds;

  int32_t  lat;            
  uint8_t  lat_ns;         
  int32_t  lon;            
  uint8_t  lon_ew;    
  int32_t  altitude;  

  uint8_t  status; 
  uint8_t  fix_quality; 
  uint8_t  num_sats; 
   
  uint32_t baro_alt;
  uint16_t sensor_status;
  deviceState_t  rocket_state;
  uint16_t batt_v;

}telem_packet_t;  
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_lpuart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

RTC_HandleTypeDef hrtc;

SUBGHZ_HandleTypeDef hsubghz;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* Definitions for Conops_Task */
osThreadId_t Conops_TaskHandle;
const osThreadAttr_t Conops_Task_attributes = {
  .name = "Conops_Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for GPS_Task */
osThreadId_t GPS_TaskHandle;
const osThreadAttr_t GPS_Task_attributes = {
  .name = "GPS_Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};
/* Definitions for Altimeter_Task */
osThreadId_t Altimeter_TaskHandle;
const osThreadAttr_t Altimeter_Task_attributes = {
  .name = "Altimeter_Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};
/* Definitions for FC_Task */
osThreadId_t FC_TaskHandle;
const osThreadAttr_t FC_Task_attributes = {
  .name = "FC_Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};
/* Definitions for Telemetry */
osThreadId_t TelemetryHandle;
const osThreadAttr_t Telemetry_attributes = {
  .name = "Telemetry",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};
/* Definitions for GPS_Queue */
osMessageQueueId_t GPS_QueueHandle;
const osMessageQueueAttr_t GPS_Queue_attributes = {
  .name = "GPS_Queue"
};
/* Definitions for Alt_Queue */
osMessageQueueId_t Alt_QueueHandle;
const osMessageQueueAttr_t Alt_Queue_attributes = {
  .name = "Alt_Queue"
};
/* Definitions for FC_Queue */
osMessageQueueId_t FC_QueueHandle;
const osMessageQueueAttr_t FC_Queue_attributes = {
  .name = "FC_Queue"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
void Conops_Process(void *argument);
void GPS_Process(void *argument);
void Altimeter_Process(void *argument);
void FC_Process(void *argument);
void Telem_process(void *argument);

/* USER CODE BEGIN PFP */
void header_art();
void check_debug_pins();
uint8_t getchar_v2(uint8_t SoftUartNumber);
static void setState(deviceState_t new_state);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Variables for GPS
#define NMEASIZE 85                        // max NMEA sentence length
extern uint8_t dma_rx_buf[];               // circular DMA landing buffer (defined in usart_if.c)
static uint8_t gps_buffer[NMEASIZE + 1];   // assembles one NMEA sentence
static volatile uint8_t gps_ready_flg;     // set when a full sentence is ready
gps_t gps_data;                            // holds the gps data

deviceState_t state; 

uint16_t batt_mV;
uint16_t batt_voltage_lookup[] = {3500, 3560, 3620, 3680, 3740, 3800, 3860, 3920, 3980, 4040, 4100};
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
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_LPUART1_UART_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  
  
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of GPS_Queue */
  GPS_QueueHandle = osMessageQueueNew (10, sizeof(gps_t), &GPS_Queue_attributes);

  /* creation of Alt_Queue */
  Alt_QueueHandle = osMessageQueueNew (10, sizeof(alt_sensor_t), &Alt_Queue_attributes);

  /* creation of FC_Queue */
  FC_QueueHandle = osMessageQueueNew (10, sizeof(uint16_t), &FC_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Conops_Task */
  Conops_TaskHandle = osThreadNew(Conops_Process, NULL, &Conops_Task_attributes);

  /* creation of GPS_Task */
  GPS_TaskHandle = osThreadNew(GPS_Process, NULL, &GPS_Task_attributes);

  /* creation of Altimeter_Task */
  Altimeter_TaskHandle = osThreadNew(Altimeter_Process, NULL, &Altimeter_Task_attributes);

  /* creation of FC_Task */
  FC_TaskHandle = osThreadNew(FC_Process, NULL, &FC_Task_attributes);

  /* creation of Telemetry */
  TelemetryHandle = osThreadNew(Telem_process, NULL, &Telemetry_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV1;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = ENABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_3CYCLES_5;
  hadc.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_3CYCLES_5;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c2.Init.Timing = 0x10805D88;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 9600;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV2;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
void MX_RTC_Init(void)
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
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SUBGHZ Initialization Function
  * @param None
  * @retval None
  */
void MX_SUBGHZ_Init(void)
{

  /* USER CODE BEGIN SUBGHZ_Init 0 */

  /* USER CODE END SUBGHZ_Init 0 */

  /* USER CODE BEGIN SUBGHZ_Init 1 */

  /* USER CODE END SUBGHZ_Init 1 */
  hsubghz.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_8;
  if (HAL_SUBGHZ_Init(&hsubghz) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SUBGHZ_Init 2 */

  /* USER CODE END SUBGHZ_Init 2 */

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
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  /* TIM1 @ 48kHz drives the software UART bit engine (5x the 9600 baud rate) */
  HAL_TIM_Base_Start_IT(&htim1);
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 41999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 59999 ;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FC_TX_GPIO_Port, FC_TX_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : FC_RX_Pin */
  GPIO_InitStruct.Pin = FC_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FC_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FC_TX_Pin */
  GPIO_InitStruct.Pin = FC_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(FC_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USR_BTN_Pin */
  GPIO_InitStruct.Pin = USR_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(USR_BTN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// User Button Interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

  if(GPIO_Pin == USR_BTN_Pin) {
    APP_LOG(TS_OFF, VLEVEL_ALWAYS, "Button Pressed, changing pins to debug mode. \r\n");

    // Reconfigure PA13 (SWDIO) and PA14 (SWCLK) from GPIO back to SWD debug
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_13 | GPIO_PIN_14;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_SWD;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    check_debug_pins();
  }

}


void header_art(){
  APP_LOG(TS_OFF, VLEVEL_L, "\r\n");
  APP_LOG(TS_OFF, VLEVEL_L, "  _   _ __  *  __ ____    \r\n");
  APP_LOG(TS_OFF, VLEVEL_L, " | | | |\\ \\   / /|  _ \\   \r\n");
  APP_LOG(TS_OFF, VLEVEL_L, " | | | | \\ \\ / / | |_) |  \r\n");
  APP_LOG(TS_OFF, VLEVEL_L, " | |_| |  \\ V /  |  _ <   \r\n");
  APP_LOG(TS_OFF, VLEVEL_L, "  \\___/ +  \\_/ + |_| \\_\\  \r\n");
  APP_LOG(TS_OFF, VLEVEL_L, "\r\n");
  APP_LOG(TS_OFF, VLEVEL_L, "------ANDURIL 3---------\r\n");
  APP_LOG(TS_OFF, VLEVEL_L, "----Sending Pebble------\r\n");
  APP_LOG(TS_OFF, VLEVEL_L, "LORA_FREQUENCY:  %d MHz\n\r", RF_FREQUENCY/1000000);
  APP_LOG(TS_OFF, VLEVEL_L, "LORA_BW:         %d kHz\n\r", (1 << LORA_BANDWIDTH) * 125);
  APP_LOG(TS_OFF, VLEVEL_L, "LORA_SF:         %d\n\r", LORA_SPREADING_FACTOR);
  APP_LOG(TS_OFF, VLEVEL_L, "CALL_SIGN:       %s\n\r", CALL_SIGN);
  APP_LOG(TS_OFF, VLEVEL_L, "------------------------\n\r");

  // Report current config of PA13 (SWDIO) / PA14 (SWCLK)
  check_debug_pins();
  APP_LOG(TS_OFF, VLEVEL_L, "------------------------\n\r");

  APP_LOG(TS_OFF, VLEVEL_L, "\r\n");
  APP_LOG(TS_OFF, VLEVEL_L, "\r\n");

}

void check_debug_pins(){

  // Report current config of PA13 (SWDIO) / PA14 (SWCLK)
  static const char *mode_str[4] = { "RX", "TX", "DEBUG", "ANALOG" };
  for (uint32_t pin = 13; pin <= 14; pin++) {
    uint32_t mode = (GPIOA->MODER >> (pin * 2)) & 0x3;
    APP_LOG(TS_OFF, VLEVEL_L, "PA%lu: %s\r\n", pin, mode_str[mode]);
  }
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_Conops_Process */

static void setState(deviceState_t new_state){
  state =  new_state;
  switch(new_state){
    case OFF:
    // power off if battery is to low
    //state it wake up in
      break;
    case PRE_FLIGHT:
      //commands to send (frequency change, and ARM)
      // maybe should also be in low power mode except radio
      break;
    case ARMED:
      // send arm command to flight computer.
      // wait for gps lock before going into in_flight
      // start timer in case flight computer fails to 
      break;
    case IN_FLIGHT:
    //  normal operation
    //  flight computer detect recovery
    // waht happens if we lose lock?
      break;
    case RECOVERY:
    // turn off bmp, fc communication, reduce transmission rates, keep gps on and battery on
    // turn off flash
      break;
    default:
      break;
  }
}

/**
  * @brief  Function implementing the Conops_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Conops_Process */
void Conops_Process(void *argument)
{
  /* init code for SubGHz_Phy */
  MX_SubGHz_Phy_Init();
  /* USER CODE BEGIN 5 */
  header_art();
  state = OFF;
  deviceState_t next_state = PRE_FLIGHT;
  
  /* Infinite loop */
  for(;;)
  {
    APP_LOG(TS_OFF, VLEVEL_ALWAYS, "Changing states...\r\n");
    if(next_state != state) setState(next_state);
    osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_GPS_Process */
/**
* @brief Function implementing the GPS_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GPS_Process */
void GPS_Process(void *argument)
{
  /* USER CODE BEGIN GPS_Process */

  gps_config(); 

  uint16_t rd_pos   = 0;   // our read index into the circular DMA buffer
  uint16_t rx_index = 0;   // position within the sentence being assembled
  /* Infinite loop */
  for(;;)
  {
    //APP_LOG(TS_OFF, VLEVEL_ALWAYS, "GPS Process\r\n");
    // write index = how far the DMA has filled (NDTR counts down from SIZE)
    uint16_t wr_pos = DMA_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_lpuart1_rx);

    while (rd_pos != wr_pos)
    {
      uint8_t c = dma_rx_buf[rd_pos];
      rd_pos = (rd_pos + 1) % DMA_RX_BUF_SIZE;

      if (c == '$') {                                   // start of sentence
        rx_index = 0;
        gps_buffer[rx_index++] = c;
      } else if (c == '\n' || c == '\r') {              // end of sentence
        if (rx_index != 0) {
          gps_buffer[rx_index] = '\0';
          gps_ready_flg = 1;
        }
        rx_index = 0;
      } else if (rx_index > 0 && rx_index < NMEASIZE) { // middle (only inside a sentence)
        gps_buffer[rx_index++] = c;
      } else {
        rx_index = 0;                                   // junk before '$' / overflow -resync
      }
    }

    if (gps_ready_flg)
    {
      // gps_parser() is destructive (writes '\0' over commas), so hand it a
      // copy — the original stays intact for the async trace to finish sending.
      static uint8_t gps_parse_buf[NMEASIZE + 1];
      memcpy(gps_parse_buf, gps_buffer, sizeof(gps_parse_buf));

      //APP_LOG(TS_OFF, VLEVEL_L, "%s\r\n", gps_buffer);
      gps_parser(&gps_data, gps_parse_buf);
  
      //send data through queue
      osMessageQueuePut(GPS_QueueHandle,&gps_data,0,osWaitForever);

      gps_ready_flg = 0;
    }

    osDelay(500);
  }
  /* USER CODE END GPS_Process */
}

/* USER CODE BEGIN Header_Altimeter_Process */
/**
* @brief Function implementing the Altimeter_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Altimeter_Process */
void Altimeter_Process(void *argument)
{
  /* USER CODE BEGIN Altimeter_Process */
  uint8_t err;
  alt_sensor_t altimeter;
  
  err = alt_config(&altimeter);
  if(err != ALT_I2C_OK){APP_LOG(TS_OFF, VLEVEL_ALWAYS, "Altimeter config error: %d\r\n", err);}
  else {APP_LOG(TS_OFF, VLEVEL_ALWAYS, "Altimeter configured successfully.\r\n");}



  /* Infinite loop */
  for(;;)
  {
    err = alt_get_data(&altimeter);   // returns ALT_I2C_BUSY until a conversion is ready
    if(err == ALT_I2C_OK){
      osMessageQueuePut(Alt_QueueHandle,&altimeter,0,osWaitForever);
    }
    osDelay(500);
  }
  /* USER CODE END Altimeter_Process */
}

/* USER CODE BEGIN Header_FC_Process */

/**
* @brief Function implementing the FC_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FC_Process */
void FC_Process(void *argument)
{
  /* USER CODE BEGIN FC_Process */
  static char line[64];
  static uint8_t line_len = 0;
  uint8_t buf[64];
  uint8_t len;
  
  SoftUartInit(0,FC_TX_GPIO_Port,FC_TX_Pin,FC_RX_GPIO_Port, FC_RX_Pin);
  SoftUartEnableRx(0);
  
  // for now
  uint16_t sensor_status = 0xFFFF;

  /* Infinite loop */
  for(;;)
  {
    len = SoftUartRxAlavailable(0);
    if (len > 0) {
        if (SoftUartReadRxBuffer(0, buf, len) == SoftUart_OK) {
          for (uint8_t i = 0; i < len; i++) {
            uint8_t c = buf[i];
            if (c == '\r' || c == '\n') {
                line[line_len] = '\0';

                if (strcmp(line, "ARM") == 0) {
                    SoftUartWaitUntilTxComplate(0);
                    SoftUartPuts(0, (uint8_t*)"\r\nARM command received\r\n", 24);
                } else {
                    SoftUartWaitUntilTxComplate(0);
                    SoftUartPuts(0, (uint8_t*)"\r\n", 2);
                }
                line_len = 0;
            } else if (line_len < sizeof(line) - 1) {
                line[line_len++] = c;
                SoftUartWaitUntilTxComplate(0);   // let the previous byte finish
                SoftUartPuts(0, &c, 1);           // echo
            }
          }

        }
    }
    osMessageQueuePut(FC_QueueHandle, &sensor_status, 0, 500);
    osDelay(500); //changed for now
  }
  /* USER CODE END FC_Process */
}

/* USER CODE BEGIN Header_Telem_process */

//adc callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{

    batt_mV = (uint16_t)HAL_ADC_GetValue(hadc)*2;
    APP_LOG(TS_OFF, VLEVEL_ALWAYS, "Batt_mv: %u \r\n",batt_mV);
    
}

/**
* @brief Function implementing the telemetry thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Telem_process */
void Telem_process(void *argument)
{
  /* USER CODE BEGIN Telem_process */

  // adc init
  // receive gps struct queues
  // receive alt struct queues
  // reveive flight computer queue
  // grab battery value
  telem_packet_t payload;
  memset(&payload, 0, sizeof(payload));
  memcpy(payload.header, (uint8_t *)"$SP", 3);


  gps_t gps;
  alt_sensor_t alt;
  uint16_t fc_data;
  uint8_t err = 0;
  osStatus_t status;
  //start adc
  HAL_ADCEx_Calibration_Start(&hadc); 
  HAL_ADC_Start_IT(&hadc);
  HAL_TIM_Base_Start(&htim2);

  /* Infinite loop */
  for(;;)
  {
    payload.rocket_state = state;
    payload.batt_v = batt_mV;
    status = osMessageQueueGet(GPS_QueueHandle, &gps, NULL, osWaitForever);
    if(status == osOK){
      payload.utc_hour    = gps.hour;
      payload.utc_minute  = gps.minute;
      payload.utc_seconds = gps.second;

      payload.lat         = gps.lat;
      payload.lat_ns      = gps.lat_ns;
      payload.lon         = gps.lon;
      payload.lon_ew      = gps.lon_ew;
      payload.altitude    = gps.altitude;

      payload.status      = gps.status;
      payload.fix_quality = gps.fix_quality;
      payload.num_sats    = gps.num_sats;
    }else{
      APP_LOG(TS_OFF, VLEVEL_ALWAYS, "Error GPS Queue: %u \r\n", status);
    }

    status = osMessageQueueGet(Alt_QueueHandle, &alt, NULL, osWaitForever);
    if(status == osOK){
      //check for own status
      if((alt.i2c_status != ALT_I2C_OK) || (alt.errors != 0)){
        //APP_LOG(TS_OFF, VLEVEL_ALWAYS, "ALT Error: %u \r\n",alt.errors);
        
      }
      else{
        payload.baro_alt  = alt.pressure;
      }
    }else{
      APP_LOG(TS_OFF, VLEVEL_ALWAYS, "Error Alt Queue: %u \r\n", status);
    }

    status = osMessageQueueGet(FC_QueueHandle, &fc_data, NULL, osWaitForever);
    if(status == osOK){
      payload.sensor_status = fc_data;
    }else{
      APP_LOG(TS_OFF, VLEVEL_ALWAYS, "Error FC Queue: %u \r\n", status);
    }

      // dump the raw payload packet as hex
    char line[3 * sizeof(payload) + 1];
    uint8_t *p = (uint8_t *)&payload;
    for (uint32_t i = 0; i < sizeof(payload); i++)
    snprintf(&line[i * 3],4, "%02X ", p[i]);
    APP_LOG(TS_OFF, VLEVEL_ALWAYS, "PKT[%u]: %s\r\n", (unsigned)sizeof(payload), line);
    err = Radio.Send((uint8_t*)&payload,sizeof(payload));
    if(err != 0){
      APP_LOG(TS_OFF, VLEVEL_ALWAYS, "radio send error: %u \r\n", err);
    }
    osThreadYield();
  }
  /* USER CODE END Telem_process */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /* TIM1 update ISR clocks the software UART (kept here so regen won't drop it) */
  if (htim->Instance == TIM1)
  {
    SoftUartHandler();
  }
  /* USER CODE END Callback 1 */
}

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
#ifdef USE_FULL_ASSERT
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
