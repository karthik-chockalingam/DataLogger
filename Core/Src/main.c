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
#include "cmsis_os.h"
#include "fatfs.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "string.h"
#include "fatfs_sd.h"
#include "lwip/opt.h"
#include "lwip/api.h"
#include "lwip/sys.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BIT_0	(1 << 0)
#define BIT_1	(1 << 1)
#define BIT_2	(1 << 2)
#define BIT_3	(1 << 3)
#define BIT_4	(1 << 4)
#define BIT_5	(1 << 5)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
RTC_HandleTypeDef hrtc;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;
osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
static struct netconn *conn, *newconn;
static struct netbuf *buf;

typedef struct
{
	uint8_t setAnalog;
	uint8_t setDigital;
	uint8_t setSamplerate;
	uint8_t setOperation;
}logConfig;

char user_config[4];

TaskHandle_t configTaskHandle;
TaskHandle_t loggerTaskHandle;

QueueHandle_t configQueueHandle = NULL;
EventGroupHandle_t eventhandleLog;

char logMessage[100];

volatile RTC_DateTypeDef currDate;
volatile RTC_TimeTypeDef currTime;

FATFS fs;
FIL fil;
FILINFO filinfo;
FRESULT fresult;
UINT readcount, writecount;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void tcpserver_init (void);
static void tcpTask(void *arg);

static void configTask(void *arg);
static void loggerTask(void *arg);

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

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  TIM1->CCR1 = 2500;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  BaseType_t status;
  status = xTaskCreate(configTask, "config-Task", 512, NULL, osPriorityNormal, &configTaskHandle);
  configASSERT(status == pdPASS);

  status = xTaskCreate(loggerTask, "logger-Task", 512, NULL, osPriorityNormal, &loggerTaskHandle);
  configASSERT(status == pdPASS);

  configQueueHandle = xQueueCreate(1, sizeof(logConfig));
  if(configQueueHandle == NULL)
  {
	  printf("Queue creation failed...\n");
  }

  eventhandleLog = xEventGroupCreate();
  if(eventhandleLog == NULL)
  {
	  printf("Event Group creation failed...\n");
  }

  /* USER CODE END RTOS_THREADS */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x18;
  sTime.Minutes = 0x30;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
  sDate.Month = RTC_MONTH_AUGUST;
  sDate.Date = 0x29;
  sDate.Year = 0x23;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 10800-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 5000-1;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PG1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**** Send RESPONSE every time the client sends some data ******/

static void tcpTask(void *arg)
{
	err_t err, accept_err, recv_error;
	logConfig clientConfig;
	/* Create a new connection identifier. */
	conn = netconn_new(NETCONN_TCP);
	printf("Entering into tcpTask...\n");

	if (conn!=NULL)
	{
		/* Bind connection to the port number 10022. */
		err = netconn_bind(conn, IP_ADDR_ANY, 10022);

		if (err == ERR_OK)
		{
			/* Tell connection to go into listening mode. */
			netconn_listen(conn);

			while (1)
			{
				/* Grab new connection. */
				accept_err = netconn_accept(conn, &newconn);

				/* Process the new connection. */
				if (accept_err == ERR_OK)
				{

					/* receive the data from the client */
					while (netconn_recv(newconn, &buf) == ERR_OK)
					{
						memset (user_config, '\0', 4);  // clear the buffer
						strncpy (user_config, buf->p->payload, 4);   // get the message from the client

						clientConfig.setAnalog = (uint8_t)user_config[0];
						clientConfig.setDigital = (uint8_t)user_config[1];
						clientConfig.setSamplerate = (uint8_t)user_config[2];
						clientConfig.setOperation = (uint8_t)user_config[3];

						BaseType_t status;
						status = xQueueSend(configQueueHandle, (void*) &clientConfig, 0);
						if(status == pdPASS)
						{
							printf("Queuing config ...\n");
						}
						else
						{
							printf("Queuing config error...\n");
						}
						//netconn_write(newconn, smsg, len, NETCONN_COPY);

						netbuf_delete(buf);
					}

					/* Close connection and discard connection identifier. */
					netconn_close(newconn);
					netconn_delete(newconn);
				}
			}
		}
		else
		{
			netconn_delete(conn);
		}
	}
}


void tcpserver_init(void)
{
  sys_thread_new("tcp-task", tcpTask, NULL, 1024, osPriorityNormal);
}


static void configTask(void *arg)
{
	logConfig userConfig;
	EventBits_t logEvents = 0x0000;
	EventBits_t statusEvents = 0x0000;

	while(1)
	{
		memset(&userConfig, '\0', sizeof(logConfig));
		if(xQueueReceive(configQueueHandle, &(userConfig), portMAX_DELAY) == pdPASS)
		{
			printf("Dequeuing config message -- config task ..\n");
			printf("Analog - %d\n", userConfig.setAnalog);
			printf("Digital -%d\n", userConfig.setDigital);
			printf("Sample Rate - %d\n", userConfig.setSamplerate);
			printf("Operation - %d\n", userConfig.setOperation);

			logEvents = 0x0000;

			// Set bit 0 - Start logging
			if(userConfig.setOperation == 2)
			{
				logEvents |= BIT_0;
				// set bit 1 - log Analog
				if(userConfig.setAnalog == 2)
				{

					logEvents |= BIT_1;
				}

				// set bit 2 - log Digital
				if(userConfig.setDigital == 2)
				{
					logEvents |= BIT_2;
				}

				// set bit 3 & 4 - log samplerate; 01 - 1samples/sec, 10 - 5samples/sec, 11 - 10samples/sec
				if(userConfig.setSamplerate == 1)
				{
					logEvents |= BIT_3;
				}
				else if(userConfig.setSamplerate == 2)
				{
					logEvents |= BIT_4;
				}
				else if(userConfig.setSamplerate == 3)
				{
					logEvents |= (BIT_3 | BIT_4);
				}
			}
			// Set bit 5 - Stop logging
			else if(userConfig.setOperation == 1)
			{
				logEvents |= BIT_5;
			}

			printf("logEvents is %x\n", logEvents);
			statusEvents = xEventGroupSetBits(eventhandleLog, logEvents);
			printf("statusEvents is %x\n", statusEvents);
		}
	}
}

static void loggerTask(void *arg)
{
	TickType_t lastwakeTime;

	uint32_t adcValue;
	uint32_t analogValue=0;
	uint8_t gpioValue=0;
	TickType_t tickdelayms = 0;
	TickType_t tickdelay = 0;

	EventBits_t eventLog;
	EventBits_t reteventLog;

	char filename[20];

	const EventBits_t bitMask = (BIT_0 | BIT_5);

	int len = 0;
	HAL_StatusTypeDef status;

	while(1)
	{
		printf("logger Task ...\n");
		eventLog = xEventGroupWaitBits(eventhandleLog, bitMask, pdFALSE, pdFALSE, portMAX_DELAY);

		if((eventLog & (BIT_0 | BIT_5)) == BIT_0)
		{
			if((eventLog & (BIT_3 | BIT_4)) == BIT_3)
			{
				tickdelayms = 1000;
			}
			else if ((eventLog & (BIT_3 | BIT_4)) == BIT_4)
			{
				tickdelayms = 200;
			}
			else if ((eventLog & (BIT_3 | BIT_4)) == (BIT_3 | BIT_4))
			{
				tickdelayms = 100;
			}

			// File System Mount
			fresult = f_mount(&fs, "/", 1);
			if(fresult != FR_OK)
			{
				printf("SD Card mount Error...\n");
			}
			else
			{
				printf("SD card mounted...\n");
			}

			status = HAL_RTC_GetTime(&hrtc, &currTime, RTC_FORMAT_BIN);
			if(status != HAL_OK)
			{
				printf("Get Time Error...\n");
			}

			//Call HAL_RTC_GetDate after HAL_RTC_GetTime
			status = HAL_RTC_GetDate(&hrtc, &currDate, RTC_FORMAT_BIN);
			if(status != HAL_OK)
			{
				printf("Get Date Error...\n");
			}

			len = sprintf((char*)filename, "%02d-%02d_%02d-%02d-%02d.txt", currDate.Date, currDate.Month, currTime.Hours, currTime.Minutes, currTime.Seconds);
			printf("File name is %s\n", filename);
			fresult = f_open(&fil, filename, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
			if(fresult != FR_OK)
			{
				printf("File creation failed...\n");
			}
			else
			{
				printf("File created successfully...\n");
			}
		}

		while((eventLog & (BIT_0 | BIT_5)) == BIT_0)
		{
			lastwakeTime = xTaskGetTickCount();

			status = HAL_RTC_GetTime(&hrtc, &currTime, RTC_FORMAT_BIN);
			if(status != HAL_OK)
			{
				printf("Get Time Error...\n");
			}

			//Call HAL_RTC_GetDate after HAL_RTC_GetTime
			status = HAL_RTC_GetDate(&hrtc, &currDate, RTC_FORMAT_BIN);
			if(status != HAL_OK)
			{
				printf("Get Date Error...\n");
			}

			len = sprintf((char*)logMessage, "%02d-%02d-%04d    %02d:%02d:%02d    ", currDate.Date, currDate.Month, 2000+currDate.Year,
																								currTime.Hours, currTime.Minutes, currTime.Seconds);

			if((eventLog & BIT_1) == BIT_1)
			{
				HAL_ADC_Start(&hadc1);
				HAL_ADC_PollForConversion(&hadc1, 10);
				adcValue = HAL_ADC_GetValue(&hadc1);
				analogValue = (adcValue * 3300) / 4095;
				HAL_ADC_Stop(&hadc1);

				len = sprintf((char*)logMessage+strlen(logMessage), "Analog-%d    ", analogValue);
			}

			if((eventLog & BIT_2) == BIT_2)
			{
				gpioValue = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_1);
				len = sprintf((char*)logMessage+strlen(logMessage), "Digital-%d", gpioValue);
			}

			len = sprintf((char*)logMessage+strlen(logMessage), "\n");
			netconn_write(newconn, logMessage, strlen(logMessage), NETCONN_COPY);
			fresult = f_write(&fil, logMessage, strlen(logMessage), &writecount);
			printf("The write count and strlen of log message is %d and %d", writecount, strlen(logMessage));
			printf("%s\n", logMessage);
			memset (logMessage, '\0', 100);

			tickdelay = pdMS_TO_TICKS(tickdelayms);
			vTaskDelayUntil(&lastwakeTime, tickdelay);

			eventLog = xEventGroupWaitBits(eventhandleLog, bitMask, pdFALSE, pdFALSE, portMAX_DELAY);
			if ((eventLog & (BIT_0 | BIT_5)) == (BIT_0 | BIT_5))
			{
				f_close(&fil);
				fresult = f_mount(NULL, "/", 1);
				if(fresult == FR_OK)
				{
					printf("SD card unmounted successfully...\n");
				}
				break;
			}
		}

		printf("The event log after exiting while is %x\n", eventLog);

		reteventLog = xEventGroupClearBits(eventhandleLog, 0xFF);
		eventLog = xEventGroupGetBits(eventhandleLog);
		printf("The event log after clearing is %x\n", eventLog);
		if (eventLog == 0x00)
		{
			printf(" All event bits are cleared..\n");
		}
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
  tcpserver_init();
  /* Infinite loop */
  for(;;)
  {
	printf("Default Task..\n");
    osDelay(60000);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
