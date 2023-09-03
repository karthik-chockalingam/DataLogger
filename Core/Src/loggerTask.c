/*
 * loggerTask.c
 *
 *  Created on: Sep 2, 2023
 *      Author: karthik
 */
#include "loggerTask.h"

FATFS fs;
FIL fil;
FILINFO filinfo;
FRESULT fresult;
UINT readcount, writecount;

extern EventGroupHandle_t eventhandleLog;

RTC_DateTypeDef currDate;
RTC_TimeTypeDef currTime;
extern RTC_HandleTypeDef hrtc;
extern ADC_HandleTypeDef hadc1;

char logMessage[100];
extern struct netconn *client_conn;

void loggerTask(void *arg)
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

				len = sprintf((char*)logMessage+strlen(logMessage), "Analog-%ld    ", analogValue);
			}

			if((eventLog & BIT_2) == BIT_2)
			{
				gpioValue = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_1);
				len = sprintf((char*)logMessage+strlen(logMessage), "Digital-%d", gpioValue);
			}

			len = sprintf((char*)logMessage+strlen(logMessage), "\n");
			netconn_write(client_conn, logMessage, strlen(logMessage), NETCONN_COPY);
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

		printf("The event log after exiting while is %lx\n", (uint32_t)eventLog);

		reteventLog = xEventGroupClearBits(eventhandleLog, 0xFF);
		eventLog = xEventGroupGetBits(eventhandleLog);
		printf("The event log after clearing is %lx\n", (uint32_t)eventLog);
		if (eventLog == 0x00)
		{
			printf(" All event bits are cleared..\n");
		}
	}
}
