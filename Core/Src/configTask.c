/*
 * configTask.c
 *
 *  Created on: Sep 2, 2023
 *      Author: karthik
 */

#include "configTask.h"

typedef struct
{
	uint8_t setAnalog;
	uint8_t setDigital;
	uint8_t setSamplerate;
	uint8_t setOperation;
}logConfig;

extern QueueHandle_t configQueueHandle;
extern EventGroupHandle_t eventhandleLog;

void configTask(void *arg)
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

			printf("logEvents is %lx\n", (uint32_t)logEvents);
			statusEvents = xEventGroupSetBits(eventhandleLog, logEvents);
			printf("statusEvents is %lx\n", (uint32_t)statusEvents);
		}
	}
}

