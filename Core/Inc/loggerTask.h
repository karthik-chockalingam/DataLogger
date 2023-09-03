/*
 * loggerTask.h
 *
 *  Created on: Sep 2, 2023
 *      Author: karthik
 */

#ifndef INC_LOGGERTASK_H_
#define INC_LOGGERTASK_H_

#include "cmsis_os.h"
#include "fatfs.h"
#include "fatfs_sd.h"
#include "lwip/api.h"
#include "lwip/sys.h"
#include <string.h>

#define BIT_0	(1 << 0)
#define BIT_1	(1 << 1)
#define BIT_2	(1 << 2)
#define BIT_3	(1 << 3)
#define BIT_4	(1 << 4)
#define BIT_5	(1 << 5)

void loggerTask(void *arg);
#endif /* INC_LOGGERTASK_H_ */
