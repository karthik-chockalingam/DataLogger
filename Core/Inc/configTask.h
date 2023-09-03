/*
 * configTask.h
 *
 *  Created on: Sep 2, 2023
 *      Author: karthik
 */

#ifndef INC_CONFIGTASK_H_
#define INC_CONFIGTASK_H_

#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>

#define BIT_0	(1 << 0)
#define BIT_1	(1 << 1)
#define BIT_2	(1 << 2)
#define BIT_3	(1 << 3)
#define BIT_4	(1 << 4)
#define BIT_5	(1 << 5)

void configTask(void *arg);

#endif /* INC_CONFIGTASK_H_ */
