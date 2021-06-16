/*
 * printf_SWO.h
 *
 *  Created on: 23 мар. 2021 г.
 *      Author: micro
 */

#ifndef INC_PRINTF_SWO_H_
#define INC_PRINTF_SWO_H_

#include "main.h"
#include <stdio.h>
#include <string.h>

extern char test_msg[300];

int _write(int file, char *ptr, int len);


#endif /* INC_PRINTF_SWO_H_ */
