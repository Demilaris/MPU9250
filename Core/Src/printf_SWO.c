/*
 * printf_SWO.c
 *
 *  Created on: 23 мар. 2021 г.
 *      Author: micro
 */

#include "printf_SWO.h"

char test_msg[300];

int _write(int file, char *ptr, int len)
{
    //return usart_write(platform_get_console(), (u8 *)ptr, len);
      int i=0;
      for(i=0 ; i<len ; i++)
        ITM_SendChar((*ptr++));
      return len;
}


