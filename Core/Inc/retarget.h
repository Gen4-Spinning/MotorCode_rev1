/*
 * retarget.h
 *
 *  Created on: 09-Mar-2023
 *      Author: Jonathan
 */

#ifndef INC_RETARGET_H_
#define INC_RETARGET_H_


#include "main.h"
#include <sys/stat.h>

void RetargetInit(UART_HandleTypeDef *huart);
int _isatty(int fd);
int _write(int fd, char* ptr, int len);
int _close(int fd);
int _lseek(int fd, int ptr, int dir);
int _read(int fd, char* ptr, int len);
int _fstat(int fd, struct stat* st);




#endif /* INC_RETARGET_H_ */
