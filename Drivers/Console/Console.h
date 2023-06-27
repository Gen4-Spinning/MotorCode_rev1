/*
 * Console.h
 *
 *  Created on: Mar 16, 2023
 *      Author: harsha
 */

#ifndef CONSOLE_H_
#define CONSOLE_H_

#include <stdio.h>
#include "Struct.h"

#define BADCONSOLE_VAL -999
#define OL_TESTLOOP_MODE 1
#define OL_FREERUN_MODE 2

#define CONSOLE_CANID 1
#define CONSOLE_DIRECTION 2
#define CONSOLE_ENCODER_OFFSET 3

typedef struct ConsoleStruct {
	char runningOL;
	char OLmode;
	char OL_queryNo;

	char CL_queryNo;
	char runningCL;

	char logging;
	char zeroRPM_Reached;
	char consoleLoggingOn;
}console;

extern console C;


void configurationFromTerminal(void);
uint8_t printSettings(void);
uint8_t runMotorCalibrationRoutine(void);
uint8_t configureSettings(uint8_t setting);

uint8_t runMotorOpenLoop(void);
uint8_t runMotorClosedLoop(void);
void Console_OL_RAMPUP(void);
void Console_OL_RAMPDOWN(void);
void Console_OL_STOP(void);

void Console_CL_RAMPUP(void);
void Console_CL_RAMPDOWN(void);
void Console_CL_STOP(void);


#endif /* CONSOLE_H_ */
