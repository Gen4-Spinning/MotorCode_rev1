/*
 * EncSpeed.h
 *
 *  Created on: Mar 11, 2023
 *      Author: harsha
 */

#ifndef ENCSPEED_H_
#define ENCSPEED_H_

#include "stdio.h"
#include "sixSector.h"
#include "Constants.h"

//Constants are defined for a 16ms buffer. See Excel to change for other values.
#define SPEED_BUFFER_SIZE 16
#define CNTS_TO_RPM_CONSTANT 1.83
#define CNTS_TO_S16_CONSTANT 58.59

typedef struct EncSpeed{
	char EncSpeed_calcTime_ms;
	char  encDirection;
	uint16_t currentEncCount;
	uint16_t previousEncCount;
	int16_t deltaEncCount;
	uint16_t total_deltaEncCount;
	uint8_t bufferIdx;
	uint16_t speedArray[SPEED_BUFFER_SIZE];
	char speedArrayFilled;
	uint16_t speed_s16;
	float speedRPM;
	char zeroSpeed;
}EncSpeed_TypeDef;

void InitializeEncSpeed_TypeDef(EncSpeed_TypeDef *e);
void CalcEncSpeed(EncSpeed_TypeDef *e,sixSectorCntrl *s);

extern EncSpeed_TypeDef encSpeed;

#endif /* ENCSPEED_H_ */
