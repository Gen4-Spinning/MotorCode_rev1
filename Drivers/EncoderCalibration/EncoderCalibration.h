/*
 * EncoderCalibration.h
 *
 *  Created on: Feb 18, 2023
 *      Author: harsha
 */

#ifndef ENCODERCALIBRATION_H_
#define ENCODERCALIBRATION_H_

#include "stdio.h"
#include "sixSector.h"

#define EMPIRICAL_OFFSET 1748
#define OFFSET_IF_WRONG_DIRECTION 1638
#define ENC_COUNTS_60_ELEC_DEGREES 546 //calculated as 16384/5/6

typedef struct EncoderCalibrationStruct{
	uint16_t calib_pwmVal;
	uint16_t calib_timeVal;
	uint8_t pv_Axis;
	uint16_t first_reading;
	uint16_t previousAngle;
	uint16_t angleReading[30];
	int16_t dAngle[30];
	int16_t deltaError[30];
	int16_t minError;
	int16_t maxError;
	int16_t avgError;
	int16_t encIndex;
	int16_t encIndex_wOffset;
}EncCalib_TypeDef;

void setupCalibration(uint8_t dutyPercentage,uint16_t delayTime_ms);
void setPhaseU(uint16_t pwmVal);
void setPhaseV(uint16_t pwmVal);
void setPhaseW(uint16_t pwmVal);
void TurnOffAllPhases(void);
void voltageOnPrincipalAxis(EncCalib_TypeDef *encC,uint8_t axis);
void InitializeEncoderCalib_TypeDef(EncCalib_TypeDef *encC);
void RunCalibration(void);
uint16_t RunCalibrationWithPrintf(void);

extern EncCalib_TypeDef encCalib;

#endif /* ENCODERCALIBRATION_H_ */
