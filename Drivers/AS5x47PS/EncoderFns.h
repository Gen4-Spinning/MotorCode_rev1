/*
 * EncoderFns.h
 *
 *  Created on: 22-Mar-2023
 *      Author: harsha
 */

#ifndef ENCODERFNS_H_
#define ENCODERFNS_H_

#include "AS5x47P.h"

#define SPI_RDNG_TO_MECH_ANGLE 0.0220 // (360/16384)
#define SPI_RDNG_TO_ENC_CNT  0.125 // (2048/16384 = 0.122)
#define ENC_CNT_TO_MECH_ANGLE 0.1758  // (360/2048)

void SetupABIwithoutPWM(void);
uint8_t Check_ABI_SetCorrectly(Settings1 settings1, Settings2 settings2);
uint8_t setupMotorEncoder_inABI_Mode(void);
uint8_t updateEncoderZeroPosition(uint16_t zeroValue);
uint16_t getEncoderStartPosition(void);
float getEncoderAngleFromABI(TIM_HandleTypeDef *htim);
float getEncoderAngleFromSPI(uint8_t continuousRead);
uint8_t AS5047_checkEncoderHealth(void);
uint8_t AS5047_EnableMagErrors(void);
#endif /* ENCODERFNS_H_ */
