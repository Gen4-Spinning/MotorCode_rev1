/*
 * Ramp.h
 *
 *  Created on: Feb 26, 2023
 *      Author: harsha
 */

#ifndef RAMP_H_
#define RAMP_H_

#include "stm32g4xx_hal.h"
#include "Struct.h"

#define RAMP_UP 0
#define RAMP_DOWN 1
#define RAMP_STEADY 2
#define RAMP_OVER 3
#define RAMP_WAIT 4
#define RAMP_CHANGE 8

typedef struct RampDutyStruct {
  uint16_t finalTargetDuty;
  uint16_t currentDuty;
  float currentDutyF;
  float dDuty_F_RU;
  float dDuty_F_RD;
  int16_t steadyRunTime_s;
  long rampUpTime_ms;
  long rampDownTime_ms;
  uint16_t ramp_callingTime_ms;
  uint8_t rampPhase;
  uint16_t transitionTarget;
  uint16_t transitionTime_ms;
  float dDuty_F_transition;
}RampDuty;

typedef struct RampRPMStruct {
  uint16_t finalTargetRPM ;
  float instTargetRPM_F;
  uint16_t instTargetRPM_s16M;
  float dRPM_F_RU;
  float dRPM_F_RD;
  int16_t steadyRunTime_s; // runTime is -1 for Forever (normal mode)
  long rampUpTime_ms;
  long rampDownTime_ms;
  uint16_t ramp_callingTime_ms;
  uint8_t rampPhase;
  uint16_t transitionTarget;
  uint16_t transitionTime_ms;
  float dDuty_F_transition;
}RampRPM;

extern RampRPM rampRPM;
extern RampDuty rampDuty;

void InitRampRPMStruct(RampRPM *ramp,uint16_t targetRPM,long rampUpTime,long rampDownTime,int16_t rampSteadyTime);
void StartRampRPM(RampRPM *ramp);
void StopRampRPM(RampRPM *ramp);
void StartRampDownRPM(RampRPM *ramp);
void ResetRampRPM(RampRPM *ramp);
void IdleRampRPM(RampRPM *ramp);
void ExecRampRPM(RampRPM *ramp,TimerTypeDef *t);
void ChangeRPM(RunMgmtTypeDef *rm, RampRPM *r);
void Recalculate_RampRPM_RampRates(RunMgmtTypeDef *rm, RampRPM *ramp,uint16_t newTarget);


void InitRampDutyStruct(RampDuty *ramp,uint16_t targetDuty,long rampUpTime,long rampDownTime,int16_t rampSteadyTime);
void StartRampDuty(RampDuty *ramp);
void StartRampDownDuty(RampDuty *ramp);
void StopRampDuty(RampDuty *ramp);
void ResetRampDuty(RampDuty *ramp);
void ExecRampDuty(RampDuty *ramp,TimerTypeDef *t);
void ChangeDuty(RunMgmtTypeDef *rm, RampDuty *r);
void Recalculate_RampDuty_RampRates(RunMgmtTypeDef *rm, RampDuty *ramp,uint16_t newTarget);


#endif /* RAMP_H_ */
