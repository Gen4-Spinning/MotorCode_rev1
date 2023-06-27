/*
 * PID.h
 *
 *  Created on: Mar 11, 2023
 *      Author: harsha
 */

#ifndef PID_H_
#define PID_H_

#include "Struct.h"
#include "Ramp.h"
#include "EncSpeed.h"
#include "Constants.h"

typedef struct PID_struct{

	uint16_t FF_percent;
	float Kp_constant;
	float Ki_constant;
	float FF_constant;
	uint16_t startOffsetConstant;

	int16_t error_s16;
	float errorF;
	float Kp_term;
	float Ki_term;

	uint16_t feedForwardTerm;
	uint16_t startOffsetTerm;
	char antiWindup;
	int16_t pwm;

}PID_Typedef;


void InitializePID_TypeDef(PID_Typedef *p);
void ExecPID(PID_Typedef *p,RampRPM *r,EncSpeed_TypeDef *eS);
void setupPID(PID_Typedef *p,float Kp,float Ki,uint16_t FF_Percent,uint16_t so);
void resetPID(PID_Typedef *p);
extern PID_Typedef PID;


#endif /* PID_H_ */
