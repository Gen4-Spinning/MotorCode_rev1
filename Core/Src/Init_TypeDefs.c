/*
 * Init_TypeDefs.c
 *
 *  Created on: 10-Mar-2023
 *      Author: harsha
 */

#include "Struct.h"
#include "Constants.h"



void InitializeTimer_TypeDef(TimerTypeDef *t){
	t->tim16_20msTimer=0;
	t->tim16_oneSecTimer=0;
}


void InitializeRunMgmt_TypeDef(RunMgmtTypeDef *rm)
{
	rm->runType=NOT_RUNNING;
	rm->rampFunction = 0;
	rm->controlType = 0;
	rm->logReturn = NO_LOG;
	rm->rampTarget = 0;
	rm->rampSteadyRunTime_s = 0; // in sec, after rampUp
	rm->rampRampUpTime_s=0;
	rm->rampRampDownTime_s=0;
	rm-> transitionTarget = 0;
	rm-> transitionTime_ms = 0;
	rm-> rotateDirection = CW; // CW by default.For Normal running this is unused For Diag it get set with the Frame
}

void InitializeSettingsObj(settingVar *stV)
{
	  stV->Kp=1.01;
	  stV->Ki=1.58;
	  stV->start_offset=1560;
	  stV->ff_percent=0.25;
	  stV->MOTID=2;
	  stV->AMS_offset_index=1590;

}

void InitializeSetup_TypeDef(setup_typeDef *s){
	s->eepromPWMValsGood = 0;
	s->eepromMotorValsGood = 0;
	s->defaults_eepromWriteFailed=0;
	s->encoderSetupOK=0;
	s->encoderMagErrorSetupOK = 0;
	s->encoderZeroValueOK = 0;
}


void InitializeRunTime_TypeDef(runtimeVarsTypeDef *runtimeVarObj)
{
	runtimeVarObj->motor_state = IDLE_STATE;
	runtimeVarObj->runmode=0;
	runtimeVarObj->targetRPM=0;
	runtimeVarObj->presentRPM=0;
	runtimeVarObj->appliedDuty=0;
	runtimeVarObj->FETtemp=0;
	runtimeVarObj->MOTtemp=0;
	runtimeVarObj->busCurrentADC=0;
	runtimeVarObj->busVoltageADC=0;
	runtimeVarObj->currentAmps = 0;
	runtimeVarObj->voltageVolts =0;
}

void InitializeState_TypeDef(StateTypeDef *s){
	s->motorState = 0;
	s->motorSetupFailed = 0;
	s->errorMsgSentOnce = 0 ;
	s->CAN_MSG = 0;
	s->oneTime = 1;
	s->CAN_ID = 0;
	s->RM_state = 0;
}

