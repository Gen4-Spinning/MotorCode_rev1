/*
 * CAN_Motor.c
 *
 *  Created on: 10-Mar-2023
 *      Author: harsha
 */

#include "CAN_Motor.h"
#include "stdio.h"
extern char UART_buffer[50];
extern UART_HandleTypeDef huart3;

void FDCAN_runtimedataFromMotor(void)
{
	TxHeader.Identifier =(0xE0901<<8)|S.CAN_ID;//set to transmit runtime data frame from flyer to motherboard
	TxHeader.DataLength = FDCAN_DLC_BYTES_12;

	TxData[0]=(R.targetRPM)>>8;
	TxData[1]=R.targetRPM;
	TxData[2]=(R.presentRPM)>>8;
	TxData[3]=R.presentRPM;
	TxData[4]=(R.appliedDuty)>>8;
	TxData[5]=R.appliedDuty;
	TxData[6]=R.FETtemp;
	TxData[7]=R.MOTtemp;
	TxData[8]=(R.busCurrentADC)>>8;
	TxData[9]=R.busCurrentADC;
	TxData[10]=(R.busVoltageADC)>>8;
	TxData[11]=R.busVoltageADC;

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void FDCAN_analysisdataFromMotor(void)
{
	TxHeader.Identifier =(0xA0801<<8)|S.CAN_ID;
	TxHeader.DataLength = FDCAN_DLC_BYTES_12;

	TxData[0]=(R.targetRPM)>>8;
	TxData[1]=R.targetRPM;
	TxData[2]=(R.presentRPM)>>8;
	TxData[3]=R.presentRPM;
	TxData[4]=(R.appliedDuty)>>8;
	TxData[5]=R.appliedDuty;
	TxData[6]=(R.proportionalTerm)>>8;
	TxData[7]=R.proportionalTerm;
	TxData[8]=(R.IntegralTerm)>>8;
	TxData[9]=R.IntegralTerm;
	TxData[10]=(R.feedforwardTerm)>>8;
	TxData[11]=R.feedforwardTerm;

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void FDCAN_errorFromMotor(void)
{
	TxHeader.Identifier =(0x60201<<8)|S.CAN_ID;
	TxHeader.DataLength = FDCAN_DLC_BYTES_2;
	TxData[0]=(R.motorError)>>8;
	TxData[1]=(R.motorError);
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void FDCAN_parseForMotor(uint8_t my_address)
{

	functionID=((RxHeader.Identifier)&0xFF0000)>>16;
	source_address=(RxHeader.Identifier)&0xFF;

	switch (functionID) {

		case MOTORSTATE_FUNCTIONID:
			S.CAN_MSG=RxData[0];
			FDCAN_ACKresponseFromMotor(my_address);
			//sprintf(UART_buffer,"\r\n CAN-Received MotorState-%02d",RxData[0]);
			//HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,29);
			break;

		case DIAGNOSTICSDATA_FUNCTIONID:
			FDCAN_ACKresponseFromMotor(my_address);
			RM.runType = DIAGNOSIS_RUN;
			FDCAN_parseDiagnosticData(&RM);
			S.RM_state = RECEIVED_RAMP_SETTINGS;
			//sprintf(UART_buffer,"\r\n CAN-Received Diagnostics Setup");
			//HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,35);
			break;

		case FULLRUN_SETUP_FUNCTIONID:
			FDCAN_ACKresponseFromMotor(my_address);
			RM.runType = NORMAL_RUN;
			FDCAN_parseFullRunSetupData(&RM);
			S.RM_state = RECEIVED_RAMP_SETTINGS;
			//sprintf(UART_buffer,"\r\n CAN-Full Run Setup");
			//HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,32);
			break;

		case CHANGETARGET_FUNCTIONID:
			FDCAN_ACKresponseFromMotor(my_address);
			S.CAN_MSG = CHANGE_RPM;
			S.RM_state = RECEIVED_CHANGE_RPM_SETTINGS;
			FDCAN_parseChangeTargetFrame(&RM);
			//sprintf(UART_buffer,"\r\n CAN-Received Change Target");
			//HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,29);
			break;

		default:
			break;
	}
}


void FDCAN_driveresponseFromMotor(uint8_t source)
{
	TxHeader.Identifier =(0xA0401<<8)|source;
	TxHeader.DataLength = FDCAN_DLC_BYTES_1;
	TxData[0]=source;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void FDCAN_ACKresponseFromMotor(uint8_t source)
{
	TxHeader.Identifier =(0x060F01<<8)|source;
	TxHeader.DataLength = FDCAN_DLC_BYTES_1;
	TxData[0]=1;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void FDCAN_parseFullRunSetupData(RunMgmtTypeDef *r){
	r->rampRampUpTime_s = RxData[0];
	r->rampRampDownTime_s = RxData[1];
	r->rampTarget = (RxData[2]<<8)|(RxData[3]);
	r->controlType = CLOSED_LOOP;
	r->rampSteadyRunTime_s = RUN_FOREVER;
	r->rampFunction = RAMP_RPM;
	r->rotateDirection = sV.default_direction;
	InitRampRPMStruct(&rampRPM,r->rampTarget,r->rampRampUpTime_s*1000,r->rampRampDownTime_s*1000,r->rampSteadyRunTime_s);
}


void FDCAN_parseDiagnosticData(RunMgmtTypeDef *r)
{	uint8_t temp = 0;
	r->controlType=RxData[0];
	r->rampTarget = (RxData[1]<<8)|(RxData[2]);
	r->rampRampUpTime_s = RxData[3];
	r->rampRampDownTime_s = RxData[4];
	r->rampSteadyRunTime_s = (RxData[5]<<8)|(RxData[6]);

	// on the app we show to default dir or reverse dir, and not CW/CCW.
	temp = RxData[7];
	if (temp == DEFAULT_DIR_COMMAND){
		r->rotateDirection = sV.default_direction;
	}else{
		r->rotateDirection = !sV.default_direction;
	}

	if (r->controlType == CLOSED_LOOP){
		r->rampFunction = RAMP_RPM;
		//fill up the Ramp RPM Struct- TargetRPRm,rampUp Time, rampDownTime, and steadY state runTime
		InitRampRPMStruct(&rampRPM,r->rampTarget,r->rampRampUpTime_s*1000,r->rampRampDownTime_s*1000,r->rampSteadyRunTime_s);
		//Later we send StepRPM through Diagnosis

	}else if (r->controlType == OPEN_LOOP){
		r->rampFunction = RAMP_DUTY;
		//fill up the Ramp Duty Struct- TargetRPRm,rampUp Time, rampDownTime, and steadY state runTime
		InitRampDutyStruct(&rampDuty,r->rampTarget,r->rampRampUpTime_s*1000,r->rampRampUpTime_s*1000,r->rampSteadyRunTime_s);
		//Later we send StepDuty through Diagnosis
	}else{

	}
}

void FDCAN_parseChangeTargetFrame(RunMgmtTypeDef *r){
	r->transitionTarget = (RxData[0]<<8|RxData[1]);
	r->transitionTime_ms = (RxData[2]<<8|RxData[3]);
}


void FDCAN_sendDiagDoneFrame(void){
	TxHeader.Identifier =(0xA1401<<8)|S.CAN_ID;
	TxHeader.DataLength = FDCAN_DLC_BYTES_1;
	TxData[0]=1;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}
