/*
 * CAN_Motor.h
 *
 *  Created on: 10-Mar-2023
 *      Author: harsha
 */

#ifndef CAN_MOTOR_H_
#define CAN_MOTOR_H_


#include "Struct.h"
#include "Ramp.h"
#include "FDCAN.h"
#include "Constants.h"


void FDCAN_driveresponseFromMotor(uint8_t source);
void FDCAN_runtimedataFromMotor(void);
void FDCAN_errorFromMotor(void);
void FDCAN_analysisdataFromMotor(void);
void FDCAN_parseForMotor(uint8_t my_address);
void FDCAN_parseDiagnosticData(RunMgmtTypeDef *r);
void FDCAN_parseFullRunSetupData(RunMgmtTypeDef *r);
void FDCAN_parseChangeTargetFrame(RunMgmtTypeDef *r);
void FDCAN_ACKresponseFromMotor(uint8_t source);
void FDCAN_sendDiagDoneFrame(void);
#endif /* CAN_MOTOR_H_ */
