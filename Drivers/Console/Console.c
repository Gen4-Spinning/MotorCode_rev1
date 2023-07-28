/*
 * Console.c
 *
 *  Created on: Mar 16, 2023
 *      Author: harsha
 */


#include "Console.h"
#include "EepromFns.h"

#include "EncoderCalibration.h"
#include "EncoderFns.h"

#include "Constants.h"
#include "Ramp.h"

#include "main.h"
#include "stdio.h"

extern TIM_HandleTypeDef htim1;

void Console_OL_RAMPUP(void){
	RM.controlType = OPEN_LOOP;
	RM.rampFunction = RAMP_DUTY;
	//Fill up other RM in live watch
	//fill up the Ramp Duty Struct- TargetRPRm,rampUp Time, rampDownTime, and steadY state runTime
	InitRampDutyStruct(&rampDuty,RM.rampTarget,RM.rampRampUpTime_s*1000,RM.rampRampDownTime_s*1000,RM.rampSteadyRunTime_s);
	// we need to start the six sector Obj, and then start the Ramp
	StartSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj,RM.rotateDirection);
	StartRampDuty(&rampDuty);
	//need to reset the timer to allow for calculation of Time to take place
	T.tim16_oneSecTimer = 0;
	T.tim16_20msTimer = 0;
}
void Console_OL_RAMPDOWN(void){
	StartRampDownDuty(&rampDuty);
}

void Console_OL_STOP(void){
	StopSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj);
    StopRampDuty(&rampDuty);
}

void Console_CL_RAMPUP(void){
	RM.controlType = CLOSED_LOOP;
	RM.rampFunction = RAMP_RPM;
	//fill up the Ramp Duty Struct- TargetRPRm,rampUp Time, rampDownTime, and steadY state runTime
	InitRampRPMStruct(&rampRPM,RM.rampTarget,RM.rampRampUpTime_s*1000,RM.rampRampDownTime_s*1000,RM.rampSteadyRunTime_s);
	// we need to start the six sector Obj, and then start the Ramp
	StartSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj,RM.rotateDirection);
	StartRampRPM(&rampRPM);
	//need to reset the timer to allow for calculation of Time to take place
	T.tim16_oneSecTimer = 0;
	T.tim16_20msTimer = 0;
}

void Console_CL_RAMPDOWN(void){
	StartRampDownRPM(&rampRPM);
}

void Console_CL_STOP(void){
	 StopRampRPM(&rampRPM);
}

int waitForNoInput(void){
	int a;
	int rc;
	while ((rc = scanf("%d", &a)) == 0)
		{
			// clear what is left, the * means only match and discard:
			scanf("%*[^\n]");
			// input was not a number, ask again:
			printf("\r\n Not a number! Try Again ");
		}
	if (rc == EOF)
	{
		printf("\r\n No number found");
		return BADCONSOLE_VAL;
	}
	else
	{
		printf("\r\n You entered %d.", a);
		return a;
	}
	return BADCONSOLE_VAL;
}


uint8_t configurePIDSettings(void){
	int noEntered;
	uint8_t firstTime = 1;
	uint16_t minLimit=0;
	uint16_t maxLimit=0;
	uint8_t  pidQueryNo=0;
	uint8_t newPIDquery = 0;
	uint8_t insidePIDLoop =0;

	float Kp_new,Ki_new;
	uint8_t FF_factor_new,startOffsetNew;
	uint8_t eepromWrite,eepromWriteSuccess;
	while(1){
		if (firstTime == 1){
			printf("\r\n***MOTOR PID SETTINGS***");
			printf("\r\n Enter new PID parameter values in the order asked.To skip entering a value enter -1");
			printf("\r\n To start, press 1 or enter -1 to exit this menu");
			firstTime = 0;
			insidePIDLoop = 0;
		}

		noEntered = waitForNoInput();

		if (eepromWrite == 1){
			if (noEntered == 1){
				eepromWriteSuccess  = writePWMSettingsToEEPROM_Manual(Kp_new,Ki_new,FF_factor_new,startOffsetNew);
				if (eepromWriteSuccess){
					printf("\r\n Success! Written into Eeprom! Restart Motor for new values to take effect");
				}else{
					printf("\r\n Fail! Not Written into Eeprom!");
				}
				printf("\r\n ------------------");
				eepromWrite = 0;
				firstTime = 1;
			}else if (noEntered == 0){
				eepromWrite = 0;
				firstTime = 1;
			}
			noEntered = 0;
		}

		else if (insidePIDLoop == 0){
			if (noEntered == 1){
				printf("\r\n -----Kp----");
				printf("\r\n Current Kp value :%6.3f. Enter new Value x 1000 : (0-5000)",sV.Kp);
				minLimit = 0;
				maxLimit = 5000;
				pidQueryNo = 1;
				insidePIDLoop = 1;
			}

			if (noEntered == -1){
			  printf("\r\n");
			  break;
			}
		}else{
			if (noEntered == -1){
				if (pidQueryNo!=5){
					printf("\r\n Skipping parameter");
					if (pidQueryNo == 1){
						Kp_new = sV.Kp;
					}else if (pidQueryNo == 2){
						Ki_new = sV.Ki;
					}else if (pidQueryNo == 3){
						FF_factor_new = sV.ff_percent;
					}else if (pidQueryNo == 4){
						startOffsetNew = sV.start_offset;
					}
					newPIDquery = 1;
				}else{
					insidePIDLoop = 0;
					firstTime = 1;
				}
			}

			if (newPIDquery == 0){
				if ((noEntered <= maxLimit ) && ( noEntered >= minLimit)){
					if (pidQueryNo == 1){
						Kp_new = noEntered/1000.0;
						printf("\r\n Kp = %6.3f",Kp_new);
						newPIDquery = 1;
					}else if (pidQueryNo == 2){
						Ki_new = noEntered/1000.0;
						printf("\r\n Ki = %6.3f",Ki_new);
						newPIDquery = 1;
					}else if (pidQueryNo == 3){
						FF_factor_new = noEntered;
						printf("\r\n FF_factor = %02d",FF_factor_new);
						newPIDquery = 1;
					}else if (pidQueryNo == 4){
						startOffsetNew = noEntered;
						printf("\r\n startOffset = %03d",startOffsetNew);
						newPIDquery = 1;
					}else{}
					noEntered = 0;
				}else{
					printf("\r\n Outside Limits!Try again");
					noEntered = 0;
				}
			}

			if (newPIDquery){
				if(pidQueryNo == 1){
					printf("\r\n -----Ki-----");
					printf("\r\n Current Kp value :%6.3f. Enter new Value x 1000 : (0-5000)",sV.Ki);
					minLimit = 0;
					maxLimit = 5000;
				}else if (pidQueryNo == 2){
					printf("\r\n -----FF_Percent-----");
					printf("\r\n Current FF_Percent value :%2d.Enter new value (0-80)",sV.ff_percent);
					minLimit = 0;
					maxLimit = 80;
				}else if (pidQueryNo == 3){
					printf("\r\n -----start Offset-----");
					printf("\r\n Current start Offset value %03d.Enter new value (0-200)",sV.start_offset);
					minLimit = 0;
					maxLimit = 200;
				}
				if (pidQueryNo == 4){
					printf("\r\n -----All values Entered!-----");
					printf("\r\n Kp:%5.2f, Ki:%5.2f, FF_percent:%02d, StartOffset:%03d",Kp_new,Ki_new,FF_factor_new,startOffsetNew);
					printf("\r\n Write into Eeprom and enable? (1 for yes,0 for no)");
					eepromWrite = 1;
				}
				pidQueryNo ++;
				newPIDquery = 0;
			}
		} // closes else

	} //finishes while
	return 1;
}


uint8_t configureSettings(uint8_t setting){
	int noEntered;
	char eepromWrite = 0;
	uint8_t firstTime = 1;
	uint16_t minLimit=0;
	uint16_t maxLimit=0;
	uint16_t noToSave =0;
	uint8_t eepromWriteSuccess =0;
	while(1){
		if (firstTime == 1){
			if (setting == CONSOLE_CANID){
				printf("\r\n***MOTOR CAN-ID MENU***");
				printf("\r\n Enter new Motor CAN ID (2-7)");
				minLimit = 2;
				maxLimit = 7;
			}else if (setting == CONSOLE_DIRECTION){
				printf("\r\n***MOTOR DEFAULT-DIRECTION MENU***");
				printf("\r\n Enter Motor Default Direction (CW-0,CCW-1)");
				minLimit = 0;
				maxLimit = 1;
			}else if (setting == CONSOLE_ENCODER_OFFSET){
				printf("\r\n***MOTOR ENCODER-OFFSET  MENU***");
				printf("\r\n Enter Encoder Offset (0-16384)");
				minLimit = 0;
				maxLimit = 16384;
			}else{}
			printf("\r\n Enter -1 to go back");
			firstTime = 0;
		}

		noEntered = waitForNoInput();

		if (eepromWrite == 0){
			if (noEntered == -1){
			  printf("\r\n");
			  break;
			}
			if ((noEntered <= maxLimit ) && ( noEntered >= minLimit)){
				noToSave = noEntered;
				printf("\r\n Write into Eeprom and enable? (1 for yes,0 for no)");
				eepromWrite = 1;
				noEntered = 0;
			}else{
				printf("\r\n Outside Limits!Try again");
				noEntered = 0;
			}
		}else{
			if (noEntered == 1){
				if (setting == CONSOLE_CANID){
					eepromWriteSuccess  = writeMotorSettingsToEEPROM_Manual(noToSave,-1,-1);
				}else if (setting == CONSOLE_DIRECTION){
					eepromWriteSuccess  = writeMotorSettingsToEEPROM_Manual(-1,-1,noToSave);
				}else if (setting == CONSOLE_ENCODER_OFFSET){
					eepromWriteSuccess  = writeMotorSettingsToEEPROM_Manual(-1,noToSave,-1);
				}
				if (eepromWriteSuccess){
					printf("\r\n Success! Written into Eeprom!Restart Motor for new values to take effect");
				}else{
					printf("\r\n Fail! Not Written into Eeprom!");
				}
				printf("\r\n Enter a new no to save or -1 to go back");
				eepromWrite = 0;
				noEntered = 0;
			}
			else if (noEntered == 0){
				printf("\r\n Exited without writing into Eeprom!");
				printf("\r\n Enter a new no to save or -1 to go back");
				eepromWrite = 0;
				noEntered = 0;
			}
			else {}
		}//finishes eepromWrite else

	} //finishes while
	return 1;
}

uint8_t printSettings(void){
	uint8_t firstTime = 1;
	int noEntered = 0;
	while(1){
		if (firstTime){
			printf("\r\n***MOTOR SETTINGS****");
			printf("\r\n MOTOR CAN ID = %d",sV.MOTID);
			printf("\r\n MOTOR Encoder Offset = %d",sV.AMS_offset_index);
			printf("\r\n MOTOR Default Dir = %d (CW=0,CCW=1)",sV.default_direction);
			printf("\r\n Kp:%6.3f, Ki:%6.3f, FF_percent:%02d, StartOffset:%03d",sV.Kp,sV.Ki,sV.ff_percent,sV.start_offset);
			printf("\r\n Enter -1 to exit");
			firstTime = 0;
		}

		noEntered = waitForNoInput();

		if (noEntered == -1){
			noEntered = 0;
			break;
		}else{
			noEntered = 0;
		}
	}

	return 1;
}

void MX_TIM1_Init_Copy(void){
  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
	Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
	Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
	Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
	Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
	Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
	Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
	Error_Handler();
  }
  HAL_TIMEx_EnableDeadTimePreload(&htim1);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 90;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
	Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}



uint8_t ToggleLogSettings(void){
	uint8_t firstTime = 1;
	int noEntered = 0;
	while(1){
		if (firstTime){
			printf("\r\n***TOGGLE LOG SETTING***");
			printf("\r\n ");
			printf("\r\n Log Enabled ? :%01d",S.loggingEnabled);
			printf("\r\n Press 1 to toggle the logging setting");
			printf("\r\n Enter -1 to exit");
			firstTime = 0;
		}

		noEntered = waitForNoInput();
		if (noEntered == -1){
			noEntered = 0;
			break;
		}else if (noEntered == 1){
			S.loggingEnabled = !(S.loggingEnabled);
			printf("\r\n Log Enabled ? :%01d",S.loggingEnabled);
			noEntered = 0;
		}else{
			noEntered = 0;
		}
	}

	return 1;
}
uint8_t runMotorCalibrationRoutine(void){
	int noEntered = 0;
	char skipScanf = 0;
	char firstTime = 1;
	uint8_t zeroed;
	while(1){

		if (firstTime == 1){
			printf("\r\n***MOTOR CALIBRATION ROUTINE****");
			printf("\r\n Start the Calibration Routine? Enter 1 for yes and -1 to go back");
			printf("\r\n You cannot stop the routine till its over");
			printf("\r\n Note down the index no and enter it through option 3 in the main Menu");
			skipScanf = 0;
			firstTime = 0;
		}

		if (skipScanf == 0){
			noEntered = waitForNoInput();
		}

		if (noEntered == 1){
			printf("\r\n running Calibration");
			zeroed = updateEncoderZeroPosition(0);
			if (zeroed){
				 MX_TIM1_Init_Copy();
				 RunCalibrationWithPrintf(); // BLOCKING
				 printf("\r\n Do you wish to run the routine again?(1 for yes, -1 for no and to go back)");
				 skipScanf = 0;
			 }else{
				 printf("\r\n Zeroing Encoder before Calibration Failed! Try Again!");
				 firstTime = 1;
			 }
			noEntered = 0;

		}

		if (noEntered == -1){
			noEntered = 0;
			printf("\r\n");
			break;
		}
	}
	return 1;
}


uint8_t runMotorOpenLoop(void){
	int noEntered = 0;
	char skipScanf = 0;
	char firstTime = 1;
	char menuLevel = 0;
	while(1){

		if (firstTime == 1){
			if (menuLevel == 0){
				printf("\r\n***MOTOR OPENLOOP TESTS****");
				printf("\r\n Enter 1 to run the calibration Test");
				printf("\r\n Enter 2 to run the openLoop Test");
				printf("\r\n press '-1' to exit");
				skipScanf = 0;
				firstTime = 0;
				C.OL_queryNo = 0;
			}
		}

		if (skipScanf == 0){
			noEntered = waitForNoInput();

			if (menuLevel == 0){
				if (noEntered == -1){
					break;
				}
				if (noEntered == 1){
					printf("\r\n *** Calibration Test ***");
					printf("\r\n ---Instructions---");
					printf("\r\n This is a 60 second test");
					printf("\r\n There should be no pulley, gearbox on the shaft.");
					printf("\r\n Current draw while running should be close to 0.050A");
					printf("\r\n ---Controls---");
					printf("\r\n press 1 to start");
					printf("\r\n press 2 to ramp down (while running)");
					printf("\r\n press 3 to stop (while running)");
					printf("\r\n press 4 to toggle direction of rotation (while stopped)");
					printf("\r\n press -1 to go back (while stopped)");
					printf("\r\n ---Data Format----");
					printf("\r\n Time(sec),PWM,RPM,Current(A),Voltage(V) \r\n");
					menuLevel = 1;
					C.OLmode = OL_TESTLOOP_MODE;
					noEntered = 0;
				}
				if (noEntered == 2){
					printf("\r\n *** OpenLoop Free Run Mode ***");
					printf("\r\n ---- 1. Enter Target duty Cycle (20-1400)");
					printf("\r\n press -1 to go back");
					C.OL_queryNo = 1;
					menuLevel = 1;
					C.OLmode = OL_FREERUN_MODE;
					noEntered = 0;
				}
			}

			else if (menuLevel == 1){
				if (C.runningOL == 0){
					if (C.OLmode == OL_TESTLOOP_MODE){
						if (noEntered == 1){
							RM.rampTarget = 300;
							RM.rampSteadyRunTime_s = 60;
							RM.rampRampUpTime_s = 6;
							RM.rampRampDownTime_s = 6;
							printf("\r\n H:Time(sec),PWM,RPM,Current(A),Voltage(V):E \r\n");
							Console_OL_RAMPUP();
							C.runningOL = 1;
							C.logging = 1;
							C.consoleLoggingOn = 1;
							noEntered = 0;
						}
						if (noEntered == 4){
							RM.rotateDirection = !RM.rotateDirection;
							printf("\r\n Rotate Direction = %01d",RM.rotateDirection);
							noEntered =0;
						}
					}
					if (C.OLmode == OL_FREERUN_MODE){
						if (C.OL_queryNo == 1){
							if (noEntered == -1){
								menuLevel = 0;
								C.OL_queryNo = 0;
								C.OLmode  = 0;
								firstTime = 1;
								noEntered = 0;
							}
							else if ((noEntered >= 20) && (noEntered <= 1400)){
								RM.rampTarget = noEntered;
								printf("\r\n ---- 2.Enter steadyState Time in sec (10-300)");
								printf("\r\n press -1 to go back");
								C.OL_queryNo = 2;
							}else{
								printf("\r\n Not in Range (20-1400). Try Again!");
							}
							noEntered = 0;
						}else if (C.OL_queryNo == 2){
							if (noEntered == -1){
								menuLevel = 0;
								C.OL_queryNo = 0;
								C.OLmode  = 0;
								firstTime = 1;
								noEntered = 0;
							}
							else if ((noEntered >= 10) && (noEntered <= 300)){
								RM.rampSteadyRunTime_s = noEntered;
								RM.rampRampUpTime_s = RM.rampTarget/50;
								if (RM.rampRampUpTime_s  < 5){
									RM.rampRampUpTime_s  = 5;
								}
								RM.rampRampDownTime_s = RM.rampRampUpTime_s;
								C.OL_queryNo = 3;
								printf("\r\n---- 3. Setup Complete");
								printf("\r\n ---Controls---");
								printf("\r\n press 1 to start");
								printf("\r\n press 2 to ramp down (while running)");
								printf("\r\n press 3 to stop (while running)");
								printf("\r\n press 4 to toggle direction of rotation (while stopped)");
								printf("\r\n press -1 to go back (while stopped)");
								printf("\r\n ---Data Format----");
								printf("\r\n Time(sec),PWM,RPM,Current(A),Voltage(V) \r\n");
							}else{
								printf("\r\n Not in Range (10-300). Try Again!");
							}
							noEntered = 0;
						}else if(C.OL_queryNo == 3){
							if (noEntered == 1){
								printf("\r\n *** Ramping Up ***");
								printf("\r\n H:Time(sec),PWM,RPM,Current(A),Voltage(V):E \r\n");
								Console_OL_RAMPUP();
								C.runningOL = 1;
								C.logging = 1;
								C.consoleLoggingOn = 1;
								noEntered = 0;
							}else if (noEntered == 4){
							printf("\r\n *** Direction Change ***");
							RM.rotateDirection = !RM.rotateDirection;
							printf("\r\n Rotate Direction = %01d",RM.rotateDirection);
							noEntered =0;
							}
						}
					}//closes if free run mode
				}else{ // if already running
					if (noEntered == 2){
						printf("\r\n *** Ramping Down ***");
						printf("\r\n Wait till the motor stops before the restarting it");
						printf("\r\n Time(sec),PWM,RPM,Current(A),Voltage(V) \r\n");
						Console_OL_RAMPDOWN();
						noEntered = 0;
						C.runningOL = 0;
					}else if (noEntered == 3){
						printf("\r\n *** Stopping ***\r\n");
						Console_OL_STOP();
						noEntered = 0;
						C.runningOL = 0;
						}
				}

				if (noEntered == -1){
					menuLevel = 0;
					firstTime = 1;
				}

			} //closes menulevel 1

		}
	} // closes while
	return 1;
}


uint8_t runMotorClosedLoop(void){
	int noEntered = 0;
	char skipScanf = 0;
	char firstTime = 1;
	while(1){
		if (firstTime == 1){
			printf("\r\n***MOTOR CLOSEDLOOP TESTS****");
			printf("\r\n 1. Enter Target RPM (40-1400)");
			printf("\r\n press -1 to go back");
			skipScanf = 0;
			firstTime = 0;
			C.CL_queryNo = 0;
		}

		if (skipScanf == 0){
			noEntered = waitForNoInput(); // blocking

			if (C.runningCL == 0){
				if (C.CL_queryNo == 0){
					if (noEntered == -1){
						break;
					}
					else if ((noEntered >= 40) && (noEntered <= 1400)){
						RM.rampTarget = noEntered;
						printf("\r\n ---- 2.Enter steadyState Time in sec (10-300)");
						printf("\r\n press -1 to go back");
						C.CL_queryNo = 1;
					}else{
						printf("\r\n Not in Range (40-1400). Try Again!");
					}
				}else if (C.CL_queryNo == 1){
					if (noEntered == -1){
						firstTime = 1;
					}else if ((noEntered >= 10) && (noEntered <= 300)){
						RM.rampSteadyRunTime_s = noEntered;
						RM.rampRampUpTime_s = RM.rampTarget/50;
						if (RM.rampRampUpTime_s  < 5){
							RM.rampRampUpTime_s  = 5;
						}
						RM.rampRampDownTime_s = RM.rampRampUpTime_s;
						C.CL_queryNo = 2;
						printf("\r\n---- 3. Setup Complete");
						printf("\r\n ---Controls---");
						printf("\r\n press 1 to start");
						printf("\r\n press 2 to ramp down (while running)");
						printf("\r\n press 3 to stop (while running)");
						printf("\r\n press 4 to toggle direction of rotation (while stopped)");
						printf("\r\n press -1 to go back (while stopped)");
						printf("\r\n ---Data Format----");
						printf("\r\n Time(sec),TargetRPM,ActualRPM,total_PWM,Kp_PWM,KI_PWM,FF_PWM,Current(A),Voltage(V)\r\n");

					}else{
						printf("\r\n Not in Range (10-300). Try Again!");
					}
				}else if (C.CL_queryNo == 2){
					if (noEntered == -1){
						firstTime = 1;
					}
					else if (noEntered == 1){
						printf("\r\n *** Ramping Up ***");
						printf("\r\n H:Time(sec),TargetRPM,ActualRPM,total_PWM,Kp_PWM,KI_PWM,FF_PWM,Current(A),Voltage(V):E\r\n");
						Console_CL_RAMPUP();
						C.runningCL = 1;
						C.logging = 1;
						C.consoleLoggingOn = 1;
						noEntered = 0;
					}else if (noEntered == 4){
						printf("\r\n *** Direction Change ***");
						RM.rotateDirection = !RM.rotateDirection;
						printf("\r\n Rotate Direction = %01d",RM.rotateDirection);
						noEntered =0;
					}
				}
			}else{ // if already running
				if (noEntered == 2){
					printf("\r\n *** Ramping Down ***");
					printf("\r\n Wait till the motor stops before the restarting it");
					printf("\r\n Time(sec),TargetRPM,ActualRPM,total_PWM,Kp_PWM,KI_PWM,FF_PWM,Current(A),Voltage(V) \r\n");
					Console_CL_RAMPDOWN();
					noEntered = 0;
					C.runningCL = 0;
				}else if (noEntered == 3){
					printf("\r\n *** Stopping ***\r\n");
					Console_CL_STOP();
					noEntered = 0;
					C.runningCL = 0;
					}
			}
		} //closes skipscanf
	} // closes while
	return 1;
}


void configurationFromTerminal(void)
{
	  char firstTime = 1;
	  int noEntered;
	  while(1){
		  if (firstTime == 1){
			  printf("\r\n***MOTOR CONFIGURATION MENU***");
			  printf("\r\n 1. View Motor Settings");
			  printf("\r\n 2. Change Motor CAN ID");
			  printf("\r\n 3. Change Motor Encoder Offset");
			  printf("\r\n 4. Change Default Direction");
			  printf("\r\n 5. Change PID Settings");
			  printf("\r\n 6. Run Encoder Calibration Routine");
			  printf("\r\n 7. Run Motor OpenLoop Tests");
			  printf("\r\n 8. Run Motor ClosedLoop Tests");
			  printf("\r\n 9. Restart Motor Code");
			  printf("\r\n 10. Enable Motor Logging");
			  printf("\r\n Enter a no btw 1-10 and enter to select an option");
			  printf("\r\n Press -1 and enter from this menu to exit \r\n");
			  firstTime = 0;
		  }

		  noEntered = waitForNoInput();

		  if (noEntered == -1){
			  printf("\r\n Bye!");
			  printf("\r\n ************");
			  break;
		  }

		  if (noEntered == 1){
			  firstTime = printSettings();
			  noEntered = 0;
		  }
		  if (noEntered == 2){
			  firstTime = configureSettings(CONSOLE_CANID);
			  noEntered = 0;
		  }
		  if (noEntered == 3){
			  firstTime = configureSettings(CONSOLE_ENCODER_OFFSET);
			  noEntered = 0;
		  }
		  if (noEntered == 4){
			  firstTime = configureSettings(CONSOLE_DIRECTION);
			  noEntered = 0;
		  }
		  if (noEntered == 5){
			  firstTime = configurePIDSettings();
			  noEntered = 0;
		  }
		  if (noEntered == 6){
			  firstTime = runMotorCalibrationRoutine();
			  noEntered = 0;
		  }
		  if (noEntered == 7){
			  firstTime = runMotorOpenLoop();
			  noEntered = 0;
		  }
		  if (noEntered == 8){
			  firstTime = runMotorClosedLoop();
			  noEntered = 0;
		  }
		  if (noEntered == 9){
			  printf("\r\n Resetting Motor Code! Exiting Console also!");
			  printf("\r\n Bye!");
			  printf("\r\n ************");
			  NVIC_SystemReset();
			  noEntered = 0;
		  }
		  if (noEntered == 10){
			  firstTime = ToggleLogSettings();
			  noEntered = 0;
		  }
	  }
}
