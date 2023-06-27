/*
 * EncoderCalibration.c
 *
 *  Created on: Feb 18, 2023
 *      Author: harsha
 */

#include "stm32g4xx_hal.h"
#include "EncoderCalibration.h"
#include "AS5x47P.h"

uint16_t encCalib_delayTime = 0;
uint16_t encCalib_pwmVal = 0;

extern TIM_HandleTypeDef htim1;

void setupCalibration(uint8_t dutyPercentage,uint16_t delayTime_ms){
	encCalib.calib_pwmVal = ((float)dutyPercentage/100.0) * htim1.Instance->ARR;
	encCalib.calib_timeVal = delayTime_ms;

	HAL_TIM_Base_Start_IT(&htim1);

	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);

	//TurnOffAllPhases();
}

void setPhaseU(uint16_t pwmVal){
	htim1.Instance->CCR1 = pwmVal;
}
void setPhaseV(uint16_t pwmVal){
	htim1.Instance->CCR2 = pwmVal;
}
void setPhaseW(uint16_t pwmVal){
	htim1.Instance->CCR3 = pwmVal;
}

void TurnOffAllPhases(void){
	htim1.Instance->CCR1 = 0;
	htim1.Instance->CCR2 = 0;
	htim1.Instance->CCR3 = 0;
}

void voltageOnPrincipalAxis(EncCalib_TypeDef *encC,uint8_t axis){
	if (axis == 1){
		setPhaseU(encC->calib_pwmVal );
		setPhaseV(0);
		setPhaseW(0);
		HAL_Delay(encC->calib_timeVal);
		TurnOffAllPhases();
	}
	if (axis == 2){
		setPhaseU(encC->calib_pwmVal );
		setPhaseV(encC->calib_pwmVal );
		setPhaseW(0);
		HAL_Delay(encC->calib_timeVal);
		TurnOffAllPhases();
	}
	if (axis == 3){
		setPhaseU(0);
		setPhaseV(encC->calib_pwmVal );
		setPhaseW(0);
		HAL_Delay(encC->calib_timeVal);
		TurnOffAllPhases();
	}
	if (axis == 4){
		setPhaseU(0);
		setPhaseV(encC->calib_pwmVal );
		setPhaseW(encC->calib_pwmVal );
		HAL_Delay(encC->calib_timeVal);
		TurnOffAllPhases();
	}
	if (axis == 5){
		setPhaseU(0);
		setPhaseV(0);
		setPhaseW(encC->calib_pwmVal );
		HAL_Delay(encC->calib_timeVal);
		TurnOffAllPhases();
	}
	if (axis == 6){
		setPhaseU(encC->calib_pwmVal );
		setPhaseV(0);
		setPhaseW(encC->calib_pwmVal );
		HAL_Delay(encC->calib_timeVal);
		TurnOffAllPhases();
	}
}

void InitializeEncoderCalib_TypeDef(EncCalib_TypeDef *encC){
	encC->calib_pwmVal = 0;
	encC->calib_timeVal = 0;
	encC->encIndex = 0;
	encC->encIndex_wOffset = 0;
	encC->first_reading = 0;
	encC->pv_Axis = 0;
	encC->minError = 0;
	encC->maxError = 0;
	encC->avgError = 0;
	for (int i=0;i<30;i++){
		encC->angleReading[i] = 0;
		encC->dAngle[i] = 0;
		encC->deltaError[i] = 0;
	}
}

void RunCalibration(void){
	setupCalibration(10,50);
	encCalib.pv_Axis = 0;

	for (int i=0;i<30; i++){
		encCalib.pv_Axis += 1;

		if (encCalib.pv_Axis > 6){
		  encCalib.pv_Axis = 1;
		}

		voltageOnPrincipalAxis(&encCalib,encCalib.pv_Axis);
		HAL_Delay(1000); // long delay for vibrations to settle.
		encCalib.angleReading[i] =  GetAveragedAngleReading(5);

		if (i == 0){
			encCalib.first_reading = encCalib.angleReading[i];
			encCalib.dAngle[i] = 0;
			encCalib.deltaError[i] = 0;
		  }
		else{
			encCalib.dAngle[i]  = encCalib.angleReading[i]- encCalib.previousAngle;
			if (encCalib.dAngle[i]  < 0){
				encCalib.dAngle[i]  = 16384 - encCalib.previousAngle + encCalib.angleReading[i];
			}
			encCalib.deltaError[i] = encCalib.dAngle[i] - ENC_COUNTS_60_ELEC_DEGREES;
		}

		encCalib.previousAngle = encCalib.angleReading[i];

		if (encCalib.deltaError[i] > encCalib.maxError){
			encCalib.maxError = encCalib.deltaError[i];
		}

	    if (encCalib.deltaError[i] < encCalib.minError){
	    	encCalib.minError = encCalib.deltaError[i];
	    }
	}

	encCalib.avgError = (encCalib.maxError + encCalib.minError)/2;
	encCalib.encIndex = encCalib.first_reading + encCalib.avgError;
	if (encCalib.encIndex < 0){
		encCalib.encIndex = 16384 + encCalib.encIndex;
	}
	if (encCalib.encIndex > 16384){
		encCalib.encIndex = encCalib.encIndex - 16384;
	}
	encCalib.encIndex_wOffset = encCalib.encIndex + EMPIRICAL_OFFSET;
	if (encCalib.encIndex_wOffset < 0){
		encCalib.encIndex_wOffset = 16384 + encCalib.encIndex_wOffset;
	}
	if (encCalib.encIndex_wOffset > 16384){
		encCalib.encIndex_wOffset = encCalib.encIndex_wOffset - 16384;
	}

	TurnOffAllChannels();
}



uint16_t RunCalibrationWithPrintf(void){
	setupCalibration(10,50);
	encCalib.pv_Axis = 0;

	for (int i=0;i<30; i++){
		printf("\r\n step %02d",i);
		encCalib.pv_Axis += 1;

		if (encCalib.pv_Axis > 6){
		  encCalib.pv_Axis = 1;
		}

		voltageOnPrincipalAxis(&encCalib,encCalib.pv_Axis);
		HAL_Delay(1000); // long delay for vibrations to settle.
		encCalib.angleReading[i] =  GetAveragedAngleReading(5);

		if (i == 0){
			encCalib.first_reading = encCalib.angleReading[i];
			encCalib.dAngle[i] = 0;
			encCalib.deltaError[i] = 0;
		  }
		else{
			encCalib.dAngle[i]  = encCalib.angleReading[i]- encCalib.previousAngle;
			if (encCalib.dAngle[i]  < 0){
				encCalib.dAngle[i]  = 16384 - encCalib.previousAngle + encCalib.angleReading[i];
			}
			encCalib.deltaError[i] = encCalib.dAngle[i] - ENC_COUNTS_60_ELEC_DEGREES;
		}

		encCalib.previousAngle = encCalib.angleReading[i];

		if (encCalib.deltaError[i] > encCalib.maxError){
			encCalib.maxError = encCalib.deltaError[i];
		}

	    if (encCalib.deltaError[i] < encCalib.minError){
	    	encCalib.minError = encCalib.deltaError[i];
	    }
	}


	encCalib.avgError = (encCalib.maxError + encCalib.minError)/2;
	encCalib.encIndex = encCalib.first_reading + encCalib.avgError;
	if (encCalib.encIndex < 0){
		encCalib.encIndex = 16384 + encCalib.encIndex;
	}
	if (encCalib.encIndex > 16384){
		encCalib.encIndex = encCalib.encIndex - 16384;
	}
	encCalib.encIndex_wOffset = encCalib.encIndex + EMPIRICAL_OFFSET;
	if (encCalib.encIndex_wOffset < 0){
		encCalib.encIndex_wOffset = 16384 + encCalib.encIndex_wOffset;
	}
	if (encCalib.encIndex_wOffset > 16384){
		encCalib.encIndex_wOffset = encCalib.encIndex_wOffset - 16384;
	}

	printf("\r\n avgError %05d",encCalib.avgError);
	printf("\r\n encIndex %05d",encCalib.encIndex);
	printf("\r\n encIndex_wOffset %05d",encCalib.encIndex_wOffset);

	TurnOffAllChannels();

	return encCalib.encIndex_wOffset;
}

