/*
 * sixSector.c
 *
 *  Created on: Feb 26, 2023
 *      Author: harsha
 */

#include "stdint.h"
#include "sixSector.h"
#include "Constants.h"

extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim1;
void setCH1_PwmMode(){
  TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M); //sets OCM to ZERO, in CCMR1
  TIM1->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);  //pwm mode 1 is 0110
}
void setCH2_PwmMode(){
  TIM1->CCMR1 &= ~(TIM_CCMR1_OC2M); //sets OCM to ZERO in CCMR1
  TIM1->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);  //pwm mode 1 is 0110
}
void setCH3_PwmMode(){
  TIM1->CCMR2 &= ~(TIM_CCMR2_OC3M); //sets OCM to ZERO in CCMR2
  TIM1->CCMR2 |= (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2);  //pwm mode 1 is 0110
}


void setCH1_ForcedActiveMode(){
  TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M); //sets OCM to ZERO
  TIM1->CCMR1 |= (TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_2);  //forced active is 0101
}
void setCH2_ForcedActiveMode(){
  TIM1->CCMR1 &= ~(TIM_CCMR1_OC2M); //sets OCM to ZERO
  TIM1->CCMR1 |= (TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_2);  //forced active is 0101
}
void setCH3_ForcedActiveMode(){
  TIM1->CCMR2 &= ~(TIM_CCMR2_OC3M); //sets OCM to ZERO
  TIM1->CCMR2 |= (TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3M_2);  //forced active is 0101
}

void setCH1_ForcedInActiveMode(){
  TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M); //sets OCM to ZERO
  TIM1->CCMR1 |= TIM_CCMR1_OC1M_2;  //forced Inactive is 0100
}
void setCH2_ForcedInActiveMode(){
  TIM1->CCMR1 &= ~(TIM_CCMR1_OC2M); //sets OCM to ZERO
  TIM1->CCMR1 |= TIM_CCMR1_OC2M_2;  //forced Inactive is 0100
}
void setCH3_ForcedInActiveMode(){
  TIM1->CCMR2 &= ~(TIM_CCMR2_OC3M); //sets OCM to ZERO
  TIM1->CCMR2 |= TIM_CCMR2_OC3M_2;  //forced Inactive is 0100
}

void CH1_Off(){
  //disable CH1. Make OCRef, forced active, and switch on the CH1N
  TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_1, TIM_CCx_DISABLE);
  setCH1N(ON);
  setCH1_ForcedInActiveMode();
}

void CH2_Off(){
  //disable CH2. Make OCRef, forced active, and switch on the CH1N
  TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_2, TIM_CCx_DISABLE);
  setCH2N(ON);
  setCH2_ForcedInActiveMode();
}

void CH3_Off(){
  //disable CH2. Make OCRef, forced active, and switch on the CH1N
  TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_3, TIM_CCx_DISABLE);
  setCH3N(ON);
  setCH3_ForcedInActiveMode();
}


void setCH1(uint8_t state){
  if (state == ON){
    TIM1->CCER |= TIM_CCER_CC1E;
  }else{
    TIM1->CCER &= ~TIM_CCER_CC1E;
  }
}

void setCH1N(uint8_t state){
  if (state == ON){
    TIM1->CCER |= TIM_CCER_CC1NE;
  }else{
    TIM1->CCER &= ~TIM_CCER_CC1NE;
  }
}

void setCH2(uint8_t state){
  if (state == ON){
    TIM1->CCER |= TIM_CCER_CC2E;
  }else{
    TIM1->CCER &= ~TIM_CCER_CC2E;
  }
}

void setCH2N(uint8_t state){
  if (state == ON){
    TIM1->CCER |= TIM_CCER_CC2NE;
  }else{
    TIM1->CCER &= ~TIM_CCER_CC2NE;
  }
}

void setCH3(uint8_t state){
  if (state == ON){
    TIM1->CCER |= TIM_CCER_CC3E;
  }else{
    TIM1->CCER &= ~TIM_CCER_CC3E;
  }
}

void setCH3N(uint8_t state){
  if (state == ON){
    TIM1->CCER |= TIM_CCER_CC3NE;
  }else{
    TIM1->CCER &= ~TIM_CCER_CC3NE;
  }
}

void setPhaseA(uint8_t switchSide){
  if (switchSide == HIGH_SIDE){
     setCH1(ON);
     setCH1N(ON);
     setCH1_PwmMode();
  }
  if (switchSide == LOW_SIDE){
    setCH1(OFF);
    setCH1N(ON);
    setCH1_ForcedActiveMode();
  }
}

void setPhaseB(uint8_t switchSide){
  if (switchSide == HIGH_SIDE){
     setCH2(ON);
     setCH2N(ON);
     setCH2_PwmMode();
  }
  if (switchSide == LOW_SIDE){
    setCH2(OFF);
    setCH2N(ON);
    setCH2_ForcedActiveMode();
  }
}

void setPhaseC(uint8_t switchSide){
  if (switchSide == HIGH_SIDE){
     setCH3(ON);
     setCH3N(ON);
     setCH3_PwmMode();
  }
  if (switchSide == LOW_SIDE){
    setCH3(OFF);
    setCH3N(ON);
    setCH3_ForcedActiveMode();
  }
}

void TurnOffAllChannels(void){
	  setCH1(OFF);
	  setCH1N(OFF);
	  setCH2(OFF);
	  setCH2N(OFF);
	  setCH3(OFF);
	  setCH3N(OFF);
}

void sixSectorInit(sixSector *sixSectorObj){
	sixSectorObj->sector1Start_CW = SECTOR1_START_CW;
	sixSectorObj->sector1Start_CCW = SECTOR1_START_CCW;
	sixSectorObj->sectorCnt = 65535;
	sixSectorObj->electricalSector  = 255;
	sixSectorObj->prev_electricalSector = 255;
}

void updateSectorBoundaries(sixSector *sixSectorObj,uint8_t direction){
	if (direction == CW){
		sixSectorObj->sectorBoundaries[0] = sixSectorObj->sector1Start_CW;
	}
	else{
		sixSectorObj->sectorBoundaries[0] = sixSectorObj->sector1Start_CCW;
	}
	sixSectorObj->sectorBoundaries[1] = sixSectorObj->sectorBoundaries[0] + SECTOR_CNT;
	sixSectorObj->sectorBoundaries[2] = sixSectorObj->sectorBoundaries[1] + SECTOR_CNT;
	sixSectorObj->sectorBoundaries[3] = sixSectorObj->sectorBoundaries[2] + SECTOR_CNT;
	sixSectorObj->sectorBoundaries[4] = sixSectorObj->sectorBoundaries[3] + SECTOR_CNT;
	sixSectorObj->sectorBoundaries[5] = sixSectorObj->sectorBoundaries[4] + SECTOR_CNT;

}
void CalcSector_fromEncoder(sixSector *sixSectorObj,uint16_t encoderCNT){
	sixSectorObj->sectorCnt = encoderCNT%ENC_CNTS_PER_POLE_PAIR;

	if ((sixSectorObj->sectorCnt >= sixSectorObj->sectorBoundaries[0] ) && (sixSectorObj->sectorCnt < sixSectorObj->sectorBoundaries[1])){
		sixSectorObj->electricalSector = 1;
	//	HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_2,DAC_ALIGN_12B_R,0);
	}
	else if ((sixSectorObj->sectorCnt >= sixSectorObj->sectorBoundaries[1] ) && (sixSectorObj->sectorCnt < sixSectorObj->sectorBoundaries[2])){
		sixSectorObj->electricalSector = 2;
	//	HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_2,DAC_ALIGN_12B_R,600);
	}
	else if ((sixSectorObj->sectorCnt >= sixSectorObj->sectorBoundaries[2] ) && (sixSectorObj->sectorCnt < sixSectorObj->sectorBoundaries[3])){
		sixSectorObj->electricalSector = 3;
	//	HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_2,DAC_ALIGN_12B_R,1200);
	}
	else if ((sixSectorObj->sectorCnt >= sixSectorObj->sectorBoundaries[3] ) && (sixSectorObj->sectorCnt < sixSectorObj->sectorBoundaries[4])){
		sixSectorObj->electricalSector = 4;
	//	HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_2,DAC_ALIGN_12B_R,1800);
	}
	else if ((sixSectorObj->sectorCnt >= sixSectorObj->sectorBoundaries[4] ) && (sixSectorObj->sectorCnt < sixSectorObj->sectorBoundaries[5])){
		sixSectorObj->electricalSector = 5;
	//	HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_2,DAC_ALIGN_12B_R,2400);
	}
	else if ((sixSectorObj->sectorCnt >= sixSectorObj->sectorBoundaries[5] ) || (sixSectorObj->sectorCnt < sixSectorObj->sectorBoundaries[0])){
		sixSectorObj->electricalSector = 6;
	//	HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_2,DAC_ALIGN_12B_R,3000);
	}
	else{
	}

}


void sixSectorCommutateCW(sixSector *sixSectorObj,uint8_t debugOn,uint16_t debugOntime_ms){

	if (sixSectorObj->prev_electricalSector != sixSectorObj->electricalSector){
		if (sixSectorObj->electricalSector == 3){
			//U mosfets OFF
			//V mosfets duty cycle with complementary outputs
			//W mosfet lower on
			// (0,duty,1)

			CH1_Off();

			setCH2(ON);
			setCH2N(ON);
			setCH2_PwmMode();

			setCH3(OFF);
			setCH3N(ON);
			setCH3_ForcedActiveMode();
		  }
		 else if ( sixSectorObj->electricalSector  == 4){
			//U mosfets lower mosfets on
			//V mosfets duty cycle with complementary outputs
			//W mosfet OFF
			// (1,duty,0)

			setCH1(OFF);
			setCH1N(ON);
			setCH1_ForcedActiveMode();

			setCH2(ON);
			setCH2N(ON);
			setCH2_PwmMode();

			CH3_Off();

		  }
		  else if (sixSectorObj->electricalSector  == 5){
			//U mosfets lower on
			//V mosfets OFF
			//W mosfet duty with complementary outputs
			//(1,0,duty)
			setCH1(OFF);
			setCH1N(ON);
			setCH1_ForcedActiveMode();

			CH2_Off();

			setCH3(ON);
			setCH3N(ON);
			setCH3_PwmMode();

		  }
		 else if (sixSectorObj->electricalSector == 6){
			//U mosfets OFF
			//V mosfets lower mosfets on
			//W mosfet duty cycle with complementary PWMs
			// (0,1,duty)

			CH1_Off();

			setCH2(OFF);
			setCH2N(ON);
			setCH2_ForcedActiveMode();

			setCH3(ON);
			setCH3N(ON);
			setCH3_PwmMode();

		  }
		 else if (sixSectorObj->electricalSector == 1){
			//U mosfets complementary outputs
			//V mosfet lower on
			//W mosfets Off
			//(duty,1,0)
			setCH1(ON);
			setCH1N(ON);
			setCH1_PwmMode();

			setCH2(OFF);
			setCH2N(ON);
			setCH2_ForcedActiveMode();

			CH3_Off();

		  }
		   else if (sixSectorObj->electricalSector == 2){
			//U mosfets duty cycle with complementary outputs
			//V mosfets OFF
			//W mosfet lower on
			// (duty,0,1)
			setCH1(ON);
			setCH1N(ON);
			setCH1_PwmMode();

			CH2_Off();

			setCH3(OFF);
			setCH3N(ON);
			setCH3_ForcedActiveMode();

		  }
		  else{
			  TurnOffAllChannels();
		  }
		sixSectorObj->prev_electricalSector = sixSectorObj->electricalSector;
	} //closes prev!= current

	//how TO USE : switch off automatic electrical Sector Calc (CalcSector_fromEncoder) in
	//the PWM interrupt and manually set the sector in the debug mode.
	if (debugOn){
		HAL_Delay(debugOntime_ms);
		sixSectorObj->electricalSector = 0;
	}
}

void sixSectorCommutateCCW(sixSector *sixSectorObj,uint8_t debugOn,uint16_t debugOntime_ms){

	if (sixSectorObj->prev_electricalSector != sixSectorObj->electricalSector){
		if (sixSectorObj->electricalSector == 6){
			//U mosfets OFF
			//V mosfets duty cycle with complementary outputs
			//W mosfet lower on
			// (0,duty,1)

			CH1_Off();

			setCH2(ON);
			setCH2N(ON);
			setCH2_PwmMode();

			setCH3(OFF);
			setCH3N(ON);
			setCH3_ForcedActiveMode();
		  }
		 else if ( sixSectorObj->electricalSector  == 1){
			//U mosfets lower mosfets on
			//V mosfets duty cycle with complementary outputs
			//W mosfet OFF
			// (1,duty,0)

			setCH1(OFF);
			setCH1N(ON);
			setCH1_ForcedActiveMode();

			setCH2(ON);
			setCH2N(ON);
			setCH2_PwmMode();

			CH3_Off();

		  }
		  else if (sixSectorObj->electricalSector  == 2){
			//U mosfets lower on
			//V mosfets OFF
			//W mosfet duty with complementary outputs
			//(1,0,duty)
			setCH1(OFF);
			setCH1N(ON);
			setCH1_ForcedActiveMode();

			CH2_Off();

			setCH3(ON);
			setCH3N(ON);
			setCH3_PwmMode();

		  }
		 else if (sixSectorObj->electricalSector == 3){
			//U mosfets OFF
			//V mosfets lower mosfets on
			//W mosfet duty cycle with complementary PWMs
			// (0,1,duty)

			CH1_Off();

			setCH2(OFF);
			setCH2N(ON);
			setCH2_ForcedActiveMode();

			setCH3(ON);
			setCH3N(ON);
			setCH3_PwmMode();

		  }
		 else if (sixSectorObj->electricalSector == 4){
			//U mosfets complementary outputs
			//V mosfet lower on
			//W mosfets Off
			//(duty,1,0)
			setCH1(ON);
			setCH1N(ON);
			setCH1_PwmMode();

			setCH2(OFF);
			setCH2N(ON);
			setCH2_ForcedActiveMode();

			CH3_Off();

		  }
		   else if (sixSectorObj->electricalSector == 5){
			//U mosfets duty cycle with complementary outputs
			//V mosfets OFF
			//W mosfet lower on
			// (duty,0,1)
			setCH1(ON);
			setCH1N(ON);
			setCH1_PwmMode();

			CH2_Off();

			setCH3(OFF);
			setCH3N(ON);
			setCH3_ForcedActiveMode();

		  }
		  else{
			  TurnOffAllChannels();
		  }
		sixSectorObj->prev_electricalSector = sixSectorObj->electricalSector;
	} //closes prev!= current

	//how TO USE : switch off automatic electrical Sector Calc (CalcSector_fromEncoder) in
	//the PWM interrupt and manually set the sector in the debug mode.
	if (debugOn){
		HAL_Delay(debugOntime_ms);
		sixSectorObj->electricalSector = 0;
	}
}



void sixSectorCntrlInit(sixSectorCntrl *sixSectorCntrlObj){
	sixSectorCntrlObj->turnOn = 0;
	sixSectorCntrlObj->direction = CW;
	sixSectorCntrlObj->duty = 0;
	sixSectorCntrlObj->dutyCycle = 0;
}

void sixSectorSetDuty(sixSectorCntrl *sixSectorCntrlObj,uint16_t duty){
	 sixSectorCntrlObj->duty = duty;
	 sixSectorCntrlObj->dutyCycle = duty*100/1499.0;

     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,duty);
     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,duty);
     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,duty);
}

void StartSixSectorObj(sixSectorCntrl *sixSectorCntrlObj,sixSector *sixSectorObj,uint8_t direction){
	//Setup the sixSector Obj
	sixSectorCntrlObj->direction = direction;
	updateSectorBoundaries(sixSectorObj,direction);
	sixSectorCntrlObj->turnOn = 1;
	sixSectorObj->prev_electricalSector = 0;
}

void StopSixSectorObj(sixSectorCntrl *sixSectorCntrlObj,sixSector *sixSectorObj){
	//Setup the sixSector Obj
	sixSectorCntrlObj->turnOn = 0;
	sixSectorSetDuty(sixSectorCntrlObj,0);
}

