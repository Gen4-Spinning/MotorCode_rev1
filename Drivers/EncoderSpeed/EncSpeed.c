
#include "EncSpeed.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>
extern TIM_HandleTypeDef htim2;

extern char UART_buffer[50];
extern UART_HandleTypeDef huart3;

uint16_t CW_directionError = 0;
uint16_t CCW_directionError = 0;
uint8_t directionErrorFlag = 0;

void InitializeEncSpeed_TypeDef(EncSpeed_TypeDef *e){
	e->EncSpeed_calcTime_ms = 1;
	e->encDirection=0;
	e->currentEncCount=0;
	e->previousEncCount=0;
	e->deltaEncCount=0;
	e->bufferIdx = 0;
	for (int i = 0;i< SPEED_BUFFER_SIZE; i++){
		e->speedArray[i]=0;
	}
	e->speedArrayFilled=0;
	e->speed_s16=0;
	e->speedRPM=0;
}

void CalcEncSpeed(EncSpeed_TypeDef *e,sixSectorCntrl *s){
	e->currentEncCount = htim2.Instance->CNT;
	e->encDirection = (TIM2->CR1 & TIM_CR1_DIR) >> TIM_CR1_DIR_Pos; // is only useful when moving.

	// here currentCnt > previousCnt, by the direction in which the encoder increases
	// previous  ---2047--2048--0--1--2--current

	//calculated top RPM = 1500. -> 25 rps. we re taking cnts in a 1ms timer.
	// 2048 pulses per rotation. so @25 rps -> 25*2048 = 51200.  per ms -> 51.2 max. so we wont get
	// an index pin within 1 ms. so we just take delta cnt.

	//TODO CANNOT differentiate btw wrong direction rotation due to bad encode index and low speed RPM.
	//TO FIX
	if (s->direction == CW ){
		if (e->encDirection == CW){
		  e->deltaEncCount = e->currentEncCount - e->previousEncCount;
		  if (e->deltaEncCount < 0){
			  e->deltaEncCount  = e->currentEncCount + (2048 - e->previousEncCount);
			  }
		  CW_directionError = 0;
		  }else{
			  CW_directionError += 1;
			  // if the RPM is close to zero then the difference in directions is OK.
			  //we just handle it by making delta count = 0;
			  if (e->total_deltaEncCount <= 2){
				  e->deltaEncCount = 0;
			  }
		  }

	}

	  // here current Count will be less than previous Count
	  //current ---2047--2048--0--1--2--previous
	  if (s->direction == CCW){
		  if (e->encDirection == CCW){
			  e->deltaEncCount = e->previousEncCount - e->currentEncCount;
			  if (e->deltaEncCount < 0){
				  e->deltaEncCount  = ((2048- e->currentEncCount) +  e->previousEncCount);
			  }
			  CCW_directionError = 0;
		  }else{
			  CCW_directionError += 1;
			  // if the RPM is close to zero then the difference in directions is OK.
			  //we just handle it by making delta count = 0;
			  if (e->total_deltaEncCount <= 2){
					  e->deltaEncCount = 0;
			  }
		  }
	  }

	  if ((CW_directionError >= 200) || (CCW_directionError >= 200)){
		  directionErrorFlag = 1;

	  }

	  uint16_t previousdeltaVal;
	  if (e->speedArrayFilled==0){
		  e->speedArray[e->bufferIdx] = e->deltaEncCount;
		  e->total_deltaEncCount += e->deltaEncCount;
	  }else{
		  previousdeltaVal = e->speedArray[e->bufferIdx];
		  e->speedArray[e->bufferIdx] = e->deltaEncCount;
		  e->total_deltaEncCount -= previousdeltaVal;
		  e->total_deltaEncCount += e->deltaEncCount;
	  }
	  e->previousEncCount = e->currentEncCount;

	  //constants are defined for 16 ms buffer
	  e->speed_s16 = e->total_deltaEncCount * CNTS_TO_S16_CONSTANT;
	  e->speedRPM = e->total_deltaEncCount * CNTS_TO_RPM_CONSTANT;
	  e->bufferIdx++;

	  if (e->bufferIdx >15){
		  e->bufferIdx = 0;
		  e->speedArrayFilled=1;
	  }

	  if (e->speedRPM < 2){ // to detect zero Speed
		  e->zeroSpeed = 1;
	  }else{
		  e->zeroSpeed = 0;
	  }
}
