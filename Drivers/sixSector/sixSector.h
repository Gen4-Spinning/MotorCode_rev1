/*
 * sixSector.h
 *
 *  Created on: Feb 26, 2023
 *      Author: harsha
 */

#ifndef SIXSECTOR_H
#define SIXSECTOR_H

#include "stm32g4xx_hal.h"

#define ENC_CNTS 2048
#define ENC_CNTS_PER_POLE_PAIR 409 // 2048/5
#define SECTOR_CNT 68 //409/6

#define ORIG_SECTOR_START 34
#define SECTOR1_START_CW ORIG_SECTOR_START //25
#define SECTOR1_START_CCW ORIG_SECTOR_START //60

#define ON 1
#define OFF 0
#define HIGH_SIDE 1
#define LOW_SIDE 0


typedef struct sixSectorStruct {
  uint16_t sector1Start_CW ;
  uint16_t sector1Start_CCW;
  uint16_t sectorBoundaries[6];
  uint16_t sectorCnt;
  uint8_t electricalSector;
  uint8_t prev_electricalSector;
}sixSector;

typedef struct sixSectorCntrlStruct {
  uint8_t turnOn;
  uint8_t direction;
  float dutyCycle;
  uint16_t duty;
}sixSectorCntrl;

void setCH1_PwmMode();
void setCH2_PwmMode();
void setCH3_PwmMode();
void setCH1_ForcedActiveMode();
void setCH2_ForcedActiveMode();
void setCH3_ForcedActiveMode();
void setCH1(uint8_t state);
void setCH1N(uint8_t state);
void setCH2(uint8_t state);
void setCH2N(uint8_t state);
void setCH3(uint8_t state);
void setCH3N(uint8_t state);
void setPhaseA(uint8_t switchSide);
void setPhaseB(uint8_t switchSide);
void setPhaseC(uint8_t switchSide);
void TurnOffAllChannels(void);
void CH1_Off();
void CH2_Off();
void CH3_Off();
void sixSectorInit(sixSector *sixSectorObj);
void CalcSector_fromEncoder(sixSector *sixSectorObj,uint16_t encoderCNT);
void sixSectorCommutateCW(sixSector *sixSectorObj,uint8_t debugOn,uint16_t debugOntime_ms);
void sixSectorCommutateCCW(sixSector *sixSectorObj,uint8_t debugOn,uint16_t debugOntime_ms);

void sixSectorCntrlInit(sixSectorCntrl *sixSectorCntrlObj);
void sixSectorSetDuty(sixSectorCntrl *sixSectorCntrlObj,uint16_t duty);
void updateSectorBoundaries(sixSector *sixSectorObj,uint8_t direction);
void StartSixSectorObj(sixSectorCntrl *sixSectorCntrlObj,sixSector *sixSectorObj,uint8_t direction);
void StopSixSectorObj(sixSectorCntrl *sixSectorCntrlObj,sixSector *sixSectorObj);

extern sixSector sixSectorObj;
extern sixSectorCntrl sixSectorCntrl_Obj;


#endif /* SIXSECTOR_H_ */
