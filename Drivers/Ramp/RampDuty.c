/*
 * Ramp.c
 *
 *  Created on: Feb 26, 2023
 *      Author: harsha
 */

#include "Ramp.h"
#include "sixSector.h"
#include "Constants.h"

void InitRampDutyStruct(RampDuty *ramp,uint16_t targetDuty,long rampUpTime,long rampDownTime,int16_t rampSteadyTime){
	uint16_t totalSteps  = 0;
	ramp->ramp_callingTime_ms = 20;
	ramp->rampUpTime_ms = rampUpTime;
	ramp->rampDownTime_ms = rampDownTime;
	ramp->steadyRunTime_s = rampSteadyTime; // in Seconds!
	ramp->finalTargetDuty = targetDuty;
	ramp->currentDutyF = 0;
	ramp->currentDuty = 0;
	//For RampUp
	totalSteps = ramp->rampUpTime_ms/ramp->ramp_callingTime_ms;
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dDuty_F_RU = ((float)ramp->finalTargetDuty)/totalSteps;

	//For RampDown
	totalSteps = ramp->rampDownTime_ms/ramp->ramp_callingTime_ms;
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dDuty_F_RD = ((float)ramp->finalTargetDuty)/totalSteps;

	ramp->rampPhase = RAMP_WAIT;

	ramp-> transitionTarget = 0;
	ramp-> transitionTime_ms = 0;
	ramp-> dDuty_F_transition = 0;
}
void StartRampDuty(RampDuty *ramp){
	ramp->rampPhase = RAMP_UP;
}

void StartRampDownDuty(RampDuty *ramp){
	ramp->rampPhase = RAMP_DOWN;
}

void StopRampDuty(RampDuty *ramp){
	ramp->rampPhase = RAMP_OVER;
	ramp->currentDutyF = 0;
	ramp->currentDuty = 0;
}

void ResetRampDuty(RampDuty *ramp){
	uint16_t totalSteps  = 0;
	ramp->ramp_callingTime_ms = 20;
	ramp->rampUpTime_ms = 10;
	ramp->rampDownTime_ms = 10;
	ramp->steadyRunTime_s = 30; // in Seconds!
	ramp->finalTargetDuty = 500;
	ramp->currentDutyF = 0;
	ramp->currentDuty = 0;

	//For RampUp
	totalSteps = ramp->rampUpTime_ms/ramp->ramp_callingTime_ms;
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dDuty_F_RU = ((float)ramp->finalTargetDuty)/totalSteps;

	//For RampDown
	totalSteps = ramp->rampDownTime_ms/ramp->ramp_callingTime_ms;
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dDuty_F_RD = ((float)ramp->finalTargetDuty)/totalSteps;

	ramp->rampPhase = RAMP_WAIT;
}


void ExecRampDuty(RampDuty *ramp,TimerTypeDef *t){
	if (ramp->rampPhase == RAMP_UP){
		if(ramp->currentDuty <= ramp->finalTargetDuty){
			ramp->currentDutyF += ramp->dDuty_F_RU;
			ramp->currentDuty = (uint16_t)(ramp->currentDutyF);
			if(ramp->currentDuty >= ramp->finalTargetDuty){
				ramp->currentDuty  = ramp->finalTargetDuty;
				ramp->rampPhase = RAMP_STEADY;
				t->tim16_oneSecTimer = 0;
				t->tim16_20msTimer = 0;
			}
		}
	}

	else if (ramp->rampPhase == RAMP_STEADY){
		if (ramp->steadyRunTime_s != RUN_FOREVER){
			if (t->tim16_oneSecTimer >= ramp->steadyRunTime_s){
				ramp->rampPhase = RAMP_DOWN;
				t->tim16_oneSecTimer = 0;
				t->tim16_20msTimer = 0;
			}
		}
	}

	else if (ramp->rampPhase == RAMP_CHANGE){
		if(ramp->currentDuty < ramp->transitionTarget){
			ramp->currentDutyF += ramp->dDuty_F_transition;
			ramp->currentDuty = (uint16_t)(ramp->currentDutyF);
			if(ramp->currentDuty >= ramp->transitionTarget){
				ramp->currentDuty  = ramp->transitionTarget;
				ramp->rampPhase = RAMP_STEADY;
				t->tim16_oneSecTimer = 0;
				t->tim16_20msTimer = 0;
			}
		}
		if(ramp->currentDuty > ramp->transitionTarget){
			ramp->currentDutyF -= ramp->dDuty_F_transition;
			ramp->currentDuty = (uint16_t)(ramp->currentDutyF);
			if(ramp->currentDuty <= ramp->transitionTarget){
				ramp->currentDuty  = ramp->transitionTarget;
				ramp->rampPhase = RAMP_STEADY;
				t->tim16_oneSecTimer = 0;
				t->tim16_20msTimer = 0;
			}
		}
	}

	else if (ramp->rampPhase == RAMP_DOWN){
		if(ramp->currentDuty >= 0){
			ramp->currentDutyF -= ramp->dDuty_F_RD;
			ramp->currentDuty = (uint16_t)(ramp->currentDutyF);
			if(ramp->currentDuty <= 0){
				ramp->currentDuty  = 0;
				ramp->rampPhase = RAMP_OVER;
				t->tim16_oneSecTimer = 0;
				t->tim16_20msTimer = 0;
			}
		}
	}
	else{
		//Do Nothing
	}

}


void  ChangeDuty(RunMgmtTypeDef *rm, RampDuty *r){
	uint16_t deltaDuty = 0;
	uint16_t totalSteps = 0;
	r->transitionTarget = rm->transitionTarget;
	r->transitionTime_ms = rm->transitionTime_ms;
	if (r->transitionTarget > r->currentDuty){
		deltaDuty = r->transitionTarget - r->currentDuty;
	}else if (r->transitionTarget < r->currentDuty){
		deltaDuty = r->currentDuty - r->transitionTarget;
	}else{
		deltaDuty = 0;
	}
	totalSteps = r->transitionTime_ms/r->ramp_callingTime_ms;
	if (totalSteps == 0){
		totalSteps = 1;
	}
	r->dDuty_F_transition = deltaDuty/totalSteps;
}


// when we get a change Duty, we want to do ramp UP/ramp Down times with that
// new target.
void Recalculate_RampDuty_RampRates(RunMgmtTypeDef *rm, RampDuty *ramp,uint16_t newTarget){
	uint16_t totalSteps = 0;
	//For RampUp
	ramp->finalTargetDuty = newTarget;
	totalSteps = ramp->rampUpTime_ms/ramp->ramp_callingTime_ms;
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dDuty_F_RU = ((float)ramp->finalTargetDuty)/totalSteps;

	//For RampDown
	totalSteps = ramp->rampDownTime_ms/ramp->ramp_callingTime_ms;
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dDuty_F_RD = ((float)ramp->finalTargetDuty)/totalSteps;
}

