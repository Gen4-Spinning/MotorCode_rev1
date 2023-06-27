/*
 * Ramp.c
 *
 *  Created on: Feb 26, 2023
 *      Author: harsha
 */

#include "Ramp.h"
#include "Constants.h"

void InitRampRPMStruct(RampRPM *ramp,uint16_t targetRPM,long rampUpTime,long rampDownTime,int16_t rampSteadyTime){
	uint16_t totalSteps  = 0;
	ramp->ramp_callingTime_ms = 20;
	ramp->rampUpTime_ms = rampUpTime;
	ramp->rampDownTime_ms = rampDownTime;
	ramp->steadyRunTime_s = rampSteadyTime; // in Seconds!
	ramp->finalTargetRPM = targetRPM;
	ramp->instTargetRPM_F = 0;
	ramp->instTargetRPM_s16M = 0;
	//For RampUp
	totalSteps = ramp->rampUpTime_ms/ramp->ramp_callingTime_ms;
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dRPM_F_RU = ((float)ramp->finalTargetRPM)/totalSteps;

	//For RampDown
	totalSteps = ramp->rampDownTime_ms/ramp->ramp_callingTime_ms;
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dRPM_F_RD = ((float)ramp->finalTargetRPM)/totalSteps;

	ramp->rampPhase = RAMP_WAIT;

	ramp-> transitionTarget = 0;
	ramp-> transitionTime_ms = 0;
	ramp-> dDuty_F_transition = 0;

}
void StartRampRPM(RampRPM *ramp){
	ramp->rampPhase = RAMP_UP;
}

void StartRampDownRPM(RampRPM *ramp){
	ramp->rampPhase = RAMP_DOWN;
}


void StopRampRPM(RampRPM *ramp){
	ramp->rampPhase = RAMP_OVER;
}

void IdleRampRPM(RampRPM *ramp){
	ramp->rampPhase = RAMP_WAIT;
	ramp->instTargetRPM_F = 0;
	ramp->instTargetRPM_s16M = 0;
}

void ResetRampRPM(RampRPM *ramp){
	uint16_t totalSteps  = 0;
	ramp->ramp_callingTime_ms = 20;
	ramp->rampUpTime_ms = 10;
	ramp->rampDownTime_ms = 10;
	ramp->steadyRunTime_s = 30; // in Seconds!
	ramp->finalTargetRPM = 500;
	ramp->instTargetRPM_F = 0;
	ramp->instTargetRPM_s16M = 0;
	//For RampUp
	totalSteps = ramp->rampUpTime_ms/ramp->ramp_callingTime_ms;
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dRPM_F_RU = ((float)ramp->finalTargetRPM)/totalSteps;

	//For RampDown
	totalSteps = ramp->rampDownTime_ms/ramp->ramp_callingTime_ms;
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dRPM_F_RD = ((float)ramp->finalTargetRPM)/totalSteps;

	ramp->rampPhase = RAMP_WAIT;
}


void ExecRampRPM(RampRPM *ramp,TimerTypeDef *t){
	if (ramp->rampPhase == RAMP_UP){
		if(ramp->instTargetRPM_F <= ramp->finalTargetRPM){
			ramp->instTargetRPM_F += ramp->dRPM_F_RU;

			if(ramp->instTargetRPM_F >= ramp->finalTargetRPM){
				ramp->instTargetRPM_F  = ramp->finalTargetRPM;
				ramp->rampPhase = RAMP_STEADY;
				t->tim16_oneSecTimer = 0;
				t->tim16_20msTimer = 0;
			}
			ramp->instTargetRPM_s16M = (uint16_t)(ramp->instTargetRPM_F * RPM_TO_S16M) ;
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
			if(ramp->instTargetRPM_F < ramp->transitionTarget){
				ramp->instTargetRPM_F += ramp->dDuty_F_transition;
				if(ramp->instTargetRPM_F >= ramp->transitionTarget){
					ramp->instTargetRPM_F  = ramp->transitionTarget;
					ramp->rampPhase = RAMP_STEADY;
					t->tim16_oneSecTimer = 0;
					t->tim16_20msTimer = 0;
				}
			}
			if(ramp->instTargetRPM_F > ramp->transitionTarget){
				ramp->instTargetRPM_F -= ramp->dDuty_F_transition;
				if(ramp->instTargetRPM_F <= ramp->transitionTarget){
					ramp->instTargetRPM_F  = ramp->transitionTarget;
					ramp->rampPhase = RAMP_STEADY;
					t->tim16_oneSecTimer = 0;
					t->tim16_20msTimer = 0;
				}
			}

			ramp->instTargetRPM_s16M = (uint16_t)(ramp->instTargetRPM_F * RPM_TO_S16M) ;
		}

	else if (ramp->rampPhase == RAMP_DOWN){
		if(ramp->instTargetRPM_F >= 0){
			ramp->instTargetRPM_F -= ramp->dRPM_F_RD;
			if(ramp->instTargetRPM_F <= 0){
				ramp->instTargetRPM_F  = 0;
				ramp->rampPhase = RAMP_OVER;
				t->tim16_oneSecTimer = 0;
				t->tim16_20msTimer = 0;
			}
			ramp->instTargetRPM_s16M = (uint16_t)(ramp->instTargetRPM_F * RPM_TO_S16M) ;
		}
	}

}

void  ChangeRPM(RunMgmtTypeDef *rm, RampRPM *r){
	uint16_t deltaRPM = 0;
	uint16_t totalSteps = 0;
	r->transitionTarget = rm->transitionTarget;
	r->transitionTime_ms = rm->transitionTime_ms;
	r->finalTargetRPM = r->transitionTarget;
	if (r->transitionTarget > r->instTargetRPM_F){
		deltaRPM = r->transitionTarget - r->instTargetRPM_F;
	}else if (r->transitionTarget < r->instTargetRPM_F){
		deltaRPM = r->instTargetRPM_F - r->transitionTarget;
	}else{
		deltaRPM = 0;
	}
	totalSteps = r->transitionTime_ms/r->ramp_callingTime_ms;
	if (totalSteps == 0){
		totalSteps = 1;
	}
	r->dDuty_F_transition = ((float)deltaRPM)/totalSteps;
}

// when we get a change RPM for the bobbin, we want to do ramp UP/ramp Down times with that
// new target.
void Recalculate_RampRPM_RampRates(RunMgmtTypeDef *rm, RampRPM *ramp,uint16_t newTarget){
	uint16_t totalSteps = 0;
	//For RampUp
	ramp->finalTargetRPM = newTarget;
	totalSteps = ramp->rampUpTime_ms/ramp->ramp_callingTime_ms;
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dRPM_F_RU = ((float)ramp->finalTargetRPM)/totalSteps;

	//For RampDown
	totalSteps = ramp->rampDownTime_ms/ramp->ramp_callingTime_ms;
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dRPM_F_RD = ((float)ramp->finalTargetRPM)/totalSteps;
}
