/*
 * ErrorShiftVals.h
 *
 *  Created on: May 8, 2023
 *      Author: harsha
 */

#ifndef INC_ERRORSHIFTVALS_H_
#define INC_ERRORSHIFTVALS_H_

#define ERR_OC_SHIFT 1	//over current
#define ERR_OV_SHIFT 2	// over Voltage
#define ERR_UV_SHIFT 3  // under Voltage
#define ERR_MTF_SHIFT 4	// motor Thermistor Fault
#define ERR_FTF_SHIFT 5	// Fet thermistor Fault
#define ERR_MOT_SHIFT 6	// motor over temperature
#define ERR_FOT_SHIFT 7 // fet over temperature
#define ERR_EWE_SHIFT 8 // eeprom writing error
#define ERR_EBV_SHIFT 9 // eeprom Bad Values
#define ERR_TE_SHIFT 10 // tracking error
#define ERR_MES_SHIFT 11 // motor encoder setup fail
#define ERR_LTE_SHIFT 12 // lift tracking error
#define ERR_LSF_SHIFT 13 // Lift GB-motorEncoder pos Calculation synchrocity fail
#define ERR_LOB_SHIFT 14 // Lift out of bounds
#define ERR_EBH_SHIFT 15 // eeprom Bad homing values



#endif /* INC_ERRORSHIFTVALS_H_ */
