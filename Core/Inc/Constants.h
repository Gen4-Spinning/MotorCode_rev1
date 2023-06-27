/*
 * Constants.h
 *
 *  Created on: 23-Feb-2023
 *      Author: harsha
 */

#ifndef INC_CONSTANTS_H_
#define INC_CONSTANTS_H_

//MOTOR STATEs
#define IDLE_STATE 1
#define RUN_STATE 2
#define ERROR_STATE 3
#define CONFIG_STATE 4
#define CALIBRATION_STATE 5

//definition of RUN/STOP/PAUSE COMMANDS
#define EMERGENCY_STOP 1 // Emergency Stop
#define START 2 		// Ramp and start
#define RAMPDOWN_STOP 3 // RAMP and stop
#define CHANGE_RPM 4 // change rpm to new rpm
#define RESUME_AFTER_PAUSE 6 //
#define NO_MSG 0

//Run MGMT Vars
#define NOT_RUNNING 0
#define NORMAL_RUN 1
#define DIAGNOSIS_RUN 2

//constants for ensuring we have updated RM struct before we send the Start/changeTarget commands
#define NO_RM_MSG 0
#define RECEIVED_RAMP_SETTINGS 1
#define RECEIVED_CHANGE_RPM_SETTINGS 2


#define RAMP_RPM 1
#define RAMP_DUTY 2
#define STEP_RPM 3
#define STEP_DUTY 4

#define OPEN_LOOP 1
#define CLOSED_LOOP 2

#define NO_LOG 0
#define RUNTIME_LOG 1
#define ANALYSIS_LOG 2

#define RUN_FOREVER -1 // for normal operation, to keep running till you get a pause. else a time

/*--------------*/

#define MAX_PWM  1499

#define MAXRPM 2000
#define RPM_TO_S16M 32
#define S16M_TO_RPM 0.03125

#define CW 0  // DONT CHANGE BECAUSE ENC direction is only 0-1 and we want to use the same vars there
#define CCW 1

//when running a Diag Command
#define DEFAULT_DIR_COMMAND 0
#define REVERSE_DIR_COMMAND 1

//ADC Defines
#define BUS_VOLTAGE_GAIN 0.017 //3.3/(0.0476*4095)
#define BUS_CURRENT_GAIN 0.00672 //3.3/(20*4095*0.006)
//Analog Watchdog Thresholds are calculated from the excel.
//Voltage Values are set at Vmax = 50,Vmin =40;
//Over current Value set at 12A


#endif
