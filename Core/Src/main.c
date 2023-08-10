/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "Struct.h"
#include "Constants.h"
#include "ErrorShiftVals.h"

#include "sixSector.h"
#include "Ramp.h"
#include "PID.h"

#include "Eeprom.h"
#include "EepromFns.h"
#include "EepromSettings.h"

#include "FDCAN.h"
#include "CAN_Motor.h"

#include "AS5x47P.h"
#include "EncoderFns.h"
#include "EncoderCalibration.h"
#include "EncSpeed.h"
#include "temperatureLUT.h"

#include "Console.h"

#include <stdio.h>
#include <math.h> // for fabs
#include "retarget.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

DAC_HandleTypeDef hdac3;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

setup_typeDef setup;
runtimeVarsTypeDef R;
settingVar sV;
console C;
ErrorsTypeDef E;
StateTypeDef S;
TimerTypeDef T;
EncCalib_TypeDef encCalib;
EncSpeed_TypeDef encSpeed;
PID_Typedef PID;

//Structs to Control the motor.
RunMgmtTypeDef RM;
RampRPM rampRPM;
RampDuty rampDuty;
sixSector sixSectorObj;
sixSectorCntrl sixSectorCntrl_Obj;


//ADC variables here
uint16_t ADC1_buff[2]={0,2008},ADC2_buff[2];

//CAN variables here
FDCAN_RxHeaderTypeDef   RxHeader;
uint8_t               RxData[16];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC3_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM15_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char UART_buffer[60];

uint8_t dbg_encoder = 0;
uint8_t dbg_rampDuty_RUStart = 0;
uint8_t dbg_rampDuty_RDStart = 0 ;
uint8_t dbg_rampDuty_ChangeDuty = 0;
uint8_t dbg_rampDuty_Stop = 0;

uint8_t dbg_rampRPM_RUStart = 0;
uint8_t dbg_rampRPM_RDStart = 0;
uint8_t dbg_rampRPM_ChangeRPM = 0;
uint8_t dbg_rampRPM_Stop = 0;


uint8_t doCalibration =0;
uint16_t indexValue = 0;

//scanf variables
char scan_string[10], confirmation_char, configuration_mode_flg;
uint8_t start_string[3];
uint8_t AMS_manual_write_success = 0;
float ABI_mechAngle;
float SPI_mechAngle;

uint8_t motThermErrorFilter = 0;
uint8_t mosfetThermErrorFilter = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* TIM1 is the PWM timer, this interrupt fires at the pwmFreq.
	 * We use it to detect the sector to do the six Sector
	 */
	if (htim->Instance==TIM1){
	  uint16_t encoderCNT = htim2.Instance->CNT;
	  CalcSector_fromEncoder(&sixSectorObj,encoderCNT);
	  //if Error Flag is 1, calibration State or not, we stop all the PWMs.
	  if (E.errorFlag==0){
		  if ((sixSectorCntrl_Obj.turnOn == 1 )&& (sixSectorCntrl_Obj.duty > 14)){
			  if (sixSectorCntrl_Obj.direction == CW){
				  sixSectorCommutateCW(&sixSectorObj,0,500);
			  }
			  if (sixSectorCntrl_Obj.direction == CCW){
				  sixSectorCommutateCCW(&sixSectorObj,0,500);
			  }
		  }
		  else{
			  // if calibration state we done want to turn off all the Channels.
			  if ((S.motorState != CALIBRATION_STATE)&&(S.motorState != CONSOLE_STATE)){
				  TurnOffAllChannels();
			  }
		  }
	  }
	  else{
		  TurnOffAllChannels();
	  }
	}

	if (htim->Instance==TIM16){  // this is the 20ms timer
		if (RM.controlType == OPEN_LOOP){
			if (rampDuty.rampPhase!=RAMP_OVER){
				ExecRampDuty(&rampDuty,&T);
				sixSectorSetDuty(&sixSectorCntrl_Obj,rampDuty.currentDuty);
			}else if (rampDuty.rampPhase == RAMP_OVER){
				StopSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj);
				ResetRampDuty(&rampDuty); // resets everything and moves the rampPhase to RAMP_Wait
				if ((RM.runType == DIAGNOSIS_RUN) && (S.emergencyStop == 0)){
					FDCAN_sendDiagDoneFrame();
				}
				S.motorState = IDLE_STATE;
				RM.logReturn=0;
				RM.logCycle = 0;
			}
			R.motor_state = S.motorState;
			R.appliedDuty = rampDuty.currentDuty;
			R.presentRPM = encSpeed.speedRPM;
			if ((C.logging)|| (S.logginginternalEnable)){ //Console Logging.
				if ((encSpeed.zeroSpeed == 1) && (T.tim16_oneSecTimer > 3)){ // need the three seconds otherwise encspeed is zero when u start and logging immediately stops.
					C.logging = 0;
				}
				sprintf(UART_buffer,"D:%03d,%04d,%04d,%5.3f,%4.2f:E\r\n",T.tim16_oneSecTimer,R.appliedDuty,R.presentRPM,R.currentAmps,R.voltageVolts);
				HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,31);
			}
		}

		if (RM.controlType == CLOSED_LOOP){ 			//ClosedLoop
			if ((rampRPM.rampPhase!=RAMP_OVER)&&(rampRPM.rampPhase!=RAMP_WAIT)){
				ExecRampRPM(&rampRPM,&T);
				ExecPID(&PID,&rampRPM,&encSpeed);
				if(fabs(PID.errorF) > 100){ // TODO - find the best threshold for this.
					E.errorFlag=E.trackingError= 1;
					R.motorError|=1<<ERR_TE_SHIFT;
				}
				sixSectorSetDuty(&sixSectorCntrl_Obj,PID.pwm);
			}else if (rampRPM.rampPhase == RAMP_OVER){
				StopSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj);
				IdleRampRPM(&rampRPM); //moves the rampPhase to RAMP_Wait. Doesnt reset  the vars incase we want to restart
				resetPID(&PID);
				if ((RM.runType == DIAGNOSIS_RUN) && (S.emergencyStop == 0)){
					FDCAN_sendDiagDoneFrame();
				}
				S.motorState = IDLE_STATE;
				RM.logReturn=0;
				RM.logCycle = 0;
			}
			R.motor_state = S.motorState;
			R.appliedDuty = PID.pwm;
			R.targetRPM = (uint16_t)(rampRPM.instTargetRPM_F);
			R.presentRPM = encSpeed.speedRPM;
			R.proportionalTerm = PID.Kp_term;
			R.IntegralTerm = PID.Ki_term;
			R.feedforwardTerm = PID.feedForwardTerm;

			if ((C.logging) || (S.logginginternalEnable)){ //Console Logging.
				if ((encSpeed.zeroSpeed == 1) && (T.tim16_oneSecTimer > 3)){
					C.logging = 0;
				}
				sprintf(UART_buffer,"D:%03d,%04d,%04d,%04d,%05d,%05d,%04d,%5.3f,%4.2f:E\r\n",T.tim16_oneSecTimer,R.targetRPM,R.presentRPM,R.appliedDuty,R.proportionalTerm,R.IntegralTerm,R.feedforwardTerm,R.currentAmps,R.voltageVolts);
				HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,53);
			}
		}

		if(sixSectorCntrl_Obj.turnOn == 1){
			RM.logCycle ++; // send data every 60 ms.
			if (RM.logCycle == 3){
				if (RM.logReturn == RUNTIME_LOG){
					FDCAN_runtimedataFromMotor();
				}else if (RM.logReturn == ANALYSIS_LOG){
					FDCAN_analysisdataFromMotor();
				}
				RM.logCycle = 0;
			}
		}

		// we Time a one sec counter over here;
		T.tim16_20msTimer++;
		if (T.tim16_20msTimer >= 50){
			T.tim16_oneSecTimer ++;
			T.tim16_20msTimer = 0;
		}
	}

/*	if (htim->Instance==TIM17){ //100 ms timer. Use this to check the encoder State
		E.motorEncoderError = AS5047_checkEncoderHealth(); //tODO why is this false triggering?
		if (E.motorEncoderError == 1){
			TurnOffAllChannels();
			E.errorFlag=1;
			R.motorError|=E.motorEncoderError<<11;
			//HAL_TIM_Base_Stop_IT(&htim17);
		}
	}*/

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  if(hadc==&hadc1)
  {
	  R.busVoltageADC=ADC1_buff[0];
	  R.voltageVolts=(BUS_VOLTAGE_GAIN*ADC1_buff[0]);
	  R.MOTtemp=32;//get_MOSFET_temperature(ADC1_buff[1]);//MOSFET LUT is used because beta value is almost the same, 0.5 deg difference

	  if(R.MOTtemp>125 && E.motorOvertemperature==0 ){
		  E.motorOvertemperature=E.errorFlag=1;
		  R.motorError|=E.motorOvertemperature<<ERR_MOT_SHIFT;
	  }

	  if((ADC1_buff[1]<50 || ADC1_buff[1]>2300)){ // open thermistor fault
		  // look for ten consecutive cycles
		  motThermErrorFilter ++;
		  if (motThermErrorFilter >= 10){
			  E.motorThermistorFault=E.errorFlag=1;
			  R.motorError|=E.motorThermistorFault<<ERR_MTF_SHIFT;
		  }
	  }else{
		  motThermErrorFilter = 0;
	  }
  }
  else
  {
	  R.busCurrentADC=ADC2_buff[0];
	  R.currentAmps=(0.9*R.prevcurrentAmps)+(0.1*BUS_CURRENT_GAIN*((float)ADC2_buff[0]));
	  R.prevcurrentAmps=R.currentAmps;
	  R.FETtemp=get_MOSFET_temperature(ADC2_buff[1]);

	  if(R.FETtemp>110 && E.fetOvertemperature==0 ){
		  E.fetOvertemperature=E.errorFlag=1;
		  R.motorError|=1<<ERR_FOT_SHIFT;
	  }
	  if((ADC2_buff[1]<50 || ADC2_buff[1]>2300)){
		  mosfetThermErrorFilter ++;
		  if (mosfetThermErrorFilter >= 10){
			  E.fetThermistorFault=E.errorFlag=1;
			  R.motorError|=1<<ERR_FTF_SHIFT;
		  }
	  }else{
		  mosfetThermErrorFilter = 0;
	  }
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retreive Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
    	FDCAN_parseForMotor(S.CAN_ID);
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      /* Notification Error */
      Error_Handler();
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if ((S.motorState == IDLE_STATE) || (S.motorState == ERROR_STATE)){
		if((start_string[0]=='$')&&(start_string[1]=='$')&&(start_string[2]=='$')){
			configuration_mode_flg=1;
			start_string[0]=start_string[1]=start_string[2]=0;
		}else{
			HAL_UART_Receive_IT(&huart3, start_string, 3);
		}
	}
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc){
	TurnOffAllChannels();
	HAL_TIM_Base_Stop_IT(&htim4);
	HAL_TIM_Base_Stop_IT(&htim15);
	E.errorFlag=1;
	if (hadc == &hadc1){
		if(ADC1_buff[0]>2940){
			E.overvoltage=1;
			R.motorError|=1<<ERR_OV_SHIFT;
		}
		else{
			E.undervoltage=1;
			R.motorError|=1<<ERR_UV_SHIFT;
		}
	}
	//TODO:TAKE current limit into EEprom?
	if (hadc == &hadc2){
		E.overcurrent=1;
		R.motorError|=1<<ERR_OC_SHIFT;
	}

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_DAC3_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM17_Init();
  MX_TIM7_Init();
  MX_TIM16_Init();
  MX_TIM15_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  InitializeSetup_TypeDef(&setup);
  InitializeSettingsObj(&sV); // initialize the settings obj
  InitializeRunTime_TypeDef(&R);
  InitializeState_TypeDef(&S);
  InitializeEncoderCalib_TypeDef(&encCalib);
  InitializeRunMgmt_TypeDef(&RM);
  InitializeTimer_TypeDef(&T);
  InitializeEncSpeed_TypeDef(&encSpeed);
  InitializePID_TypeDef(&PID);

  //Calibrate ADCs
  HAL_Delay(100);
  HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
  HAL_Delay(50);
  HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
  HAL_Delay(400);

  // NOW start the Timers for PWM
  HAL_TIM_Base_Start_IT(&htim1);

  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);

  TurnOffAllChannels();

  //Read EEprom and check values.
  readSettingsFromEEPROM(&sV);//read values from EEPROM
  setup.eepromMotorValsGood = checkEEPROM_MotorSettings(&sV);
  setup.eepromPWMValsGood = checkEEPROM_PWMSettings(&sV);

  //Comment for Production
  //setup.eepromMotorValsGood = 0;
  if (setup.eepromMotorValsGood == 0){
	  AMS_manual_write_success = writeMotorSettingsToEEPROM_Manual(RD_CALENDER_ADDRESS,9199,CCW);
	  sV.AMS_offset_index = 9199;//2837
	  sV.MOTID = RD_CALENDER_ADDRESS;
	  sV.default_direction = CW;
	  //force user to set the correct values before he continues.
	  // there are no 'default' settings
  }
  //setup.eepromMotorValsGood = 1;

  //TODO : startoffset cant be anything other than 0 right now. FIX
  //setup.eepromPWMValsGood = 0;
  if (setup.eepromPWMValsGood == 0){
	  sV.Kp = 0.023;
	  sV.Ki = 0.001;
	  sV.ff_percent = 60;
	  sV.start_offset = 0;
	  //loadPWMDefaultSettings(&settingVarObj);
	  setup.defaults_eepromWriteFailed += writePWMSettingsToEEPROM(&sV);
  }
  //setup PID with the correct Settings.
  setupPID(&PID,sV.Kp,sV.Ki,sV.ff_percent,sV.start_offset);

  //Now setup the Encoder.Use whatever value you got from the eeprom, even if its bad.
  setup.encoderSetupOK = setupMotorEncoder_inABI_Mode(); //setup ABI mode without PWM
  setup.encoderZeroValueOK = updateEncoderZeroPosition(sV.AMS_offset_index);// set the Zero position
  setup.encoderMagErrorSetupOK = AS5047_EnableMagErrors();

  uint16_t startingCNT_val = getEncoderStartPosition(); // get the current angle and ..
  __HAL_TIM_SET_COUNTER(&htim2,startingCNT_val); // set it in the timer CNT register.
  HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_ALL); //start the encoder

  // Now that all the setup is then, we check the values and set an error Flag if there is an issue
  if ((setup.eepromMotorValsGood == 0 ) || (setup.eepromPWMValsGood == 0)){
	  S.motorSetupFailed = E.errorFlag = E.eepromBadValueError = 1;
	  R.motorError = 1 << ERR_EBV_SHIFT;
  }
  if (setup.defaults_eepromWriteFailed == 1 ){
	  S.motorSetupFailed = E.errorFlag = E.eepromWriteError = 1;
	  R.motorError = 1 << ERR_EWE_SHIFT;
  }

  if ((setup.encoderSetupOK == 0 ) || (setup.encoderZeroValueOK == 0 ) || (setup.encoderMagErrorSetupOK == 0)){
	  S.motorSetupFailed = E.errorFlag = E.motorEncoderSetupError = 1;
	  R.motorError = 1 << ERR_MES_SHIFT;
  }

  // Initialize all the sixSector Structs
  sixSectorInit(&sixSectorObj);
  sixSectorCntrlInit(&sixSectorCntrl_Obj);
  updateSectorBoundaries(&sixSectorObj,CCW);

  //Start 2 Timers - one for speed calculation and one for ramp calculation
  HAL_TIM_Base_Start_IT(&htim7); // 1 ms interrupt for speed calculation, speed is averaged over 16 readings.
  HAL_TIM_Base_Start_IT(&htim16);  //20ms timer for ramp

  //SetUP CAN
  S.CAN_ID= sV.MOTID ;//settingVarObj.MOTID; // should come from the EEprom
  MX_FDCAN1_Init();
  FDCAN_TxInit(); // this has to be here.

  //For configuration through the UART
  RetargetInit(&huart3);
  HAL_UART_Receive_IT(&huart3, start_string, 3);

  //Start the ADCs(after CANID is set)
  HAL_TIM_Base_Start_IT(&htim4);//start timer for adc1 trigger
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC1_buff,2);
  HAL_TIM_Base_Start_IT(&htim15);//start timer for adc2 trigger
  HAL_ADC_Start_DMA(&hadc2,(uint32_t*)ADC2_buff,2);

  //start the 100ms timer used to check encoder Health
  //HAL_TIM_Base_Start_IT(&htim17);

   S.motorState = IDLE_STATE; // always start from IDLE
  //TODO DONT ALLOW TO START IF MOTOR IS STILL ROTATING

  HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //send all Error MSgs every 1 sec here
	  if (E.errorFlag == 1){
		  S.motorState = ERROR_STATE;
		  RM.controlType = 0; // stop control loops
		  if (S.errorMsgSentOnce == 0){
			  FDCAN_errorFromMotor();
			  sprintf(UART_buffer,"Error!ErrorReason = %05d\r\n",R.motorError);
			  HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,27);
			  S.errorMsgSentOnce = 1;
		  }
		  // go inside the console and do things
		  //like come out of lob mode, and reset motor
		  if(configuration_mode_flg){
			  S.motorState = CONSOLE_STATE;
			  configurationFromTerminal(); // BLOCKING
			  configuration_mode_flg=0;
			  HAL_UART_Receive_IT(&huart3, start_string, 3);
			  S.motorState = IDLE_STATE;
		  }
	  }

	  /* logging always , except when your in CONSOLE STATE, or when encSpeed is zero */
	  if (S.loggingEnabled){
		  if (S.motorState == CONSOLE_STATE){
			  S.logginginternalEnable = 0;
		  }else if (encSpeed.zeroSpeed == 1){
			  S.logginginternalEnable = 0;
		  }else{
			  S.logginginternalEnable = 1;
		  }
	  }

	  //move motorState to CONFIGState when doing Console Work. will prevent running CAN-start/pause/halt msgs
	  //Also only allow when your in IDLE STATE
	  if (S.motorState == IDLE_STATE){
		  if(configuration_mode_flg){
			  S.motorState = CONSOLE_STATE;
			  configurationFromTerminal(); // BLOCKING
			  configuration_mode_flg=0;
			  HAL_UART_Receive_IT(&huart3, start_string, 3);
			  S.motorState = IDLE_STATE;
		  }

		  // only allow in IDLE_STATE
		  //TODO Improvement:in both directions
		  if (doCalibration == 1){
			 S.motorState = CALIBRATION_STATE;
			 uint8_t zeroed = updateEncoderZeroPosition(0);
			 uint8_t calibrationError = 0;
			 if (zeroed){
				 MX_TIM1_Init();
				 RunCalibration(); // BLOCKING
				 calibrationError = 0;
			 }else{
				 calibrationError = 1;
			 }
			 doCalibration = 0;
			 S.motorState = IDLE_STATE;
		  }

	  }

	  //----DBG functions to test with Laptop/ UART ---
	  if (S.motorState == IDLE_STATE){

		  if (dbg_encoder == 1){ // Read both ABI and SPI angles and see if they re the same
			  ABI_mechAngle = getEncoderAngleFromABI(&htim2);
			  SPI_mechAngle = getEncoderAngleFromSPI(1);
			  //dbg_encoder = 0;
		  }
		  //Fill up other RM in live watch
		  /*--------------------CLOSED LOOP -----------------------*/
		  if (dbg_rampRPM_RUStart){
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
			dbg_rampRPM_RUStart = 0;
		  }

		  if (dbg_rampRPM_RDStart){
			  StartRampDownRPM(&rampRPM);
			  dbg_rampRPM_RDStart= 0;
		  }

		  //Set transition Target and Transition Time in RM
		  if(dbg_rampRPM_ChangeRPM){
			  ChangeRPM(&RM,&rampRPM);
			  Recalculate_RampRPM_RampRates(&RM,&rampRPM,RM.transitionTarget);
			  rampRPM.rampPhase = RAMP_CHANGE;
			  dbg_rampRPM_ChangeRPM = 0;
		  }

		  //STOP CLOSED LOOP
		  if (dbg_rampRPM_Stop){
			  // we need to stop the six sector Obj, and then stop the Ramp
			  StopSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj);
			  //sixSectorCntrl_Obj.turnOn = 0;
			  StopRampRPM(&rampRPM);
			  dbg_rampRPM_Stop =0;
		  }

		  /*--------------------OPEN LOOP -----------------------*/
		  if (dbg_rampDuty_RUStart){
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
			dbg_rampDuty_RUStart = 0;
		  }

		  if (dbg_rampDuty_RDStart){
			  StartRampDownDuty(&rampDuty);
			  dbg_rampDuty_RDStart= 0;
		  }

		  //Set transition Target and Transition Time in RM
		  if(dbg_rampDuty_ChangeDuty){
			  ChangeDuty(&RM,&rampDuty);
			  Recalculate_RampDuty_RampRates(&RM,&rampDuty,RM.transitionTarget);
			  rampDuty.rampPhase = RAMP_CHANGE;
			  dbg_rampDuty_ChangeDuty = 0;
		  }

		  if (dbg_rampDuty_Stop){
			  // we need to stop the six sector Obj, and then stop the Ramp
			  StopSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj);
			  StopRampDuty(&rampDuty);
			  dbg_rampDuty_Stop =0;
		  }
	  }
	  /*-------------------------------------------------------*/

	  //MAIN CONTROL FUNCTIONS FROM CAN COMMANDS

	  /*if you get a resume when you are ramping down it will restart */
	  if ((S.CAN_MSG == START)|| (S.CAN_MSG == RESUME_AFTER_PAUSE)){
		  if (S.motorState == IDLE_STATE){
		      S.CAN_MSG = NO_MSG; // we ve used the MSG ,now discard it.
		      if ((RM.rampTarget != 0) && (RM.rampRampUpTime_s!= 0) && (RM.rampRampDownTime_s != 0)){
				  S.motorState = RUN_STATE;
				  RM.logReturn = RUNTIME_LOG;
				  StartSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj,RM.rotateDirection); // default rotate dir is set in the CAN parse Diag Frame
				  if (RM.controlType == OPEN_LOOP){
					  StartRampDuty(&rampDuty);
				  }
				  if (RM.controlType == CLOSED_LOOP){
					  StartRampRPM(&rampRPM);
				  }
				  S.emergencyStop = 0;
				  //need to reset the timer to allow for calculation of Time to take place
				  T.tim16_oneSecTimer = 0;
				  T.tim16_20msTimer = 0;
		      }
		  }else{
			  S.CAN_MSG = NO_MSG; // To prevent any repeated Starts
		  }
	  }

	  if (S.CAN_MSG == EMERGENCY_STOP){
		  if(S.motorState == RUN_STATE){
			  S.CAN_MSG = NO_MSG;
			  StopSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj);
			  if (RM.controlType == OPEN_LOOP){
				  StopRampDuty(&rampDuty);
			  }
			  if (RM.controlType == CLOSED_LOOP){
				  StopRampRPM(&rampRPM);
			  }
			  S.emergencyStop = 1;
			  //state becomes IDLE where the rpm becomes zero in the interrupt
		  }else{
			  S.CAN_MSG = NO_MSG;
		  }
	  }

	  if (S.CAN_MSG == RAMPDOWN_STOP){
		  if (S.motorState == RUN_STATE){
			  S.CAN_MSG = NO_MSG;
			  if (RM.controlType == OPEN_LOOP){
				  StartRampDownDuty(&rampDuty);
			  }
			  if (RM.controlType == CLOSED_LOOP){
				  StartRampDownRPM(&rampRPM);
			  }
			  //state becomes IDLE where the rpm becomes zero in the interrupt
		  }else{
			 S.CAN_MSG = NO_MSG;
		  }
	  }

	  if (S.CAN_MSG == CHANGE_RPM){
		  if(S.motorState == RUN_STATE){
			  S.RM_state = NO_RM_MSG;
			  S.CAN_MSG = NO_MSG;
			  if (RM.controlType == OPEN_LOOP){
				  ChangeDuty(&RM,&rampDuty);
				  Recalculate_RampDuty_RampRates(&RM,&rampDuty,RM.transitionTarget);
				  rampDuty.rampPhase = RAMP_CHANGE;
			  }
			  if (RM.controlType == CLOSED_LOOP){
				  ChangeRPM(&RM,&rampRPM);
				  Recalculate_RampRPM_RampRates(&RM,&rampRPM,RM.transitionTarget);
				  rampRPM.rampPhase = RAMP_CHANGE;
			  }
		  }else{
			 S.CAN_MSG = NO_MSG;
		  }

	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 75;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T4_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_4;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analog WatchDog 1
  */
  AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.Channel = ADC_CHANNEL_11;
  AnalogWDGConfig.ITMode = ENABLE;
  AnalogWDGConfig.HighThreshold = 198;
  AnalogWDGConfig.LowThreshold = 154;
  AnalogWDGConfig.FilteringConfig = ADC_AWD_FILTERING_NONE;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T15_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = ENABLE;
  hadc2.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_4;
  hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
  hadc2.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc2.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analog WatchDog 1
  */
  AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.Channel = ADC_CHANNEL_4;
  AnalogWDGConfig.ITMode = ENABLE;
  AnalogWDGConfig.HighThreshold = 93;
  AnalogWDGConfig.LowThreshold = 0;
  AnalogWDGConfig.FilteringConfig = ADC_AWD_FILTERING_NONE;
  if (HAL_ADC_AnalogWDGConfig(&hadc2, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC3_Init(void)
{

  /* USER CODE BEGIN DAC3_Init 0 */

  /* USER CODE END DAC3_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC3_Init 1 */

  /* USER CODE END DAC3_Init 1 */

  /** DAC Initialization
  */
  hdac3.Instance = DAC3;
  if (HAL_DAC_Init(&hdac3) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC3_Init 2 */

  /* USER CODE END DAC3_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV10;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = ENABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 6;
  hfdcan1.Init.NominalTimeSeg1 = 23;
  hfdcan1.Init.NominalTimeSeg2 = 6;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 3;
  hfdcan1.Init.DataTimeSeg1 = 11;
  hfdcan1.Init.DataTimeSeg2 = 3;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  FDCAN_RxFilterInit();
  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x70A06EFF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

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

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 2048;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 29999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 99;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1499;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 1499;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 99;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 1999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1499;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 1499;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 9999;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|DAC_DBG_Pin|DBG_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin|FAULT_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_INDEX_Pin */
  GPIO_InitStruct.Pin = ENC_INDEX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC_INDEX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 DAC_DBG_Pin DBG_OUT_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|DAC_DBG_Pin|DBG_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS_Pin FAULT_LED_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin|FAULT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
