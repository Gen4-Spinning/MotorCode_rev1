/*
 * EepromFns.c
 *
 *  Created on: 07-Mar-2023
 *      Author: Jonathan
 */
#include "Struct.h"
#include "Eeprom.h"
#include "EepromSettings.h"
#include "EepromFns.h"

void readSettingsFromEEPROM(settingVar *sV)
{
	sV->Kp=EE_ReadFloat(Kp_ADDRESS);
	sV->Ki=EE_ReadFloat(Ki_ADDRESS);
	sV->start_offset=EE_ReadInteger(START_OFFSET_ADDRESS);
	sV->ff_percent=EE_ReadInteger(FEED_FORWARD_FACTOR_ADDRESS);
	sV->AMS_offset_index=EE_ReadInteger(OFFSET_INDEX_ADDRESS);
	sV->MOTID=EE_ReadInteger(MOTID_ADDRESS);
	sV->default_direction=EE_ReadInteger(DEFAULT_DIRECTION_ADDRESS);
}

void loadPWMDefaultSettings(settingVar *sV){
	// safe settings
	sV->Kp=0;
	sV->Ki=0;
	sV->start_offset=100;
	sV->ff_percent=10;
}

uint8_t writePWMSettingsToEEPROM(settingVar *stV)
{
	uint8_t dataWritten = 0;
	dataWritten += EE_WriteFloat(stV->Kp,Kp_ADDRESS);
	HAL_Delay(1);
	dataWritten += EE_WriteFloat(stV->Ki,Ki_ADDRESS);
	HAL_Delay(1);
	dataWritten += EE_WriteInteger(stV->start_offset,START_OFFSET_ADDRESS);
	HAL_Delay(1);
	dataWritten += EE_WriteInteger(stV->ff_percent,FEED_FORWARD_FACTOR_ADDRESS);
	HAL_Delay(1);
    if (dataWritten == 4)
    	{return 1;}
    else{
    	return 0;}
}


uint8_t writePWMSettingsToEEPROM_Manual(float Kp, float Ki,uint16_t ff_percent,uint16_t startOffset)
{
	uint8_t dataWritten = 0;
	dataWritten += EE_WriteFloat(Kp,Kp_ADDRESS);
	HAL_Delay(1);
	dataWritten += EE_WriteFloat(Ki,Ki_ADDRESS);
	HAL_Delay(1);
	dataWritten += EE_WriteInteger(startOffset,START_OFFSET_ADDRESS);
	HAL_Delay(1);
	dataWritten += EE_WriteInteger(ff_percent,FEED_FORWARD_FACTOR_ADDRESS);
	HAL_Delay(1);
    if (dataWritten == 4)
    	{return 1;}
    else{
    	return 0;}
}

uint8_t writeMotorSettingsToEEPROM_Manual(int8_t motorID, int16_t AMS_offset,int16_t default_direction){
	uint8_t dataWritten = 0;
	uint8_t target = 3;
	if (motorID != -1){
		dataWritten += EE_WriteInteger((uint16_t)motorID,MOTID_ADDRESS);
		HAL_Delay(1);
	}else{
		target -= 1;
	}

	if (AMS_offset != -1){
		dataWritten += EE_WriteInteger(AMS_offset,OFFSET_INDEX_ADDRESS);
		HAL_Delay(1);
	}else{
		target -= 1;
	}

	if (default_direction != -1){
		dataWritten += EE_WriteInteger(default_direction,DEFAULT_DIRECTION_ADDRESS);
		HAL_Delay(1);
	}else{
		target -=1;
	}

	if (dataWritten == target)
		{return 1;}
	else{
		return 0;
	}
}

uint8_t writeMotorSettingsToEEPROM(settingVar *stV){
	uint8_t dataWritten = 0;
	dataWritten += EE_WriteInteger(stV->MOTID,MOTID_ADDRESS);
	HAL_Delay(1);
	dataWritten += EE_WriteInteger(stV->AMS_offset_index,OFFSET_INDEX_ADDRESS);
	HAL_Delay(1);
	dataWritten += EE_WriteInteger(stV->default_direction,DEFAULT_DIRECTION_ADDRESS);
	HAL_Delay(1);
	if (dataWritten == 3)
		{return 1;}
	else{
		return 0;
	}
}

uint8_t checkEEPROM_PWMSettings(settingVar *sV)
{
	uint8_t EEPROM_check=1;
	if(sV->Kp>5.0 || sV->Kp<0.000)
	{
		EEPROM_check=0;
	}
	if(sV->Ki>5.0 || sV->Ki<0.000)
	{
		EEPROM_check=0;
	}
	if(sV->start_offset>200)
	{
		EEPROM_check=0;
	}
	if(sV->ff_percent>80)
	{
		EEPROM_check=0;
	}
	return EEPROM_check;
}

uint8_t checkEEPROM_MotorSettings(settingVar *sV){
	uint8_t EEPROM_check=1;
	if(sV->MOTID>10)
	{
		EEPROM_check=0;
	}
	if(sV->AMS_offset_index>16384)
	{
		EEPROM_check=0;
	}
	if(sV->default_direction>2)
	{
		EEPROM_check=0;
	}
	return EEPROM_check;

}


