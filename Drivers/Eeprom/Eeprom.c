/*
 * Eeprom.c
 *
 *  Created on: 05-Mar-2023
 *      Author: harsha
 */

#include "Eeprom.h"
#include "string.h"
#include "main.h"


extern I2C_HandleTypeDef hi2c1;


char integerArray[2]  = {0};
uint8_t floatArray[4]  = {0};

uint8_t EE_WriteInteger(uint16_t integer,uint16_t memoryLocation){
	integerArray[0] = (integer >> 8) & 0xFF;
	integerArray[1] = integer & 0xFF;
	HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, memoryLocation, I2C_MEMADD_SIZE_8BIT, (uint8_t*)integerArray,2,HAL_MAX_DELAY);
	HAL_StatusTypeDef OK = HAL_I2C_IsDeviceReady(&hi2c1, EEPROM_ADDRESS, 20, 100);
	if (OK == HAL_OK){
		return 1;
	}else{
		return 0;
	}
}

//send out only uint16_t , you need to make it a signed int if needed
uint16_t EE_ReadInteger(uint16_t memoryLocation){
	uint16_t out1 = 0;
	HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS, memoryLocation, I2C_MEMADD_SIZE_8BIT, (uint8_t*)integerArray,2, HAL_MAX_DELAY);
	HAL_I2C_IsDeviceReady(&hi2c1, EEPROM_ADDRESS, 20, 100);
	out1 = (integerArray[0]& 0xFF)<< 8;
	out1 += integerArray[1];
	return out1;
}

void float2Bytes(float val,uint8_t* bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    uint8_t temp_array[4];
  } u;
  // Overwrite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 4);
}

float bytes2Float(uint8_t* bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    uint8_t temp_array[4];
  } u;
  // Overwrite bytes of union with float variable
  memcpy(u.temp_array,bytes_array, 4);
  return u.float_variable;

}


uint8_t EE_WriteFloat(float floatVal,uint16_t memoryLocation){
	float2Bytes(floatVal,floatArray);
	HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, memoryLocation, I2C_MEMADD_SIZE_8BIT, (uint8_t*)floatArray,4,HAL_MAX_DELAY);
	HAL_StatusTypeDef OK = HAL_I2C_IsDeviceReady(&hi2c1, EEPROM_ADDRESS,40, 200);
	if (OK == HAL_OK){
		return 1;
	}else{
		return 0;
	}
}

float EE_ReadFloat(uint16_t memoryLocation){
	float out1 = 0;
	HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS, memoryLocation, I2C_MEMADD_SIZE_8BIT, (uint8_t*)floatArray,4, HAL_MAX_DELAY);
	HAL_I2C_IsDeviceReady(&hi2c1, EEPROM_ADDRESS, 20, 100);
	out1 = bytes2Float(floatArray);
	return out1;
}
