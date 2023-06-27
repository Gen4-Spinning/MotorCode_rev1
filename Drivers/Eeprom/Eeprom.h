/*
 * Eeprom.h
 *
 *  Created on: 05-Mar-2023
 *      Author: harsha
 */

#ifndef EEPROM_EEPROM_H_
#define EEPROM_EEPROM_H_

#include "stm32g4xx_hal.h"
#include "stdio.h"

#define EEPROM_ADDRESS	0xA0

uint8_t EE_WriteInteger(uint16_t integer,uint16_t memoryLocation);
uint16_t EE_ReadInteger(uint16_t memoryLocation);
void float2Bytes(float val,uint8_t* bytes_array);
float bytes2Float(uint8_t* bytes_array);

uint8_t EE_WriteFloat(float floatVal,uint16_t memoryLocation);
float EE_ReadFloat(uint16_t memoryLocation);

#endif /* EEPROM_EEPROM_H_ */
