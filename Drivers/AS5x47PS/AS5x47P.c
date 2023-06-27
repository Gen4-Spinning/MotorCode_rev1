#include "main.h"
#include "AS5x47P.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
extern SPI_HandleTypeDef hspi1;

uint16_t transmitVar;
uint8_t txbuf[2];
uint8_t rxbuf[2];
uint8_t finalrxbuf[2];
uint16_t finalDataVal;
float readdata;


uint8_t checkReadForError(uint16_t data){
	return (data & 0x4000)>>15; // check if 15th bit is 1.
}

// Check Parity of a given data.
bool parityCheck(uint16_t data){
  uint16_t count=0;
  uint16_t b = 1;
  for (int i=0; i<15; i++){
    if (data & (b << i)) {
      count++;
    }
  }

  if (count%2==0) {
    return 0;
  } else {
    return 1;
  }
}

//SPI_write and read are polling Fns;Timeout is set for 100ms
uint8_t AS5047_SPI_Write(uint16_t addressFrame, uint16_t valueFrame) {

  //write which address needed to be updated.
  txbuf[0] = (addressFrame) >> 8;
  txbuf[1] = addressFrame & 0xFF;
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&txbuf, (uint8_t*)&rxbuf, 2, 100);
  while( hspi1.State == HAL_SPI_STATE_BUSY );
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  //this is value that will be stored inside the respective register
  txbuf[0] = (valueFrame) >> 8;
  txbuf[1] = valueFrame & 0xFF;
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&txbuf, (uint8_t*)&rxbuf, 2, 100);
  while( hspi1.State == HAL_SPI_STATE_BUSY );
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  //check if data has been written properly, will receive the written value in the recieve buffer
  txbuf[0] = NOP_FRAME >> 8;
  txbuf[1] = NOP_FRAME & 0xFF;
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&txbuf, (uint8_t*)&rxbuf, 2, 100);
  while( hspi1.State == HAL_SPI_STATE_BUSY );
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  //DEBUG WHY THIS IS NOT WORKING LATER.
  //check if recieved data is same as what we wanted to write
  uint16_t recievedFrame = ((uint16_t)rxbuf[0])<<8;
  recievedFrame += rxbuf[1];

  uint16_t receivedData = recievedFrame & 0x3FFF;
  uint16_t writtenData = valueFrame & 0x3FFF;

  if ((writtenData) == receivedData){
	  return 1;
  }

  return 0;
}


uint16_t AS5047_SPI_Read(uint16_t command, uint8_t continuousRead) {
  //write command frame.
  uint16_t finalDataVal;
  txbuf[0] = command >> 8U;
  txbuf[1] = command & 0xFF;
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&txbuf, (uint8_t*)&rxbuf, 2,100);
  while( hspi1.State == HAL_SPI_STATE_BUSY );
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  if (continuousRead ==0){
	  //this is the NOP command frame for receiving data if you want to read the register once.
	  txbuf[0] = NOP_FRAME >> 8;
	  txbuf[1] = NOP_FRAME & 0xFF;
	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&txbuf, (uint8_t*)&rxbuf, 2,100);
	  while( hspi1.State == HAL_SPI_STATE_BUSY );
	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  }
  finalDataVal = ((uint16_t)rxbuf[0])<<8;
  finalDataVal += rxbuf[1];

  return finalDataVal;

}

// in the SPI interrupt we will recieve the data, and we need to SET the CS port.
//you hve to handle the data analysis part given data is in the interrupt.
void AS5047_SPI_Read_IT(uint16_t command){
	txbuf[0] = command >> 8U;
	txbuf[1] = command & 0xFF;
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)&txbuf, (uint8_t*)&rxbuf, 2);
}


uint16_t AS5047_readRegister(uint16_t registerAddress,uint8_t continuousRead){
  CommandFrame command;
  command.values.rw = READ;
  command.values.commandFrame = registerAddress;
  command.values.parc = parityCheck(command.raw);
  uint16_t out= AS5047_SPI_Read(command.raw, continuousRead);
  return out;
}


void AS5047_writeRegister(uint16_t registerAddress, uint16_t registerValue) {
  CommandFrame command;
  command.values.rw = WRITE;
  command.values.commandFrame = registerAddress;
  command.values.parc = parityCheck(command.raw);

  WriteDataFrame contentFrame;
  contentFrame.values.data = registerValue;
  contentFrame.values.low = 0; //always low
  contentFrame.values.pard = parityCheck(contentFrame.raw);
  uint8_t out = AS5047_SPI_Write(command.raw, contentFrame.raw);
}

uint16_t AS5047_ReadZeroValue(void){
  ZPOSH_frame ZPOS_H;
  ZPOSL_frame ZPOS_L;

  ZPOS_H.raw = AS5047_readRegister(ZPOSM_REG,0);
  ZPOS_L.raw = AS5047_readRegister(ZPOSL_REG,0);

  uint16_t zpos = (ZPOS_H.raw << 6) + (ZPOS_L.raw & 0x3F);
  return zpos;
}


void AS5047_WriteZeroValue(uint16_t zeroValue){
  ZPOSH_frame ZPOS_H;
  ZPOSL_frame ZPOS_L;

  ZPOS_L.values.zposl = zeroValue & 0x003F;
  ZPOS_H.values.zposh = (zeroValue >> 6) & 0x00ff;

  AS5047_writeRegister(ZPOSM_REG, ZPOS_H.raw);
  AS5047_writeRegister(ZPOSL_REG, ZPOS_L.raw);

}


/************************/

uint16_t readRegister16(uint16_t address){
  CommandFrame command;
  command.values.rw = READ;
  command.values.commandFrame = address;
  command.values.parc = parityCheck(command.raw);

  uint16_t out = readData(command.raw, NOP_FRAME);
  return out;
}

uint16_t readData(uint16_t command, uint16_t nopCommand) {

  //write command frame.
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  transmitVar = command;
  txbuf[0] = transmitVar >> 8U;
  txbuf[1] = transmitVar & 0xFF;
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&txbuf, (uint8_t*)&rxbuf, 2, 100);
  while( hspi1.State == HAL_SPI_STATE_BUSY );
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

/*  for (int i=0;i<40;i++){
   asm ("NOP");
  }*/

  //this is NOP command frame for receiving data
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  transmitVar = nopCommand;
  txbuf[0] = transmitVar >> 8U;
  txbuf[1] = transmitVar & 0xFF;
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&txbuf, (uint8_t*)&finalrxbuf, 2, 100);
  while( hspi1.State == HAL_SPI_STATE_BUSY );
  finalDataVal = ((uint16_t)finalrxbuf[0])<<8;
  finalDataVal += finalrxbuf[1];
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  return finalDataVal;

}

// Sending of address and value via SPI for writting data into register via SPI.
// The data that is given to this function call must be a cleaned and pre-processes one. (Containing alll the paritycheck, R/W bit etc)
// This is similar to that of readData Function call. It just transmits data. There is no cleaning process and such.
void writeData(uint16_t address, uint16_t value) {

  //write which address needed to be updated.
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  transmitVar = address;
  txbuf[0] = (transmitVar) >> 8;
  txbuf[1] = transmitVar & 0xFF;
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&txbuf, (uint8_t*)&rxbuf, 2, 100);
  while( hspi1.State == HAL_SPI_STATE_BUSY );
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  //this is value that will be stored inside the respective register
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  transmitVar = value;
  txbuf[0] = (transmitVar) >> 8;
  txbuf[1] = transmitVar & 0xFF;
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&txbuf, (uint8_t*)&finalrxbuf, 2, 100);
  while( hspi1.State == HAL_SPI_STATE_BUSY );
  finalDataVal = finalrxbuf[0] << 8;
  finalDataVal &= finalrxbuf[1];
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

}

// Preprocesses data before sending data via SPI
// Populating the CmdFrame is done here.
// See documentation for details on implementation
ReadDataFrame readRegister(uint16_t registerAddress){
  CommandFrame command;
  command.values.rw = READ;
  command.values.commandFrame = registerAddress;
  command.values.parc = parityCheck(command.raw);

  CommandFrame nopCommand;
  nopCommand.values.rw = READ;
  nopCommand.values.commandFrame = NOP_REG;
  nopCommand.values.parc = parityCheck(nopCommand.raw);
  ReadDataFrame receivedFrame;
  receivedFrame.raw = readData(command.raw, nopCommand.raw);
  return receivedFrame;
}

uint16_t getProgrammedZeroOffset(void){
  CommandFrame command;
  command.values.rw = READ;
  command.values.commandFrame = ZPOSM_REG;
  command.values.parc = parityCheck(command.raw);

  CommandFrame nopCommand;
  nopCommand.values.rw = READ;
  nopCommand.values.commandFrame = NOP_REG;
  nopCommand.values.parc = parityCheck(nopCommand.raw);

  uint16_t zposH16,zposL16; // even though ZPOS registers are only 8 bit, the fn we have is 16 bit, so just leave that as is.

  ZPOSH_frame ZPOS_H;
  ZPOSL_frame ZPOS_L;

  zposH16 = readData(command.raw, nopCommand.raw);
  ZPOS_H.raw = zposH16;

  command.values.rw = READ;
  command.values.commandFrame = ZPOSL_REG;
  command.values.parc = parityCheck(command.raw);


  zposL16 = readData(command.raw, nopCommand.raw);
  ZPOS_L.raw = zposL16;

  uint16_t zpos = (ZPOS_H.raw << 6) + (ZPOS_L.raw & 0x3F);
  return zpos;
}


void writeRegister(uint16_t registerAddress, uint16_t registerValue) {
  CommandFrame command;
  command.values.rw = WRITE;
  command.values.commandFrame = registerAddress;
  command.values.parc = parityCheck(command.raw);

  WriteDataFrame contentFrame;
  contentFrame.values.data = registerValue;
  contentFrame.values.low = 0; //always low
  contentFrame.values.pard = parityCheck(contentFrame.raw);
  writeData(command.raw, contentFrame.raw);

  //Add a nop write here and check what value you get back. it should be what you wrote.
  //can return one if successful, else false.
}


// reading data from the register Eg: readFromRegister(ERRFL_REG);
void readFromRegister(uint16_t address){
  readRegister(address);
}


uint16_t angleReading(){
  Angle angle;
  ReadDataFrame readdataframe = readRegister(ANGLE_REG);
  angle.raw = readdataframe.values.data;
  return angle.values.cordicang;
}


/*void writeZeroReg(uint16_t zeroValue){
  Zposl zposl;
  zposl.values.zposl = zeroValue & 0x003F;

  Zposm zposm;
  zposm.values.zposm = (zeroValue >> 6) & 0x00ff;

  writeRegister(ZPOSM_REG, zposm.raw);
  writeRegister(ZPOSL_REG, zposl.raw);

}
*/

uint16_t angleArrayEnc[10] = {0}; // rdngs has to be btw 2 and 10
uint16_t GetAveragedAngleReading(uint16_t rdngs){
	long longAngleData=0;
	uint16_t angleData=0;
	uint8_t count = 0;
	for (int i=0;i<rdngs;i++){
		uint16_t angle_ = angleReading();
		if (angle_ != 0){
			count++;
			angleArrayEnc[i] = angle_;
			longAngleData += (int16_t) angle_;
		}
		HAL_Delay(1);
	}
	angleData = longAngleData/count;
	return angleData;
 }

