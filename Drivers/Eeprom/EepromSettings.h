/*
 * EepromSettings.h
 *
 *  Created on: 06-Mar-2023
 *      Author: harsha
 */

#ifndef INC_EEPROMSETTINGS_H_
#define INC_EEPROMSETTINGS_H_

//Addresses
//Dont let a address go across address 32 and its multiples. Thats one page.
#define Kp_ADDRESS 0X02	// float
#define Ki_ADDRESS 0X06 // float
#define START_OFFSET_ADDRESS 0X0A //int
#define FEED_FORWARD_FACTOR_ADDRESS 0X0C //float
#define MOTID_ADDRESS 0X10 //int
#define OFFSET_INDEX_ADDRESS 0X12 //int
#define DEFAULT_DIRECTION_ADDRESS 0X14 //int

#endif /* INC_EEPROMSETTINGS_H_ */
