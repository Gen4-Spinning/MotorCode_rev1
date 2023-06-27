/*
 * temperature LUT.h
 *
 *  Created on: Mar 5, 2023
 *      Author: Jonathan
 */

#ifndef TEMPERATURE_TEMPERATURELUT_H_
#define TEMPERATURE_TEMPERATURELUT_H_

uint8_t get_MOSFET_temperature(uint16_t MOSFETadc);
uint8_t get_MOTOR_temperature(uint16_t MOTORadc);

#endif /* TEMPERATURE_TEMPERATURELUT_H_ */
