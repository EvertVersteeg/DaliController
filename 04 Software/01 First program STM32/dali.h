/*
 * dali.h
 *
 *  Created on: Aug 27, 2021
 *      Author: evers
 */

#ifndef SRC_DALI_H_
#define SRC_DALI_H_

#include "main.h"
#include <stdbool.h>


uint16_t delay1=417;
uint16_t delay2=417;
uint16_t period=417+417;
uint16_t value_adc;
uint16_t maxLevel;
uint16_t minLevel;
uint16_t analogLevel;

uint32_t daliTimeout = 20000; //us, DALI response timeout
uint16_t analogLevel = 870; //analog border level (less - "0"; more - "1")
bool msgMode; //0 - get only response from dali bus to COM; 1 - response with text (comments)
bool getResponse;


#endif /* SRC_DALI_H_ */
