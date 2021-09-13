/*
 * isens.h
 *
 *  Created on: 30 авг. 2021 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#ifndef ISENS_H_
#define ISENS_H_

#include "main.h"

#define DEBOUNCE_TOUT    30

typedef enum  {
  ISENS_1,
  ISENS_2,
  ISENS_3,
  ISENS_4,
  ISENS_5,
  ISENS_6,
  ISENS_SB1,
  ISENS_SB2,
  ISENS_NUM
} eIsens;

typedef enum {
  ISENS_DOWN,
  ISENS_UP,
  ISENS_BREAKAGE,
  ISENS_SHORT
} eISensState;

typedef struct {
  GPIO_TypeDef * gpio;
  uint16_t pin;
  uint8_t portNum;
  uint8_t pinNum;
} sGpioPin;

typedef struct {
//  sGpioPin pinOut;
  sGpioPin pinIn;
  uint32_t isensCount;
  eISensState state;
  uint32_t debounceTout;
  uint8_t isensFlag;
} sISens;

extern sISens iSens[ISENS_NUM];

void isensInit( void );
void isensProcess( void );

#endif /* ISENS_H_ */