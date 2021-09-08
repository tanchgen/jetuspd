/*
 * isens.h
 *
 *  Created on: 30 авг. 2021 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#ifndef ISENS_H_
#define ISENS_H_

#include "main.h"

enum  {
  ISENS_1,
  ISENS_2,
  ISENS_3,
  ISENS_4,
  ISENS_5,
  ISENS_6,
  ISENS_SB1,
  ISENS_SB2,
  ISENS_NUM
};

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
} sISens;

extern sISens iSens[ISENS_NUM];

void isensInit( sISens * pin );

#endif /* ISENS_H_ */
