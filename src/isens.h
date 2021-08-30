/*
 * isens.h
 *
 *  Created on: 30 авг. 2021 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#ifndef ISENS_H_
#define ISENS_H_

#include "main.h"

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
  sGpioPin pinOut;
  sGpioPin pinIn;
  uint32_t isensCount;
  eISensState state;
} sISens;

extern sISens iSens;

void isensInit( sISens * pin );

#endif /* ISENS_H_ */
