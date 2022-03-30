/*
 * calend.h
 *
 *  Created on: 6 мар. 2022 г.
 *      Author: jet Tanchin Gennady <g.tanchin@yandex.ru>
 */

#ifndef CALEND_H_
#define CALEND_H_

#include "stdint.h"
#include "list.h"

typedef enum {
  SMALLER,
  EQUAL,
  GREATER
} eCmp;

typedef enum field {
  F_BEGIN,
  F_END,
  F_STEP,
  F_NUM
} eField;

typedef enum timesect{
  TS_MIN,
  TS_HOUR,
  TS_DAY,
  TS_NUM
} eTimeSect;


typedef struct {
  struct list_head node;
  uint8_t beg;     // Начало интервала
  uint8_t end;     // Конец интервала
  uint8_t step;     // Шаг приращения внутри интервала
} sCal;

typedef struct {
  struct list_head mQ;
  struct list_head hQ;
  struct list_head dQ;
} sCalend;

#endif /* CALEND_H_ */
