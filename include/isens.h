/*
 * isens.h
 *
 *  Created on: 30 авг. 2021 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#ifndef ISENS_H_
#define ISENS_H_

#include "main.h"

#define ISENS_DB_TOUT    10

#define ISENS_TIM                 TIM2
#define ISENS_TIM_CLK_EN          RCC->APB1ENR |= RCC_APB1ENR_TIM2EN
#define ISENS_TIM_IRQn            TIM2_IRQn
#define ISENS_TIM_IRQ_PRIORITY    (3)
#define ISENS_IRQHandler          TIM2_IRQHandler
#define ISENS_TIM_FREQ            200000


typedef enum  {
  ISENS_1,
  ISENS_2,
  ISENS_3,
  ISENS_4,
//  ISENS_5,
//  ISENS_6,
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
} sSimplePin;

typedef struct {
//  sGpioPin pinOut;
  sSimplePin pinIn;
  uint32_t isensCount;
  eISensState state;
  uint32_t debounceTout;
  uint8_t isensFlag;
  uint32_t tstime;            // Метка времени срабатывания датчика - сек
  uint8_t tsss;               // Метка времени срабатывания датчика - SS
  struct timer_list dbTimer;   // Таймер антидребезга
} sISens;

// ------------------ Archive Calendar --------------------------------------------
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

// -------------------------------------------------------------------------------

extern char * defCal;
extern sISens iSens[ISENS_NUM];

void isensInit( void );
void isensEnable( void );
void isensProcess( void );
void sensPubAlrmSet( sCalend * cal );

#endif /* ISENS_H_ */
