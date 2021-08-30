/*
 * my_time.h
 *
 *  Created on: 31 окт. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */

#ifndef MY_TIME_H_
#define MY_TIME_H_

#include <stdint.h>
//#include "stm32f0xx_rtc.h"
//#include "stm32f0xx.h"

#define ALARM_UPDATE_TOUT   100

#define ALRM_A      0   // НЕ МЕНЯТЬ !!!!
#define ALRM_B      1   // НЕ МЕНЯТЬ !!!!

#define RTCCLOCK     32768

typedef struct {
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t ss;     // Доля секунды = (PREDIV_S - SS) / (PREDIV_S + 1)
} tTime;

typedef struct {
  uint8_t year;
  uint8_t month;
  uint8_t date;
  uint8_t wday;
} tDate;

typedef struct {
  uint8_t year;
  uint8_t month;
  uint8_t date;
  uint8_t wday;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t ss;     // Доля секунды = (PREDIV_S - SS) / (PREDIV_S + 1)
} tRtc;

#define TIMEZONE_MSK      (+3)

  // DEF: standard signed format
  // UNDEF: non-standard unsigned format
  #define _XT_SIGNED

#ifdef  _XT_SIGNED
  typedef int32_t                           tUxTime;
#else
  typedef uint32                          tUxTime;
#endif

typedef struct
{
  uint32_t SYSCLK_Frequency;
  uint32_t HCLK_Frequency;
  uint32_t PCLK1_Frequency;
  uint32_t PCLK2_Frequency;
}RCC_ClocksTypeDef;

extern volatile tRtc rtc;
extern volatile tUxTime uxTime;
extern volatile uint8_t sendToutFlag;
extern volatile uint8_t minTout;
extern volatile uint8_t minToutRx;
extern volatile uint8_t uxSecTout;


extern volatile uint32_t mTick;

// *********** Инициализация структуры ВРЕМЯ (сейчас - системное ) ************
void rtcInit(void);
void timeInit( void );
// Получение системного мремени
uint32_t getTick( void );
tUxTime xTm2Utime( volatile tRtc * prtc );

void timersProcess( void );

void timersHandler( void );
void mDelay( uint32_t del );

void setRtcTime( tUxTime xtime );
tUxTime getRtcTime( void );

void RTC_GetTime( volatile tRtc * prtc );

void setAlrm( tUxTime xtime );
tUxTime getAlrm( void );
void correctAlrm( void );
void setAlrmSecMask( uint8_t secMask );

void usTimInit( void );
void usTimStop( void );
/* Установка и запуск таймера микосекунд
 * us - время в мкс.
 */
void usTimSet( uint32_t us );
// Таймер мигания светодиодом
void errTimInit( void );

#endif /* UNIX_TIME_H_ */
