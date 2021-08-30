/*
 * time.c
 *
 *  Created on: 31 окт. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */

#include "stm32f4xx.h"

#include "main.h"
#include "my_time.h"

#define BIN2BCD(__VALUE__) (uint8_t)((((__VALUE__) / 10U) << 4U) | ((__VALUE__) % 10U))
#define BCD2BIN(__VALUE__) (uint8_t)(((uint8_t)((__VALUE__) & (uint8_t)0xF0U) >> (uint8_t)0x4U) * 10U + ((__VALUE__) & (uint8_t)0x0FU))

volatile tRtc rtc;
volatile tUxTime uxTime;
volatile uint8_t sendToutFlag = SET;
volatile uint8_t minTout;
volatile uint8_t minToutRx;
volatile uint8_t uxSecTout;

//// Для тестирования - массив интервалов таймера WUT
//uint8_t wutCount;
//struct tWutTest {
//  eState wutState;
//  uint32_t wutVol;
//} wutTest[20];

static void RTC_SetTime( volatile tRtc * prtc );
//static void RTC_GetTime( volatile tRtc * prtc );
static void RTC_SetDate( volatile tRtc * prtc );
static void RTC_GetDate( volatile tRtc * prtc );
static void RTC_SetAlrm( tRtc * prtc );
static void RTC_GetAlrm( tRtc * prtc );
static void RTC_CorrAlrm( tRtc * prtc );

// *********** Инициализация структуры ВРЕМЯ (сейчас - системное ) ************
void rtcInit(void){
#if 1
  // **************** RTC Clock configuration ***********************
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  PWR->CR |= PWR_CR_DBP;
  RCC->BDCR |= RCC_BDCR_BDRST;
  RCC->BDCR &= ~RCC_BDCR_BDRST;

  RCC->BDCR |= RCC_BDCR_LSEON;

  while( ( PWR->CR & PWR_CR_DBP) == 0 )
  {}
  while((RCC->BDCR & RCC_BDCR_LSERDY)!=RCC_BDCR_LSERDY)
  {}

  RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_RTCSEL) | RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_0;
  RCC->APB1ENR &=~ RCC_APB1ENR_PWREN;

  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->ISR |= RTC_ISR_INIT;
  while((RTC->ISR & RTC_ISR_INITF)!=RTC_ISR_INITF)
  {}
  RTC->PRER = 0x007F00FF;
  RTC->ISR &= ~RTC_ISR_INIT;

  RTC->CR &=~ RTC_CR_ALRAE;
  while((RTC->ISR & RTC_ISR_ALRAWF) != RTC_ISR_ALRAWF)
  {}
  // Устанавливаем секунды в будильник - разбиваем все ноды на 60 групп
//  RTC->ALRMAR = (uint32_t)(BIN2BCD(rfm.nodeAddr % 60));
  RTC->ALRMAR = 0L;
  RTC->ALRMAR |= RTC_ALRMAR_MSK4 | RTC_ALRMAR_MSK3 | RTC_ALRMAR_MSK2 | RTC_ALRMAR_MSK1;

  RTC->CR = RTC_CR_ALRAIE | RTC_CR_ALRAE;

  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;

  EXTI->IMR |= EXTI_IMR_MR17;
  EXTI->RTSR |= EXTI_RTSR_TR17;
  NVIC_SetPriority(RTC_Alarm_IRQn, 0);
  NVIC_EnableIRQ(RTC_Alarm_IRQn);
  NVIC_SetPriority(RTC_WKUP_IRQn, 0);
  NVIC_EnableIRQ(RTC_WKUP_IRQn);

//
//
//
//  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
//  PWR->CR |= PWR_CR_DBP;
//  RCC->BDCR |= RCC_BDCR_BDRST;
//  RCC->BDCR &= ~RCC_BDCR_BDRST;
//  // Enable the LSE
//  RCC->BDCR |= RCC_BDCR_LSEON;
//  for( mTick = 0; !( PWR->CR & PWR_CR_DBP); mTick++) {
//    if ( mTick == HSE_STARTUP_TIMEOUT) {
//      while(1)
//      {}
//    }
//  }
//
//  while( (RCC->BDCR & RCC_BDCR_LSERDY) != RCC_BDCR_LSERDY )
//  {}
//  /* Restore the Content of BDCR register */
//  RCC->BDCR = (RCC->BDCR & ~(RCC_BDCR_RTCSEL)) | RCC_BDCR_RTCSEL_LSE;
//
//  RCC->BDCR |= RCC_BDCR_RTCEN;
//
//  // Write access for RTC registers
//  RTC->WPR = 0xCA;
//  RTC->WPR = 0x53;
//
//  // --- Configure Clock Prescaler -----
//  // Enable init phase
//  RTC->ISR |= RTC_ISR_INIT;
//  while((RTC->ISR & RTC_ISR_INITF)!=RTC_ISR_INITF)
//  {}
//  // RTCCLOCK deviser
//  RTC->PRER = 0x007F00FF;
//  RTC->ISR &= ~RTC_ISR_INIT;
//
//  // --- Configure Alarm A -----
//  // Disable alarm A to modify it
//  RTC->CR &= ~RTC_CR_ALRAE;
//  while((RTC->ISR & RTC_ISR_ALRAWF) != RTC_ISR_ALRAWF)
//  {}
//  // Устанавливаем секунды в будильник - разбиваем все ноды на 60 групп
//  RTC->ALRMAR = (uint32_t)(BIN2BCD(rfm.nodeAddr % 60));
//  // Alarm A every day, every hour, every minute, every second
//  RTC->ALRMAR |= RTC_ALRMAR_MSK4 | RTC_ALRMAR_MSK3 | RTC_ALRMAR_MSK2 | RTC_ALRMAR_MSK1;
//  RTC->CR |= RTC_CR_ALRAIE | RTC_CR_ALRAE;
//
//  // Disable write access
//  RTC->WPR = 0xFE;
//  RTC->WPR = 0x64;
//
//  // Configure exti and nvic for RTC ALARM IT
//  EXTI->IMR |= EXTI_IMR_MR17;
//  // Rising edge for line 17
//  EXTI->RTSR |= EXTI_RTSR_TR17;
//
//  NVIC_SetPriority(RTC_IRQn, 1);
//  NVIC_EnableIRQ(RTC_IRQn);

#else
  uint32_t tempReg;
  uint32_t prediv;
  uint32_t source;

// **************** RTC Clock configuration ***********************
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  PWR->CR |= PWR_CR_DBP;

  /* Wait for Backup domain Write protection disable */

  RCC->BDCR |= RCC_BDCR_LSEON;
  for( mTick = 0; !( PWR->CR & PWR_CR_DBP); mTick++) {
    if ( mTick == HSE_STARTUP_TIMEOUT) {
      while(1)
      {}
    }
  }

  if( (RCC->BDCR & RCC_BDCR_LSERDY) == RCC_BDCR_LSERDY ){
    source = RCC_BDCR_RTCSEL_LSE;
    prediv = (uint32_t)(0x7F);
  }
  else {
    RCC->CSR |= ((uint32_t)RCC_CSR_LSION);
    for( mTick = 0; !(RCC->CSR & RCC_CSR_LSIRDY); mTick++) {
      if ( mTick == HSE_STARTUP_TIMEOUT) {
        while(1)
        {}
      }
    }
    // Для LSI = 40MHz
    source = RCC_BDCR_RTCSEL_LSI;
    prediv = (uint32_t)(0x9C);
  }

  /* Store the content of BDCR register before the reset of Backup Domain */
  tempReg = (RCC->BDCR & ~(RCC_BDCR_RTCSEL));
  /* RTC Clock selection can be changed only if the Backup Domain is reset */
  RCC->BDCR |= RCC_BDCR_BDRST;
  RCC->BDCR &= ~RCC_BDCR_BDRST;
  /* Restore the Content of BDCR register */
  RCC->BDCR = tempReg | source;

  RCC->BDCR |= RCC_BDCR_RTCEN;

// ******************** RTC System configuration *************************

  /* Disable the write protection for RTC registers */
  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;

  /* Set Initialization mode */
  RTC->ISR |= RTC_ISR_INIT;
  while( (RTC->ISR & RTC_ISR_INITF) == 0 )
  {}
  // Set 24-Hour format
  RTC->CR &= ((uint32_t)~(RTC_CR_FMT));

  RTC->PRER = ((uint32_t)prediv << 16) | 0xFF;

  // Устанавливаем секунды в будильник - разбиваем все ноды на 60 групп
  RTC->ALRMAR = (uint32_t)(BIN2BCD(rfm.nodeAddr % 60));
  // Alarm A every day, every hour, every minute, every second
  RTC->ALRMAR |= RTC_ALRMAR_MSK4 | RTC_ALRMAR_MSK3 | RTC_ALRMAR_MSK2 | RTC_ALRMAR_MSK1;
  RTC->ISR &= ~RTC_ISR_ALRAF;
  RTC->CR |= RTC_CR_ALRAIE | RTC_CR_ALRAE;

  // Exit Initialization mode
  RTC->ISR &= RTC_ISR_INIT;
  while( (RTC->ISR & RTC_ISR_INITF) == 0 )
  {}

  /* Enable the write protection for RTC registers */
  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;

  // Configure exti and nvic for RTC ALARM IT
  EXTI->IMR |= EXTI_IMR_MR17;
  // Rising edge for line 17
  EXTI->RTSR |= EXTI_RTSR_TR17;

  NVIC_SetPriority(RTC_IRQn, 1);
  NVIC_EnableIRQ(RTC_IRQn);
#endif
}

void timeInit( void ) {
  //Инициализируем RTC
  rtcInit();

  /*##-1- Configure the Date #################################################*/
  /* Set Date: Saturday February 15th 2018 */
  rtc.year = 21;
  rtc.month = 9;
  rtc.date = 1;
  rtc.wday = 4;
  rtc.hour = 12;
  rtc.min = 0;
  rtc.sec = 0;
  rtc.ss = 0;

  while( (RTC->ISR & RTC_ISR_RSF) == 0)
  {}
  RTC_SetDate( &rtc );
  RTC_SetTime( &rtc );
  // Интервал будильника - измерение минуты
  minTout = 1;
  // Интервал будильника - передача минуты
  minToutRx = 1;
  // Интервал будильника - секунды
  uxSecTout = 10;
  while( RTC->DR == 0x2101 )
  {}
  getRtcTime();
}

// Получение системного мремени
uint32_t getTick( void ) {
  // Возвращает количество тиков
  return mTick;
}

#define _TBIAS_DAYS   ((70 * (uint32_t)365) + 17)
#define _TBIAS_SECS   (_TBIAS_DAYS * (uint32_t)86400)
#define _TBIAS_YEAR   0
#define MONTAB(year)    ((((year) & 03) || ((year) == 0)) ? mos : lmos)

const int16_t lmos[] = {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
const int16_t mos[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

#define Daysto32(year, mon) (((year - 1) / 4) + MONTAB(year)[mon])

/////////////////////////////////////////////////////////////////////

tUxTime xTm2Utime( volatile tRtc * prtc ){
  /* convert time structure to scalar time */
int32_t   days;
int32_t   secs;
int32_t   mon, year;

  /* Calculate number of days. */
  mon = prtc->month - 1;
  // Годы считаем от 1900г.
  year = (prtc->year + 100) - _TBIAS_YEAR;
  days  = Daysto32(year, mon) - 1;
  days += 365 * year;
  days += prtc->date;
  days -= _TBIAS_DAYS;

  /* Calculate number of seconds. */
  secs  = 3600 * prtc->hour;
  secs += 60 * prtc->min;
  secs += prtc->sec;

  secs += (days * (tUxTime)86400);

  return (secs);
}

/////////////////////////////////////////////////////////////////////

void xUtime2Tm( volatile tRtc * prtc, tUxTime secsarg){
  uint32_t    secs;
  int32_t   days;
  int32_t   mon;
  int32_t   year;
  int32_t   i;
  const int16_t * pm;

  #ifdef  _XT_SIGNED
  if (secsarg >= 0) {
      secs = (uint32_t)secsarg;
      days = _TBIAS_DAYS;
    } else {
      secs = (uint32_t)secsarg + _TBIAS_SECS;
      days = 0;
    }
  #else
    secs = secsarg;
    days = _TBIAS_DAYS;
  #endif

    /* days, hour, min, sec */
  days += secs / 86400;
  secs = secs % 86400;
  prtc->hour = secs / 3600;
  secs %= 3600;
  prtc->min = secs / 60;
  prtc->sec = secs % 60;

  prtc->wday = (days + 1) % 7;

  /* determine year */
  for (year = days / 365; days < (i = Daysto32(year, 0) + 365*year); ) { --year; }
  days -= i;
  // Годы выставляем от эпохи 2000г., а не 1900г., как в UNIX Time
  prtc->year = (year - 100) + _TBIAS_YEAR;

    /* determine month */
  pm = MONTAB(year);
  for (mon = 12; days < pm[--mon]; );
  prtc->month = mon + 1;
  prtc->date = days - pm[mon] + 1;
}

void setRtcTime( tUxTime xtime ){

  xUtime2Tm( &rtc, xtime);
  RTC_SetTime( &rtc );
  RTC_SetDate( &rtc );
}

tUxTime getRtcTime( void ){

  RTC_GetTime( &rtc );
  RTC_GetDate( &rtc );
  return xTm2Utime( &rtc );
}

uint8_t getRtcMin( void ){
  while((RTC->ISR & RTC_ISR_RSF) == 0)
  {}
  return BCD2BIN( (RTC->TR >> 8) & 0x7F );
}

/* Установка будильника
 *  xtime - UNIX-времени
 *  alrm - номере будильника
 */
void setAlrm( tUxTime xtime){
  tRtc tmpRtc;

  xUtime2Tm( &tmpRtc, xtime);
  RTC_SetAlrm( &tmpRtc );
}

/* Получениевремени будильника
 *  alrm - номере будильника
 *  Возвращает - UNIX-время
 */
tUxTime getAlrm( void ){
  tRtc tmpRtc;

  RTC_GetAlrm( &tmpRtc );
  return xTm2Utime( &tmpRtc );
}

/* Коррекция будильника в соответствии с реалиями занятости канала:
 * В следующий раз будем пробовать отправлять данные именно в это значение секунд,
 * раз именно сейчас канал свободен.
 */
void correctAlrm( void ){
  tRtc tmpRtc;

  // Получим текущее время, заодно обновим глобальное значение
  uxTime = getRtcTime();
  xUtime2Tm( &tmpRtc, uxTime);
  RTC_CorrAlrm( &tmpRtc );
}

void setAlrmSecMask( uint8_t secMask ){
  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->CR &=~ RTC_CR_ALRAE;
  while ((RTC->ISR & RTC_ISR_ALRAWF) != RTC_ISR_ALRAWF)
  {}
  // Alarm A every day, every hour, every minute, every second
  if( secMask ){
    RTC->ALRMAR |= RTC_ALRMAR_MSK1;
  }
  else {
    RTC->ALRMAR &= ~RTC_ALRMAR_MSK1;
  }
  RTC->CR = RTC_CR_ALRAIE | RTC_CR_ALRAE;
  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;
}

void timersHandler( void ) {

#if 0
  // Таймаут timerCount1
  if ( timerCount1 > 1) {
    timerCount1--;
  }
  // Таймаут timerCount2
  if ( timerCount2 > 1) {
    timerCount2--;
  }
  // Таймаут timerCount3
  if ( timerCount3 > 1) {
    timerCount3--;
  }
  if ( !(mTick % 1000) ){
    secondFlag = SET;
  }
#endif
}

#if 0
void timersProcess( void ) {

#if 0
  // Таймаут timerCount1
  if ( timerCount1 == 0) {
    timerCount1 = TOUTCOUNT1;
  }
  // Таймаут timerCount2
  if ( timerCount2 == 0) {
    timerCount2 = TOUTCOUNT2;
  }
  // Таймаут timerCount3
  if ( timerCount3 == 3) {
    timerCount3 = TOUTCOUNT3;
  }
#endif
  if (secondFlag) {
    secondFlag = RESET;
  }
}
#endif

#if 1
// Задержка по SysTick без прерывания
void mDelay( uint32_t t_ms ){
	SysTick->VAL = 0;
	while ( t_ms > 0 ){
		while ( !( SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk ) ) // wait for underflow
		{}
		t_ms--;
	}
}
#else
// Задержка в мс
void mDelay( uint32_t del ){
  uint32_t finish = mTick + del;
  while ( mTick < finish)
  {}
}
#endif

static void RTC_SetTime( volatile tRtc * prtc ){
  register uint32_t temp = 0U;

  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->ISR |= RTC_ISR_INIT;
  while((RTC->ISR & RTC_ISR_INITF)!=RTC_ISR_INITF)
  {}

  temp = (BIN2BCD( prtc->hour ) << 16 |
          BIN2BCD( prtc->min ) << 8 |
          BIN2BCD( prtc->sec ) );
  // Time reserved mask
  temp &= (uint32_t)0x007F7F7F;
  RTC->TR = ( RTC->TR & (RTC_TR_PM | RTC_TR_HT | RTC_TR_HU | RTC_TR_MNT | RTC_TR_MNU | RTC_TR_ST | RTC_TR_SU)) | temp;
  // Сбрасываем флаг синхронизации часов
  RTC->ISR &= ~RTC_ISR_RSF;
  RTC->ISR &= ~RTC_ISR_INIT;
  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;
}


static void RTC_SetDate( volatile tRtc * prtc ){
  register uint32_t temp = 0U;

  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->ISR |= RTC_ISR_INIT;
  while((RTC->ISR & RTC_ISR_INITF)!=RTC_ISR_INITF)
  {}

  temp = (BIN2BCD( prtc->year ) << 16 |
          BIN2BCD( prtc->month ) << 8 |
          BIN2BCD( prtc->date ) |
          BIN2BCD( prtc->wday ) << 13 );
  // Date reserved mask
  temp &= (uint32_t)0x00FFFF3F;
  RTC->DR = ( RTC->DR & ~(RTC_DR_YT | RTC_DR_YU | RTC_DR_MT | RTC_DR_MU | RTC_DR_DT | RTC_DR_DU | RTC_DR_WDU)) | temp;
  // Сбрасываем флаг синхронизации часов
  RTC->ISR &= ~RTC_ISR_RSF;
  RTC->ISR &= ~RTC_ISR_INIT;
  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;
}

void RTC_GetTime( volatile tRtc * prtc ){
  while((RTC->ISR & RTC_ISR_RSF) == 0)
  {}
  uint32_t tmpTr = RTC->TR;
  prtc->hour = BCD2BIN( tmpTr >> 16 );
  prtc->min = BCD2BIN( tmpTr >> 8 );
  prtc->sec = BCD2BIN( tmpTr  );
  prtc->ss = RTC->SSR;
}

static void RTC_GetDate( volatile tRtc * prtc ){
  while((RTC->ISR & RTC_ISR_RSF) == 0)
  {}
  uint32_t tmpDr = RTC->DR;
  prtc->year = BCD2BIN( tmpDr >> 16 );
  prtc->month = BCD2BIN( (tmpDr >> 8) & 0x1f );
  prtc->date = BCD2BIN( tmpDr );
  prtc->wday = ( tmpDr >> 13 ) & 0x7;
}

static void RTC_SetAlrm( tRtc * prtc ){
  register uint32_t temp = 0U;

  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->ISR |= RTC_ISR_INIT;
  while((RTC->ISR & RTC_ISR_INITF)!=RTC_ISR_INITF)
  {}

  temp = (BIN2BCD( prtc->date ) << 24 |
          BIN2BCD( prtc->hour ) << 16 |
          BIN2BCD( prtc->min ) << 8 |
          BIN2BCD( prtc->sec ) );
  RTC->ALRMAR = ( RTC->ALRMAR & (RTC_ALRMAR_PM | RTC_ALRMAR_DT | RTC_ALRMAR_DU | RTC_ALRMAR_HT | RTC_ALRMAR_HU | RTC_ALRMAR_MNT | RTC_ALRMAR_MNU | RTC_ALRMAR_ST | RTC_ALRMAR_SU)) | temp;

  RTC->ISR &= ~RTC_ISR_INIT;
  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;
}

static void RTC_GetAlrm( tRtc * prtc ){
  prtc->date = BCD2BIN( RTC->ALRMAR >> 24 );
  prtc->hour = BCD2BIN( RTC->ALRMAR >> 16 );
  prtc->min = BCD2BIN( RTC->ALRMAR >> 8);
  prtc->sec = BCD2BIN( RTC->ALRMAR );
}

static void RTC_CorrAlrm( tRtc * prtc ){
  register uint32_t temp = 0U;

  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->ISR |= RTC_ISR_INIT;
  while((RTC->ISR & RTC_ISR_INITF)!=RTC_ISR_INITF)
  {}

  temp = BIN2BCD( prtc->sec );
  RTC->ALRMAR = ( RTC->ALRMAR & (RTC_ALRMAR_ST | RTC_ALRMAR_SU)) | temp;

  RTC->ISR &= ~RTC_ISR_INIT;
  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;
}

void usTimStop( void ){
  // Остонавливаем
  TIM2->CR1 &= ~TIM_CR1_CEN;
  // Стираем все флаги
  TIM2->SR = 0;
}

/* Установка и запуск wakeup-таймера
 * us - время в мкс.
 */
void usTimSet( uint32_t us ){

  TIM2->CNT = us;
  //Запускаем
  TIM2->CR1 |= TIM_CR1_CEN;
}

void usTimInit( void ){
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  // Не включаем
  TIM2->CR1 &= ~TIM_CR1_CEN;
  // Прескалер - для 1МГц (1мкс)
  TIM2->PSC = RCC_Clocks.PCLK1_Frequency/1000000 - 1;
  // Обратный отсчет
  TIM2->CR1 |= TIM_CR1_DIR;
  // Прерывание по обнулению
  TIM2->DIER |= TIM_DIER_UIE;

  NVIC_SetPriority(TIM2_IRQn, 1);
  NVIC_EnableIRQ( TIM2_IRQn);
}

// Таймер мигания светодиода при ошибке реле
void errTimInit( void ){
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
  // Не включаем
  TIM6->CR1 &= ~TIM_CR1_CEN;
  // Прескалер - для 1кГц (1мс)
  TIM6->PSC = RCC_Clocks.PCLK1_Frequency/1000 - 1;
  // Срабатывать будет каждые 250мс (два мигания за секунду)
  TIM6->ARR = 249;
  // Прерывание по обнулению
  TIM2->DIER |= TIM_DIER_UDE;

  NVIC_SetPriority(TIM6_DAC_IRQn, 3);
  NVIC_EnableIRQ( TIM6_DAC_IRQn);
}
