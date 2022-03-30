#include <stddef.h>

#include "main.h"
#include "stm32l1xx_hal.h"
#include "stm32l1xx_ll_iwdg.h"

#include "times.h"
#include "uart.h"

#define PREDIV_S      0x128

extern const sUartHnd simHnd;

volatile uint32_t  mTick = 0;

// Флаг запуска активных таймеров
volatile FlagStatus actTimRun = RESET;
// Флаг запуска составления списка активных таймеров
volatile FlagStatus actTimListRun = RESET;

const uint8_t SynchPrediv = 0xFF;
const uint8_t AsynchPrediv = 0x7F;

/** Голова очереди ms таймеров на исполнение. */
static struct list_head  msTimersQueue = LIST_HEAD_INIT(msTimersQueue);

/// Голова очереди секундных таймеров.
struct list_head  rtcTimQueue = LIST_HEAD_INIT(rtcTimQueue);
/// Голова очереди секундных таймеров на исполнение.
struct list_head  actRtcTimQueue = LIST_HEAD_INIT(actRtcTimQueue);

//#define RTC_RSF_MASK            ((uint32_t)0xFFFFFF5F)
/* ------------ RCC registers bit address in the alias region ----------- */
#define RCC_OFFSET                (RCC_BASE - PERIPH_BASE)
/* --- BDCR Register ---*/
/* Alias word address of RTCEN bit */
#define BDCR_OFFSET               (RCC_OFFSET + 0x70)
//#define RTCEN_BitNumber           0x0F
//#define BDCR_RTCEN_BB             (PERIPH_BB_BASE + (BDCR_OFFSET * 32) + (RTCEN_BitNumber * 4))
/* Alias word address of BDRST bit */
//#define BDRST_BitNumber           0x10
//#define BDCR_BDRST_BB             (PERIPH_BB_BASE + (BDCR_OFFSET * 32) + (BDRST_BitNumber * 4))
/* Alias word address of LSEON bit */
//#define LSEON_BitNumber           0x0
#define BDCR_LSEON_BB             (PERIPH_BB_BASE + (BDCR_OFFSET * 32) + (LSEON_BitNumber * 4))

#define BIN2BCD(__VALUE__) (uint8_t)((((__VALUE__) / 10U) << 4U) | ((__VALUE__) % 10U))
#define BCD2BIN(__VALUE__) (uint8_t)(((uint8_t)((__VALUE__) & (uint8_t)0xF0U) >> (uint8_t)0x4U) * 10U + ((__VALUE__) & (uint8_t)0x0FU))

// for LSI
#define WUT_K     ((uint32_t)(1e6 * 4 / LSI_VALUE))

volatile tRtc rtc;
volatile tUxTime uxTime;
volatile uint8_t sendToutFlag = SET;
volatile uint8_t minTout;
volatile uint8_t minToutRx;
volatile uint8_t uxSecTout;

static void rtcSetTime( volatile tRtc * prtc );
//static void rtcGetTime( volatile tRtc * prtc );
//static void RTC_GetTime( volatile tRtc * prtc );
static void rtcSetDate( volatile tRtc * prtc );
//static void rtcGetDate( volatile tRtc * prtc );
void rtcSetAlrm( tRtc * prtc );
void rtcSetAlrmMask( tRtc * prtc, uint32_t mask );
void rtcGetAlrm( tRtc * prtc );
void rtcCorrAlrm( tRtc * prtc );

//void uartRxClock( void );
void ledProcess( uint32_t tick );

// *********** Инициализация структуры ВРЕМЯ (сейчас - системное ) ************
void rtcInit(void){
  uint32_t psc;

  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  PWR->CR |= PWR_CR_DBP;
  RCC->CSR |= RCC_CSR_RTCRST;
  RCC->CSR &= ~RCC_CSR_RTCRST;

  // Enable the LSE
  RCC->CSR |= RCC_CSR_LSEON;
  // Wait while it is not ready
  for(uint32_t tmpc = 0; ((RCC->CSR & RCC_CSR_LSERDY) != RCC_CSR_LSERDY) && (tmpc < 10000); tmpc++ )
  {}

  if( RCC->CSR & RCC_CSR_LSERDY ){
    // LSE is ON
    // LSE enable for RTC clock
    RCC->CSR = (RCC->CSR & ~RCC_CSR_RTCSEL) | RCC_CSR_RTCSEL_0 | RCC_CSR_RTCEN;
    psc = 0x007F00FF;
  }
  else {
    // LSE is OFF
    RCC->CSR &= ~RCC_CSR_LSEON;
    // LSE enable for RTC clock
    RCC->CSR |= RCC_CSR_LSION;
    for(uint32_t tmpc = 0; ((RCC->CSR & RCC_CSR_LSIRDY) != RCC_CSR_LSIRDY) && (tmpc < 10000); tmpc++ )
    {}
    if( (RCC->CSR & RCC_CSR_LSIRDY) == RESET ){
      trace_puts("LSI Fail");
      ErrHandler( STOP );
    }

    RCC->CSR = (RCC->CSR & ~RCC_CSR_RTCSEL) | RCC_CSR_RTCSEL_1 | RCC_CSR_RTCEN;
    // ~38kHz / 0x80 / 0x129 = ~1Hz (1s)
    psc = 0x007F0000 | (PREDIV_S - 1);
  }

  // Write access for RTC registers
  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;

  // --- Configure Clock Prescaler -----
  // Enable init phase
  RTC->ISR |= RTC_ISR_INIT;
  while((RTC->ISR & RTC_ISR_INITF)!=RTC_ISR_INITF)
  {}
  // RTCCLOCK deviser
  RTC->PRER = psc;
  assert_param( (RTC->PRER & 0xFFFF) == (PREDIV_S - 1) );

  RTC->ISR &= ~RTC_ISR_INIT;

  RTC->CR |= RTC_CR_BYPSHAD;

  // --- Configure Alarm A -----
  // Disable alarm A to modify it
  RTC->CR &= ~RTC_CR_ALRAE;
  while((RTC->ISR & RTC_ISR_ALRAWF) != RTC_ISR_ALRAWF)
  {}
  RTC->ALRMAR = 0;
  RTC->CR |= RTC_CR_ALRAIE | RTC_CR_ALRAE;

  // --- Configure Alarm B -----
  // Disable alarm B to modify it
  RTC->CR &= ~RTC_CR_ALRBE;
  while((RTC->ISR & RTC_ISR_ALRBWF) != RTC_ISR_ALRBWF)
  {}
  RTC->ALRMBR = 0;
  // Alarm B every day, every hour, every minute, every second
  RTC->ALRMBR |= RTC_ALRMBR_MSK4 | RTC_ALRMBR_MSK3 | RTC_ALRMBR_MSK2 | RTC_ALRMBR_MSK1;
  RTC->CR |= RTC_CR_ALRBIE | RTC_CR_ALRBE;

  // --- Configure WakeUp Timer -----
  RTC->CR &= ~RTC_CR_WUTE;
  while((RTC->ISR & RTC_ISR_WUTWF) != RTC_ISR_WUTWF)
  {}

  // частота = RTCCLOCK (32768кГц) / 4: T = ~122.07мкс
  RTC->CR = (RTC->CR & ~RTC_CR_WUCKSEL) | RTC_CR_WUCKSEL_1 | RTC_CR_WUTIE;

  // Disable WUT
  RTC->CR &= ~RTC_CR_WUTE;

  // Disable write access
  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;

  // Configure exti and nvic for RTC ALARM IT
  EXTI->IMR |= EXTI_IMR_MR17;
  // Rising edge for line 17
  EXTI->RTSR |= EXTI_RTSR_TR17;

  // Configure exti and nvic for RTC WakeUp-Timer IT
  EXTI->IMR |= EXTI_IMR_MR20;
  // Rising edge for line 20
  EXTI->RTSR |= EXTI_RTSR_TR20;

  NVIC_SetPriority(RTC_Alarm_IRQn, 1);
  NVIC_EnableIRQ(RTC_Alarm_IRQn);
  NVIC_SetPriority(RTC_WKUP_IRQn, 1);
  NVIC_EnableIRQ(RTC_WKUP_IRQn);
}

void timeInit( void ) {
  //Инициализируем RTC
  uint32_t vol;

  rtcInit();

  /*##-1- Configure the Date #################################################*/
  /* Set Date: Wednesday June 1st 2016 */
  rtc.year = 21;
  rtc.month = 07;
  rtc.date = 30;
  rtc.wday = 5;
  rtc.hour = 12;
  rtc.min = 0;
  rtc.sec = 50;;
  rtc.ss = 0;

  vol = RTC->DR;
  rtcSetDate( &rtc );
  rtcSetTime( &rtc );
//  // Выставляем будильник для  измерения температуры
//  rtc.sec = BIN2BCD(rfm.nodeAddr % 60) + 1;
//  uxTime = xTm2Utime( &rtc );
//  setAlrm( uxTime, ALRM_A );
//
  while( (vol == RTC->DR) )
  {}
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
  rtcSetTime( &rtc );
  rtcSetDate( &rtc );
}

tUxTime getRtcTime( void ){
  uint32_t tmpTr;
  uint32_t tmpDr;

  rtc.ss = RTC->SSR;
  tmpTr = RTC->TR;
  tmpDr = RTC->DR;
  if( (rtc.ss != RTC->SSR) || (tmpTr != RTC->TR) || (tmpDr = RTC->DR) ){
    rtc.ss = RTC->SSR;
    tmpTr = RTC->TR;
    tmpDr = RTC->DR;
  }
  rtc.hour = BCD2BIN( tmpTr >> 16 );
  rtc.min = BCD2BIN( tmpTr >> 8 );
  rtc.sec = BCD2BIN( tmpTr  );
  rtc.year = BCD2BIN( tmpDr >> 16 );
  rtc.month = BCD2BIN( (tmpDr >> 8) & 0x1f );
  rtc.date = BCD2BIN( tmpDr );
  rtc.wday = ( tmpDr >> 13 ) & 0x7;

  return (uxTime = xTm2Utime( &rtc ));
}

uint8_t getRtcMin( void ){
  uint32_t tmpTr;

  tmpTr = RTC->TR;
  if( tmpTr != RTC->TR ){
    tmpTr = RTC->TR;
  }
  return BCD2BIN( (tmpTr >> 8) & 0x7F );
}

/* Установка будильника
 *  xtime - UNIX-времени
 *  alrm - номере будильника
 */
void setAlrm( tUxTime xtime){
  tRtc tmpRtc;

  xUtime2Tm( &tmpRtc, xtime);
  rtcSetAlrm( &tmpRtc );
}

/* Установка будильника c маской
 *  xtime - UNIX-времени
 *  alrm - номере будильника
 */
void setAlrmMask( tUxTime xtime, uint32_t mask){
  tRtc tmpRtc;

  xUtime2Tm( &tmpRtc, xtime);
  rtcSetAlrmMask( &tmpRtc, mask & (RTC_ALRMAR_MSK4 | RTC_ALRMAR_MSK3 | RTC_ALRMAR_MSK2 | RTC_ALRMAR_MSK1) );
}

/* Получениевремени будильника
 *  alrm - номере будильника
 *  Возвращает - UNIX-время
 */
tUxTime getAlrm( void ){
  tRtc tmpRtc;

  rtcGetAlrm( &tmpRtc );
  return xTm2Utime( &tmpRtc );
}

/* Коррекция будильника в соответствии с реалиями занятости канала:
 * В следующий раз будем пробовать отправлять данные именно в это значение секунд,
 * раз именно сейчас канал свободен.
 */
void correctAlrm( void ){
  tRtc tmpRtc;

  // Получим текущее время, заодно обновим глобальное значение
  getRtcTime();
  xUtime2Tm( &tmpRtc, uxTime);
  rtcCorrAlrm( &tmpRtc );
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
    while ( !( SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk ) ) // wait for underflow
//  SysTick->VAL = 0;
  while ( t_ms > 0 ){
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

static void rtcSetTime( volatile tRtc * prtc ){
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
  RTC->TR = ( RTC->TR & ~(RTC_TR_PM | RTC_TR_HT | RTC_TR_HU | RTC_TR_MNT | RTC_TR_MNU | RTC_TR_ST | RTC_TR_SU)) | temp;
//  // Сбрасываем флаг синхронизации часов
//  RTC->ISR &= ~RTC_ISR_RSF;
  RTC->ISR &= ~RTC_ISR_INIT;
  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;
}


static void rtcSetDate( volatile tRtc * prtc ){
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
  RTC->DR = temp;
//  // Сбрасываем флаг синхронизации часов
//  RTC->ISR &= ~RTC_ISR_RSF;
  RTC->ISR &= ~RTC_ISR_INIT;
  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;
}

void rtcGetTime( volatile tRtc * prtc ){
  uint32_t tmpTr = RTC->TR;
  if( tmpTr != RTC->TR ){
    tmpTr = RTC->TR;
  }
  prtc->hour = BCD2BIN( tmpTr >> 16 );
  prtc->min = BCD2BIN( tmpTr >> 8 );
  prtc->sec = BCD2BIN( tmpTr  );
  prtc->ss = 0x129 - RTC->SSR;
}

void rtcGetDate( volatile tRtc * prtc ){
  uint32_t tmpDr = RTC->DR;
  if( tmpDr != RTC->DR ){
    tmpDr = RTC->DR;
  }
  prtc->year = BCD2BIN( tmpDr >> 16 );
  prtc->month = BCD2BIN( (tmpDr >> 8) & 0x1f );
  prtc->date = BCD2BIN( tmpDr );
  prtc->wday = ( tmpDr >> 13 ) & 0x7;
}

void rtcSetAlrm( tRtc * prtc ){
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
  RTC->CR &= ~RTC_CR_ALRAE;
  while( (RTC->ISR & RTC_ISR_ALRAWF) == RESET )
  {}
  RTC->ALRMAR = temp;

  RTC->CR |= RTC_CR_ALRAE;
  RTC->ISR &= ~RTC_ISR_INIT;
  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;
}

void rtcSetAlrmMask( tRtc * prtc, uint32_t mask ){
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
  RTC->CR &= ~RTC_CR_ALRAE;
  while( (RTC->ISR & RTC_ISR_ALRAWF) == RESET )
  {}
  RTC->ALRMAR = (mask & (RTC_ALRMAR_MSK4 | RTC_ALRMAR_MSK3 | RTC_ALRMAR_MSK2 | RTC_ALRMAR_MSK1)) | temp;

  RTC->CR |= RTC_CR_ALRAE;
  RTC->ISR &= ~RTC_ISR_INIT;
  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;
}

void rtcGetAlrm( tRtc * prtc ){
  prtc->date = BCD2BIN( RTC->ALRMAR >> 24 );
  prtc->hour = BCD2BIN( RTC->ALRMAR >> 16 );
  prtc->min = BCD2BIN( RTC->ALRMAR >> 8);
  prtc->sec = BCD2BIN( RTC->ALRMAR );
}

void rtcCorrAlrm( tRtc * prtc ){
  register uint32_t temp = 0U;

  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->ISR |= RTC_ISR_INIT;
  while((RTC->ISR & RTC_ISR_INITF)!=RTC_ISR_INITF)
  {}

  temp = BIN2BCD( prtc->sec );
  RTC->ALRMAR = ( RTC->ALRMAR & ~(RTC_ALRMAR_ST | RTC_ALRMAR_SU)) | temp;

  RTC->ISR &= ~RTC_ISR_INIT;
  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;
}


void rtcSetWut( uint32_t mks ){
  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
//  RTC->ISR |= RTC_ISR_INIT;
//  while((RTC->ISR & RTC_ISR_INITF)!=RTC_ISR_INITF)
//  {}

  mks /= WUT_K;

  RTC->CR &= ~RTC_CR_WUTE;
  while( (RTC->ISR & RTC_ISR_WUTWF) == RESET )
  {}
  RTC->WUTR = mks;

  RTC->CR |= RTC_CR_WUTE;
//  RTC->ISR &= ~RTC_ISR_INIT;
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
  TIM2->PSC = rccClocks.PCLK1_Frequency/1000000 - 1;
  // Обратный отсчет
  TIM2->CR1 |= TIM_CR1_DIR;
  // Прерывание по обнулению
  TIM2->DIER |= TIM_DIER_UIE;

  NVIC_SetPriority(TIM2_IRQn, 1);
  NVIC_EnableIRQ( TIM2_IRQn);
}

// ================================= ms Timers ==========================================
void timerSetup(struct timer_list *timer, void (*function)(uintptr_t), uintptr_t data)
{
	timer->function = function;
	timer->data     = data;

  timer->entry.next = NULL;
}

///**
//  * @brief  Проверка таймера на ожидание исполнения.
//  *
//  * @param[in]  timer дескриптор таймера
//  *
//  * @retval true/false
//  */
//inline bool timerPending(const struct timer_list *timer){
//  return (timer->entry.next != NULL);
//}

/**
  * @brief	Непосредственное удаление таймера из очереди.
  *
  * @param[in]	timer		дескриптор таймера
  * @param[in]	clear_pending	удаление признака размещения в очереди на исполнение
  *
  * @retval	none
  */
static void timerDetach(struct timer_list *timer, bool clear_pending)
{
	list_del(&timer->entry);

	if (clear_pending)
		timer->entry.next = NULL;
}

bool timerMod(struct timer_list *timer, uint32_t expires) {
	bool  retval = false;

	expires += mTick;

	if (timerPending(timer)) {
		/*
		 * This is a common optimization triggered by the
		 * networking code - if the timer is re-modified
		 * to be the same thing then just return:
		 */
		if (timer->expires == expires)
			return true;

		timerDetach(timer, false);

		retval = true;
	}

	timer->expires = expires;

	list_add_tail(&timer->entry, &msTimersQueue);

	return retval;
}

bool timerModArg(struct timer_list *timer, uint32_t expires, uintptr_t arg) {
  bool  retval = false;

  timer->data = arg;

  expires += mTick;

  if (timerPending(timer)) {
    /*
     * This is a common optimization triggered by the
     * networking code - if the timer is re-modified
     * to be the same thing then just return:
     */
    if (timer->expires == expires)
      return true;

    timerDetach(timer, false);

    retval = true;
  }

  timer->expires = expires;

  list_add_tail(&timer->entry, &msTimersQueue);

  return retval;
}

void timerAdd(struct timer_list *timer)
{
	timerMod(timer, timer->expires - mTick);
}

bool timerDel(struct timer_list *timer)
{
	if (!timerPending(timer))
		return false;

	timerDetach(timer, true);

	return true;
}


// Обработка стека таймеров для добавки
void timerStack( struct timer_list *timer, uint32_t tout, eTimStack ts ){
  static struct {
    struct timer_list *timer;
    uint32_t tout;
    eTimStack ts;
  } timPr[5];
  static uint8_t timCount;

  if( timer != NULL ){
    // Добавляем таймер
    timPr[timCount].timer = timer;
    timPr[timCount].tout = tout;
    timPr[timCount].ts = ts;
    timCount++;
    assert_param( timCount < 5 );
  }
  else {
    for(; timCount; ){
      timCount--;
      if( timPr[timCount].ts == TIMER_MOD ){
        timerMod( timPr[timCount].timer, timPr[timCount].tout );
      }
      else {
        timerDel( timPr[timCount].timer );
      }
    }
  }
}


/**
  * @brief  Обработка данных подсистемы таймеров.
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void timersClock( void ){

  static uint32_t     _prev_jiffies;
  struct list_head    work_list;
  struct list_head   *curr, *next;
  struct timer_list  *timer;

  if (time_after(mTick, _prev_jiffies)) {
    _prev_jiffies = mTick;

    timerStack( NULL, 0, 0 );

    INIT_LIST_HEAD(&work_list);

    list_for_each_safe(curr, next, &msTimersQueue) {
      timer = list_entry(curr, struct timer_list, entry);

      if (time_after(_prev_jiffies, timer->expires))
        list_move_tail(&timer->entry, &work_list);
    }

    while (!list_empty(&work_list)) {
      timer = list_first_entry(&work_list, struct timer_list, entry);

      timerDetach(timer, true);

      if (timer->function != NULL)
        timer->function(timer->data);
    }
  }
}

/**
  * @brief  Инициализация подсистемы таймеров.
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void timersInit( void ) {

  /* Настраиваем системный таймер на заданную частоту. */
  SysTick_Config(rccClocks.SYSCLK_Frequency / TICK_HZ);
}
// ==================================================================================

// ============================= RTC Timers =========================================
void rtcTimSetup(struct timer_list *rtctim, void (*function)(uintptr_t), uintptr_t arg ) {
  rtctim->function = function;
  rtctim->data     = arg;

  rtctim->entry.next = NULL;
}

/**
  * @brief  Непосредственное удаление таймера из очереди.
  *
  * @param[in]  timer   дескриптор таймера
  * @param[in]  clear_pending удаление признака размещения в очереди на исполнение
  *
  * @retval none
  */
static void rtcTimDetach(struct timer_list *rtctim, bool clear ) {
  list_del(&rtctim->entry);

  if( clear ){
    rtctim->entry.next = NULL;
  }
}

bool rtcTimMod(struct timer_list *rtcTim, uint32_t sec) {
  bool  retval = false;

  sec += getRtcTime();

  if (rtcTimPending(rtcTim)) {
    rtcTimDetach(rtcTim, false);
  }

  rtcTim->expires = sec;

  trace_printf("a:%d\n", sec );

  list_add_tail(&rtcTim->entry, &rtcTimQueue);
  actTimListRun = SET;

  return retval;
}

bool rtcTimModArg(struct timer_list *rtcTim, uint32_t sec, uintptr_t arg) {
  rtcTim->data = arg;
  return rtcTimMod( rtcTim, sec );
}

bool rtcTimDel(struct timer_list *rtctim) {
  if (!rtcTimPending( rtctim )) {
    return false;
  }

  rtcTimDetach(rtctim, true);
  actTimListRun = SET;

  return true;
}


// Переустановка на новое время при переводе часов
void rtcTimCorr( int32_t timediff ){
  struct list_head   *curr, *next;
  struct timer_list  *rtcTim;

  list_for_each_safe(curr, next, &actRtcTimQueue) {
    rtcTim = list_entry(curr, struct timer_list, entry);
    // Вносим поправку на новое время
    rtcTim->expires += timediff;
    // Возвращаем таймеры из списка "Активных таймеров" в основной
    list_move_tail( &(rtcTim->entry), &rtcTimQueue );
  }
  list_for_each_safe(curr, next, &rtcTimQueue) {
    rtcTim = list_entry(curr, struct timer_list, entry);
    // Вносим поправку на новое время
    rtcTim->expires += timediff;
  }
}


// Составляем список "Активных" таймеров из ближайших таймеров
uint32_t rtc2ActTimProc( tUxTime ut ){
  struct list_head   *curr, *next;
  struct timer_list  *rtcTim;
  struct timer_list  *tmptim[10];
  uint8_t tcount = 0;
  uint32_t sec = (~0UL);
  uint32_t sec0 = 0;

  list_for_each_safe(curr, next, &actRtcTimQueue) {
    rtcTim = list_entry(curr, struct timer_list, entry);
    // Возвращаем таймеры из списка "Активных таймеров" в основной
    list_move_tail( &(rtcTim->entry), &rtcTimQueue );
  }

  // Составляем список активных RTC-таймеров
  list_for_each_safe(curr, next, &rtcTimQueue) {
    rtcTim = list_entry(curr, struct timer_list, entry);

    sec0 = (int32_t)rtcTim->expires;
    assert_param( ut > 0);
    if(sec0 <= (uint32_t)ut){
      sec0 = ut + 1;
    }
    if( sec0 < sec ){
      // Этот таймер раньше других - Состовляем список заново
      tcount = 0;
      tmptim[tcount++] = rtcTim;
      sec = sec0;
    }
    else if( sec0 == sec ){
      // Этот таймер в то же  время - Добавляем в список самых ранних
      tmptim[tcount++] = rtcTim;
    }
  }

//    assert_param( tcount > 0 );

  // Очистим список
  INIT_LIST_HEAD(&actRtcTimQueue);

  for( ; tcount; ){
    tcount--;
    // Переносим ближайшие таймеры в список "Активных таймеров"
    list_move_tail( &(tmptim[tcount]->entry), &actRtcTimQueue );
  }

  return sec0;
}


FlagStatus actTimProc( void ){
  struct timer_list  *actTim;
  tUxTime ut = getRtcTime();

  // Выполняем все RTC-таймеры, назначеные на это время
  while (!list_empty(&actRtcTimQueue)) {
    actTim = list_first_entry(&actRtcTimQueue, struct timer_list, entry);

    if( (uint32_t)ut < actTim->expires ){
      return -1;
    }
    rtcTimDetach(actTim, true);

    if (actTim->function != NULL) {
      actTim->function(actTim->data);
    }
  }
  return 0;
}


/**
  * @brief Установка делителя "Prescaler" аппаратного таймера для нужной частоты счета
  *         Если делитель больше 0xFFFF, выставляется 0xFFFF и возвращается -1
  *
  * @param[in]  tim устанавливаемый таймер
  * @param[in]  tim_frequency частота срабатывания таймера, Гц
  * @param[out]  psc Значение Далителя таймера TIM_PSC
  *
  * @retval если без ошибок - возвращает "0", иначе, если переполнение, возвращает "-1"
  */
uint8_t timPscSet( TIM_TypeDef * tim, uint32_t tim_frequency, uint16_t * psc){
  uint32_t tmp;
  uint32_t timClk;
  uint32_t prescaler;
  uint8_t rc = SET;

  assert_param(IS_TIM_ALL_PERIPH(tim));
  if( IS_TIM_PCLK1_PERIPH(tim) ){
    // PCLK1 TIMs
    tmp = RCC->CFGR & RCC_CFGR_PPRE1;
    tmp = ((tmp >> 8) & 0x3);
    timClk = rccClocks.PCLK1_Frequency;
  }
  else {
    // PCLK1 TIMs
    tmp = RCC->CFGR & RCC_CFGR_PPRE2;
    tmp = ((tmp >> 11) & 0x3);
    timClk = rccClocks.PCLK2_Frequency;
  }
  if( tmp == 0 ){
    timClk *= 2;
  }
  prescaler = timClk / (tim_frequency) - 1;

  if(prescaler <= 0xFFFF){
    rc = RESET;
  }
  else {
    prescaler = 0xFFFF;
  }
  *psc = (uint16_t)prescaler;

  return rc;
}

/**
  * @brief	Обработчик прерываний SysTick.
  *
  * @param	none
  *
  * @retval	none
  */
void SysTick_Handler(void){
	++mTick;

	// Нужно проверять регулярно и достаточно часто
	uartRxClock( simHnd.rxh );
  uartTxClock( simHnd.txh );

  ledProcess( mTick );
}

