#ifndef _TIMES_H
#define _TIMES_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "list.h"

/** Частота работы системного таймера. */
#define TICK_HZ   (1000)

// Таймауты в мс
#define TOUT_50         50
#define TOUT_100        100
#define TOUT_200        200
#define TOUT_300        300
#define TOUT_500        500
#define TOUT_1000       1000
#define TOUT_1500       1500
#define TOUT_2000       2000
#define TOUT_3000       3000
#define TOUT_5000       5000
#define TOUT_10000      10000
#define TOUT_15000      15000

// Таймауты в с
#define SEC_1           1
#define SEC_2           2
#define SEC_3           3
#define SEC_5           5
#define SEC_10          10
#define SEC_15          15
#define SEC_20          20
#define SEC_30          30
#define SEC_60          60    // 1 мин
#define SEC_120         120   // 2 мин
#define SEC_180         180   // 3 мин
#define SEC_300         300   // 5 мин
#define MIN_1           SEC_60
#define MIN_2           SEC_120
#define MIN_3           SEC_180
#define MIN_5           SEC_300
#define MIN_10          (MIN_1 * 10)
#define MIN_20          (MIN_1 * 20)
#define MIN_30          (MIN_1 * 30)
#define MIN_60          (MIN_1 * 60)
#define HOUR_1          (MIN_60)
#define HOUR_2          (HOUR_1 * 2)
#define HOUR_3          (HOUR_3 * 3)
#define HOUR_12         (HOUR_1 * 12)
#define HOUR_24         (HOUR_1 * 24)
#define DAY_1           (HOUR_24)
#define DAY_2           (DAY_1 * 2)
#define DAY_3           (DAY_1 * 3)
#define DAY_5           (DAY_1 * 5)
#define HOUR_10         (DAY_1 * 10)


/** Тайм-аут переходов при включении. */
/** Тайм-аут переходов между состояниями выводов GPIO FPGA_EN_x или FPGA_EN_x. */
#define PWR_EN_STATE_TOUT       TOUT_100
#define PWR_EN_STATE_TOUT05     TOUT_50
#define PWR_EN_STATE_TOUT1      TOUT_100
#define PWR_EN_STATE_TOUT2      TOUT_200
#define PWR_EN_STATE_TOUT3      TOUT_300
#define PWR_EN_STATE_TOUT5      TOUT_500
#define PWR_EN_STATE_TOUT10     TOUT_1000
#define PWR_EN_STATE_TOUT15     TOUT_1500


///* Таймаут готовности устройств на шине I2C */
//#define I2C_READY_TOUT      2500

/** Тайм-аут процедуры антидребезга входов EXTI. */
#define KEY_DEBOUNCE_TOUT  TOUT_50


#define IS_TIM_ALL_PERIPH(PERIPH) (((PERIPH) == TIM2) || \
                                   ((PERIPH) == TIM3) || \
                                   ((PERIPH) == TIM4) || \
                                   ((PERIPH) == TIM6) || \
                                   ((PERIPH) == TIM7) || \
/*                                   ((PERIPH) == TIM1) ||*/ \
/*                                   ((PERIPH) == TIM8) ||*/ \
                                   ((PERIPH) == TIM9) || \
                                   ((PERIPH) == TIM10) || \
                                   ((PERIPH) == TIM11))



#define IS_TIM_PCLK1_PERIPH(PERIPH) (((PERIPH) == TIM2) || \
                                      ((PERIPH) == TIM3) || \
                                      ((PERIPH) == TIM4) || \
                                      ((PERIPH) == TIM6) || \
                                      ((PERIPH) == TIM7))

#define IS_TIM_PCLK2_PERIPH(PERIPH) (((PERIPH) == TIM1) || \
                                      ((PERIPH) == TIM8) || \
                                      ((PERIPH) == TIM9) || \
                                      ((PERIPH) == TIM10) || \
                                      ((PERIPH) == TIM11))

#define ALARM_UPDATE_TOUT   100

#define ALRM_A      0   // НЕ МЕНЯТЬ !!!!
#define ALRM_B      1   // НЕ МЕНЯТЬ !!!!

#define RTCCLOCK     32768


typedef enum {
  TIMER_DEL,
  TIMER_MOD
} eTimStack;

typedef struct
{
  uint32_t SYSCLK_Frequency;
  uint32_t HCLK_Frequency;
  uint32_t PCLK1_Frequency;
  uint32_t PCLK2_Frequency;
}RCC_ClocksTypeDef;


typedef struct {
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
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
  uint16_t ss;
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

extern volatile tRtc rtc;
extern volatile tUxTime uxTime;
extern volatile uint8_t sendToutFlag;
extern volatile uint8_t minTout;
extern volatile uint8_t minToutRx;
extern volatile uint8_t uxSecTout;


extern volatile uint32_t mTick;
extern RCC_ClocksTypeDef  rccClocks;

// Флаг запуска активных таймеров
extern volatile FlagStatus actTimRun;
// Флаг запуска составления списка активных таймеров
extern volatile FlagStatus actTimListRun;
extern volatile FlagStatus sleepStartFlag;

// *********** Инициализация структуры ВРЕМЯ (сейчас - системное ) ************
void rtcInit(void);
//void timeInit( void );
// Получение системного мремени
uint32_t getTick( void );
tUxTime xTm2Utime( volatile tRtc * prtc );

void timersProcess( void );

void timersHandler( void );
void mDelay( uint32_t del );

void setRtcTime( tUxTime xtime );
tUxTime getRtcTime( void );

void rtcGetDate( volatile tRtc * prtc );
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

void timeInit( void );

/** Инициализатор таймера (аналогичен функции ::timerSetup). */
#define TIMER_INITIALIZER(_function, _data) {	\
		.entry = {NULL, NULL},		\
		.expires = 0,			\
		.function = (_function),	\
		.data = (_data),		\
	}

/** Структура дескриптора таймера. */
struct timer_list {
	/** Точка включения в очередь таймеров. */
	struct list_head  entry;

	/** Время исполнения таймера. */
	uint32_t  expires;

	/** Обработчик тайм-аута. */
	void (*function)(uintptr_t);

	/** Данные таймера (аргумент обработчика тайм-аута). */
	uintptr_t  data;
};

/** Счетчик тиков системного таймера. */
extern volatile uint32_t  mTick;
/// Голова очереди секундных таймеров.
extern struct list_head  rtcTimQueue;
/// Голова очереди секундных таймеров на исполнение.
extern struct list_head  actRtcTimQueue;


/*
 * These inlines deal with timer wrapping correctly. You are 
 * strongly encouraged to use them
 *	1. Because people otherwise forget
 *	2. Because if the timer wrap changes in future you
 *	   won't have to alter your code.
 *
 * time_after(a,b) returns true if the time a is after time b.
 */
#define time_after(a, b)	((int32_t)(b) - (int32_t)(a) < 0)
#define time_before(a, b)	time_after(b, a)

#define time_after_eq(a, b)	((int32_t)(a) - (int32_t)(b) >= 0)
#define time_before_eq(a, b)	time_after_eq(b, a)

/**
  * @brief	Задержка исполнения на заданное число миллисекунд.
  *
  * @param[in]	msecs	число миллисекунд
  *
  * @retval	none
  */
void mDelay(const uint32_t msecs);

/**
  * @brief	Инициализация таймера.
  *
  * @param[in]	timer		дескриптор таймера
  *
  * @retval	none
  */
void timerInit(struct timer_list *timer);

/**
  * @brief	Настройка таймера.
  *
  * @param[in]	timer		дескриптор таймера
  * @param[in]	function	функция-обработчик тайм-аута
  * @param[in]	data		данные таймера (аргумент функции-обработчика тайм-аута)
  *
  * @retval	none
  */
void timerSetup(struct timer_list *timer, void (*function)(uintptr_t), uintptr_t data);

/**
  * @brief	Модификация таймера.
  *
  * @param[in]	timer	дескриптор таймера
  * @param[in]	expires	время исполнения таймера
  *
  * @retval	признак модификации таймера из очереди на ожидание исполнения
  */
bool timerMod(struct timer_list *timer, uint32_t expires);

bool timerModArg(struct timer_list *timer, uint32_t expires, uintptr_t arg);
/**
  * @brief	Запуск таймера.
  *
  * @param[in]	timer	дескриптор таймера
  *
  * @retval	none
  */
void timerAdd(struct timer_list *timer);

/**
  * @brief	Удаление таймера.
  *
  * @param[in]	timer	дескриптор таймера
  *
  * @retval	признак удаления таймера из очереди на ожидание исполнения
  */
bool timerDel(struct timer_list *timer);

/**
  * @brief  Проверка таймера на ожидание исполнения.
  *
  * @param[in]  timer дескриптор таймера
  *
  * @retval true/false
  */
static inline bool timerPending(const struct timer_list *timer){
  return (timer->entry.next != NULL);
}

/**
  * @brief  Проверка таймера на ожидание исполнения.
  *
  * @param[in]  rtctim дескриптор таймера
  *
  * @retval true/false
  */
static inline bool rtcTimPending(const struct timer_list *rtctim){
  return (rtctim->entry.next != NULL);
}

void timerStack( struct timer_list *timer, uint32_t tout, eTimStack ts );

uint8_t timPscSet( TIM_TypeDef * tim, uint32_t tim_frequency, uint16_t * psc);
void timerListClear( void );
void timersClock( void );

void rtcTimSetup(struct timer_list *rtctim, void (*function)(uintptr_t), uintptr_t arg );
bool rtcTimMod(struct timer_list *rtcTim, uint32_t sec);
bool rtcTimModArg(struct timer_list *rtcTim, uint32_t sec, uintptr_t arg);
bool rtcTimDel(struct timer_list *rtctim);

void rtcAlrmCb( void );


#endif /* _TIMES_H */

