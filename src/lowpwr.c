/*
 * lowpwr.c
 *
 *  Created on: 1 февр. 2022 г.
 *      Author: jet
 */
#include "diag/trace.h"
#include "lowpwr.h"


uint32_t tmpCount = 0;

static uint32_t gpioaModer;
static uint32_t gpioaPupdr;
static uint32_t gpioaOdr;

//                                 .   .   .   .   .   .   .   .
const uint32_t gpioaModerSl = 0b11101001111101110111010100000000;
const uint32_t gpioaOdrSl = 0x80001000;

static uint32_t gpiobModer;
static uint32_t gpiobPupdr;
static uint32_t gpiobOdr;
//                                 .   .   .   .   .   .   .   .
const uint32_t gpiobModerSl = 0b01011101011101011111010100010000;
const uint32_t gpiobOdrSl = 0x00001010;

static uint32_t gpiocModer;
static uint32_t gpiocPupdr;
static uint32_t gpiocOdr;

const uint32_t gpiocModerSl = 0x00000000;
const uint32_t gpiocOdrSl = 0x00000000;


// XXX: UNTIL DEVELOPING: WUT wakeup flag
volatile FlagStatus sleepFlag = RESET;
// Флаг запуска составления списка активных таймеров
volatile FlagStatus sleepStartFlag = RESET;

// ======================== Function prototype =======================================
//  Переключаем тактирование на MSI
void rccMsiSw( void );
//  Переключаем тактирование на HSI
void rccHsiSw( void );
//void rtcSetWut( uint32_t mks );
uint32_t rtc2ActTimProc( tUxTime ut );
// ===================================================================================

void pwrInit( void ){
  //------------------- Stop mode config -------------------------
  // Stop mode
  PWR->CR &= ~PWR_CR_PDDS;
  // Clear PWR_CSR_WUF
  PWR->CR |= PWR_CR_CWUF;
  // Выключаем VREFIN при остановке + Быстрое просыпание:
  // не ждем, пока восстановится VREFIN, проверяем только при запуске АЦП
  PWR->CR |= PWR_CR_ULP | PWR_CR_FWU | PWR_CR_LPSDSR;
  // Interrupt-only Wakeup, DeepSleep enable, SleepOnExit enable
#if STOP_EN
  SCB->SCR = (SCB->SCR & ~SCB_SCR_SEVONPEND_Msk) | SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk;
#else
  SCB->SCR = (SCB->SCR & ~SCB_SCR_SEVONPEND_Msk) | SCB_SCR_SLEEPDEEP_Msk;// | SCB_SCR_SLEEPONEXIT_Msk;
#endif // STOP_EN
  FLASH->OBR &= ~FLASH_OBR_BOR_LEV;
}

/**
  * @brief  Запуск засыпания
  *
  * @retval none
  */
static void sleepStart( void ){
  /* TODO: Перевод периферии в режим сна:
   * 1. Отключить тактирование ненужной периферии
   * 2. Перевезти неиспользуемые входы в  ANALOG
   * 3. Переключить тактирование с HSI на MCI
   * 4. ...
   */

  // ----------------- 1 . . . ----------------------------
//  RCC->AHBENR &= ~RCC_AHBENR_DMA1EN;

  RCC->APB1ENR &= ~(
      RCC_APB1ENR_TIM6EN
      | RCC_APB1ENR_TIM7EN
      | RCC_APB1ENR_USART3EN
  );
  // ----------------- 2 . . . ----------------------------
  // Сохраняем значения GPIO
  gpioaModer = GPIOA->MODER;
  gpioaPupdr = GPIOA->PUPDR;
  gpioaOdr = GPIOA->ODR;

  GPIOA->MODER = gpioaModerSl;
  GPIOA->PUPDR = 0;
  // Устанавливаем статическое значение, кроме PA13, PA14
  GPIOA->ODR = (gpioaOdrSl & ~(GPIO_PIN_13 | GPIO_PIN_14)) | (GPIOA->ODR & (GPIO_PIN_13 | GPIO_PIN_14));

  gpiobModer = GPIOB->MODER;
  gpiobPupdr = GPIOB->PUPDR;
  gpiobOdr = GPIOB->ODR;

  GPIOB->MODER = gpiobModerSl;
  GPIOB->PUPDR = 0;
  // Устанавливаем статическое значение, кроме PB5
  GPIOB->ODR = (gpiobOdrSl & ~GPIO_PIN_5) | (GPIOB->ODR & GPIO_PIN_5);

  gpiocModer = GPIOC->MODER;
  gpiocPupdr = GPIOC->PUPDR;
  gpiocOdr = GPIOC->ODR;

  GPIOC->MODER = gpiocModerSl;
  GPIOC->PUPDR = 0;
  // Устанавливаем статическое значение
  GPIOC->ODR = gpiocOdrSl;

  // ----------------- 3 . . . ----------------------------
  // Переключение тактирования на MCI
  rccMsiSw();
  // Power range 3
  while( (PWR->CSR & PWR_CSR_VOSF) != 0 )
  {}
  PWR->CR |= PWR_CR_VOS;
  while( (PWR->CSR & PWR_CSR_VOSF) != 0 )
  {}
  PWR->CR &= ~PWR_CR_PVDE;
  // ----------------- 4 . . . ----------------------------
  SysTick->CTRL  &= ~SysTick_CTRL_ENABLE_Msk;

  RCC->APB1ENR &= ~RCC_APB1ENR_PWREN;

  SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;
  sleepFlag = SET;
}


/**
  * @brief  Запуск засыпания
  *
  * @retval none
  */
void sleepStop( void ){
  /* TODO: Перевод периферии в режим работы:
   * 1. Отключить тактирование ненужной периферии
   * 2. Перевезти неиспользуемые входы в  ANALOG
   * 3. Переключить тактирование с HSI на MCI
   * 4. ...
   */

  // ----------------- 1 . . . ----------------------------

//  RCC->AHBENR |= RCC_AHBENR_DMA1EN;

  RCC->APB1ENR |= (
      RCC_APB1ENR_TIM6EN
      | RCC_APB1ENR_TIM7EN
      | RCC_APB1ENR_USART3EN
      | RCC_APB1ENR_PWREN
  );
  PWR->CR = (PWR->CR & ~PWR_CR_LPSDSR) | PWR_CR_CWUF;

  // ----------------- 2 . . . ----------------------------
  // Сохраняем значения GPIO
  GPIOA->MODER = gpioaModer;
  GPIOA->PUPDR = gpioaPupdr;
  GPIOA->ODR = gpioaOdr;

  GPIOB->MODER = gpiobModer;
  GPIOB->PUPDR = gpiobPupdr;
  GPIOB->ODR = gpiobOdr;

  GPIOC->MODER = gpiocModer;
  GPIOC->PUPDR = gpiocPupdr;
  GPIOC->ODR = gpiocOdr;

  // ----------------- 3 . . . ----------------------------
  // Переключение тактирования на MCI
  rccHsiSw();
  // ----------------- 4 . . . ----------------------------
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

  SCB->SCR &= ~((uint32_t)SCB_SCR_SLEEPONEXIT_Msk);

  PWR->CR |= PWR_CR_PVDE;
  // TODO: Убрать, когда сделаем реальное засыпание
  sleepFlag = RESET;
}


// XXX: FOr teting only !!!
// Проверка, что нет RTC-таймеров раньше, чем WUT-таймер
void wutTimeCheck( uint32_t mks ){
  uint32_t wuttime = getRtcTime() + mks/1e6 + 1;
  struct timer_list * tmp;

  if( list_empty(&actRtcTimQueue) ){
    return;
  }

  tmp = list_first_entry( &actRtcTimQueue, struct timer_list, entry );
  assert_param( wuttime <= tmp->expires );
}


/**
  * @brief  Выполняется по срабатыванию будильника.
  *
  * @retval none
  */
static inline void sleepProcess( void ){
  // TODO: Заменить на реальное засыпание
  sleepStart();
  __WFI();
//  while(sleepFlag)
//  {}
}


/**
  * @brief  Обработка данных RTC-таймеров.
  *
  * @retval none
  */
void rtcTimProcess( void ){
  uint32_t ssleep;
  uint32_t ut = getRtcTime();

// =================== CREATE ACTIVE TIMERS LIST ==============================
  if( actTimListRun ){

    ssleep = rtc2ActTimProc( ut );
    // Заводим будильник на время "Активных таймеров"

    if( ssleep ){
      setAlrm( ut + ssleep );
    }
    actTimListRun = RESET;
  }

// =================== SLEEP BEGIN ==============================
  if( sleepStartFlag ){
    sleepProcess();
    sleepStartFlag = RESET;
  }

// =================== ACTIVE TIMERS RUN ==============================
  // Когда проснулись...
  if( actTimRun ){
    rtcAlrmCb();

    actTimRun = RESET;
    return;
  }
}

// ==================================================================================
static inline void rtcStopWut( void ){
  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
//  RTC->ISR |= RTC_ISR_INIT;
//  while((RTC->ISR & RTC_ISR_INITF)!=RTC_ISR_INITF)
//  {}
  RTC->CR &= ~RTC_CR_WUTE;
  while( (RTC->ISR & RTC_ISR_WUTWF) == RESET )
  {}
//  RTC->ISR &= ~RTC_ISR_INIT;
  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;
}


/**
  * @brief  Выполняется по срабатыванию будильника.
  *
  * @retval none
  */
void rtcWakeupCb( void ){
  if( (RTC->ISR & RTC_ISR_ALRAF) && (RTC->CR & RTC_CR_ALRAIE) ){
    actTimRun = SET;

    sleepStop();
    RTC->ISR &= ~RTC_ISR_ALRAF;
    EXTI->PR = EXTI_IMR_MR17;
  }
  if( (RTC->ISR & RTC_ISR_ALRBF) && (RTC->CR & RTC_CR_ALRBIE) ){
    // Alarm B: Every 1secund
    // Refresh IWDG down-counter to default value
    IWDG->KR = 0x0000AAAA;
    RTC->ISR = ~(RTC_ISR_ALRBF | RTC_ISR_INIT) | (RTC->ISR & RTC_ISR_INIT);
    EXTI->PR = EXTI_IMR_MR17;
    tmpCount++;
//    trace_printf( "b: %u\n", getRtcTime() );
  }
  if( (RTC->ISR & RTC_ISR_WUTF) && (RTC->CR & RTC_CR_WUTIE) ){
    // WUT stop
    rtcStopWut();
    RTC->ISR &= ~RTC_ISR_WUTF;
    EXTI->PR = EXTI_IMR_MR20;

    sleepStop();
  }
}



