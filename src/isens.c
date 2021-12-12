/*
 * isens.c
 *
 *  Created on: 30 авг. 2021 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#include "times.h"
#include "logger.h"
#include "main.h"
#include "uspd.h"
#include "isens.h"

// -------------- ДЛЯ ТЕСТА ----------------------------------
#define ISENS_ARCH_TOUT        10
#define ARCH_READ_TOUT         30

struct timer_list isArchTimer;
struct timer_list archReadTimer;

void isensDbTout( uintptr_t arg );

sISens iSens[ISENS_NUM] = {
  {
    .pinIn = {GPIOA, GPIO_PIN_0, 0, 0},
    .isensCount = 0,
    .state = ISENS_DOWN,
  },
  {
    .pinIn = {GPIOA, GPIO_PIN_1, 0, 1},
    .isensCount = 0,
    .state = ISENS_DOWN,
  },
  {
    .pinIn = {GPIOA, GPIO_PIN_2, 0, 2},
    .isensCount = 0,
    .state = ISENS_DOWN,
  },
  {
    .pinIn = {GPIOA, GPIO_PIN_3, 0, 3},
    .isensCount = 0,
    .state = ISENS_DOWN,
  },
//  {
//    .pinIn = {GPIOB, GPIO_PIN_0, GPIO_MODE_IT_RISING_FALLING, GPIO_PULLDOWN, GPIO_SPEED_FREQ_LOW, AF0, Bit_SET, Bit_SET, RESET },
//    .isensCount = 0,
//    .state = ISENS_DOWN,
//  },
//  {
//    .pinIn = {GPIOB, GPIO_PIN_1, GPIO_MODE_IT_RISING_FALLING, GPIO_PULLDOWN, GPIO_SPEED_FREQ_LOW, AF0, Bit_SET, Bit_SET, RESET },
//    .isensCount = 0,
//    .state = ISENS_DOWN,
//  },
};


void isArchTout( uintptr_t arg ){
  uint32_t tout = *((uint32_t *)arg) * TOUT_1000;
  uspd.archWrFlag = SET;
  timerMod( &isArchTimer, tout );
}

void archReadTout( uintptr_t arg ){
  uint32_t tout = *((uint32_t *)arg) * TOUT_1000;
  uspd.readArchSensQuery = SET;
  uspd.readArchEvntQuery = SET;
  timerMod( &archReadTimer, tout );
}

void isensProcess( void ){

  if( uspd.archWrFlag ){
    // Настало время записи датчиков в Архив
    uint32_t isdata[ISENS_NUM];

    for( eIsens s = 0; s < ISENS_NUM; s++ ){
      isdata[s] = iSens[s].isensCount;
    }

    assert_param( ISENS_NUM <= 4 );
    if( logger( getRtcTime(), DEVID_ISENS_1, isdata, ISENS_NUM ) == 1){
      // Записано в Архив успешно
      uspd.archWrFlag = RESET;
    }
    else {
      Error_Handler( NON_STOP );
    }
  }
}


// Обработка сигналов с датчиков Холла
void ISENS_IRQHandler( void ){
  eIsens is;
  uint32_t tm;
  uint32_t dtime;;

  tm = getRtcTime();

  if(ISENS_TIM->SR & TIM_SR_CC1IF){
    // Сработал Датчик 1
    is = ISENS_1;
    ISENS_TIM->SR &= ~TIM_SR_CC1IF;
  }
  else if(ISENS_TIM->SR & TIM_SR_CC2IF){
    // Сработал Датчик 2
    is = ISENS_2;
    ISENS_TIM->SR &= ~TIM_SR_CC2IF;
  }
  else if(ISENS_TIM->SR & TIM_SR_CC3IF){
    // Сработал Датчик 3
    is = ISENS_3;
    ISENS_TIM->SR &= ~TIM_SR_CC3IF;
  }
  else if(ISENS_TIM->SR & TIM_SR_CC4IF){
    // Сработал Датчик 4
    is = ISENS_4;
    ISENS_TIM->SR &= ~TIM_SR_CC4IF;
  }
  else {
//    while(1)
//    {}
    ISENS_TIM->SR = 0;
    return;
  }

  dtime = (tm - iSens[is].tstime) * 1000 + rtc.ss;
  dtime -= iSens[is].tsss;
  if( dtime < 12 ){
    // Слишком короткий период
//    logger( tm, is + DEVID_PULSE_1, NULL, 0 );
  }
  else {
    timerMod( &(iSens[is].dbTimer), ISENS_DB_TOUT );
  }
}




/**
  * @brief  Обработчик тайм-аута антидребезга выводов GPIO.
  *
  * @param[in]  arg данные таймера (дескриптор таймера)
  *
  * @retval none
  */
void isensDbTout( uintptr_t arg ){
  sISens * sens = (sISens *)arg;

  sens->debounceTout = 0;
  // Нынешнее состояния пина
  if( ((sens->pinIn.gpio)->IDR & (sens->pinIn.pin)) ){
    // Состояние сохранилось в "1" - НЕ ложное срабатывание
    sens->isensCount++;
    // Метка времени
    sens->tstime = getRtcTime();
    sens->tsss = rtc.ss;
  }

  return;
}


void isensPinInit( sISens * sens ){
  GPIO_TypeDef * gpio;
  uint8_t pinNum;

  assert_param( sens->pinIn.gpio != NULL );
  // ------------  Вывод Геркона -----------------------
  gpio = sens->pinIn.gpio;
  pinNum = sens->pinIn.pinNum;

  gpio->MODER = (gpio->MODER & ~(0x3 << (pinNum * 2) )) | (0x2 << (pinNum * 2) );
  gpio->PUPDR = (gpio->PUPDR & ~(0x3 << (pinNum * 2) )); // | (0x2 << (pinNum * 2) );
  // Alternate function TIM1_CH1-4 = 6
  gpio->AFR[pinNum / 8] = (gpio->AFR[pinNum / 8] & ~(0xF << ((pinNum % 8) * 4))) | (AF1 << ((pinNum % 8) * 4));
}


void isensTimInit( void ){
  // TIM Init
  uint16_t psc;

  ISENS_TIM_CLK_EN;

  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM2_STOP;

  // Расчет предделителя
  psc = (rccClocks.PCLK1_Frequency/ISENS_TIM_FREQ) - 1;

  ISENS_TIM->PSC = psc;

  ISENS_TIM->CCER = 0;

  ISENS_TIM->ARR = -1;
  ISENS_TIM->CCMR1 = (ISENS_TIM->CCMR1 & ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S))
                      | (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);
  ISENS_TIM->CCMR2 = (ISENS_TIM->CCMR2 & ~(TIM_CCMR2_CC3S | TIM_CCMR2_CC4S))
                      | (TIM_CCMR2_CC3S_0 | TIM_CCMR2_CC4S_0);
  ISENS_TIM->CCER |= TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P | TIM_CCER_CC4P;  // Растущий фронт

  ISENS_TIM->CR1 |= TIM_CR1_URS;
  ISENS_TIM->DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE;

  NVIC_EnableIRQ( ISENS_TIM_IRQn );
  NVIC_SetPriority( ISENS_TIM_IRQn, 0);
}

void isensInit( void ){
  // Вывод
  gpioPinSetup( &gpioPinSensOn );

  for( uint8_t i = 0; i < ISENS_NUM; i++ ){
    isensPinInit( &(iSens[i]) );
    timerSetup( &(iSens[i].dbTimer), isensDbTout, (uintptr_t)&(iSens[i]) );
  }
  // Инициализация таймера, на который подключены ISENS
  isensTimInit();

  gpioPinResetNow( &gpioPinSensOn );

  // ДЛЯ ТЕСТА
  uspdCfg.arxTout = ISENS_ARCH_TOUT;
  timerSetup( &isArchTimer, isArchTout, (uintptr_t)&(uspdCfg.arxTout) );
  timerSetup( &archReadTimer, archReadTout, (uintptr_t)&(uspdCfg.arxTout) );
}


void isensEnable( void ){
  tUxTime tm;
  // Ввод

  // Выбор канала
  ISENS_TIM->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);
  ISENS_TIM->CR1 |= TIM_CR1_CEN;

  tm = getRtcTime();
  for( uint8_t i = 0; i < ISENS_NUM; i++ ){
    // Метка времени
    iSens[i].tstime = tm;
    iSens[i].tsss = rtc.ss;
  }

  // --------------------- ДЛЯ ТЕСТА ----------------------------
  timerMod( &isArchTimer, ISENS_ARCH_TOUT * TOUT_1000 );
  timerMod( &archReadTimer, ARCH_READ_TOUT * TOUT_1000 );
}


/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  (void)GPIO_Pin;
  while(1)
  {}
}

