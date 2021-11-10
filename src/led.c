/*
 * led.c
 *
 *  Created on: 17 янв. 2019 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#include <led.h>
#include "main.h"

// ------------------- LEDS -----------------------
sLed ledHandle[ LED_NUM ] = {
    // LED_G
  {
   .ledPin = {GPIOB, GPIO_PIN_8, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, AF0, Bit_RESET, Bit_RESET, RESET },
   .ledOnTout = 0,
   .ledOffTout = 0,
   .ledToggleCount = 0,
   .baseToggleCount = 0,
   .ledToggleFlag = RESET,
   .ledTim = TIM6,
  },
  // LED_R
  {
   .ledPin = {GPIOB, GPIO_PIN_9, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, AF0, Bit_RESET, Bit_RESET, RESET },
   .ledOnTout = 0,
   .ledOffTout = 0,
   .ledToggleCount = 0,
   .baseToggleCount = 0,
   .ledToggleFlag = RESET,
   .ledTim = TIM7,
  },
};



/**
  * @brief  Проверка выключенр ли светодиод
  *
  * @param[in]  led номер светодиода @ref enum eLed
  *
  * @retval FlagStatus
  */
FlagStatus ledIsOff( eLed led ){
  FlagStatus rc;
  sLed * pled;

  if( led >= LED_NUM ){
    return true;
  }

  pled = &(ledHandle[led]);
  rc = ( (gpioPinReadNow( &(pled->ledPin) ) == Bit_RESET)? true: false);
  return rc;
}

/**
  * @brief  Включает светодиод на определенное время
  *
  * @param[in]  led номер светодиода @ref enum eLed
  * @param[in]  endure длительность свечения
  *
  * @retval none
  */
void ledOn( eLed led, uint32_t endure){
  sLed * pled;

  if( led >= LED_NUM ){
    return;
  }

  pled = &(ledHandle[led]);
  gpioPinSetNow( &(pled->ledPin) );
  if(endure > 0){
    // Выставляем таймер
    pled->ledOnTout = endure;
    pled->ledTim->ARR = pled->ledOnTout - 1;
    // Включаем
    pled->ledTim->CR1 |= TIM_CR1_CEN;
  }
  else {
    pled->ledTim->CR1 &= ~TIM_CR1_CEN ;
    pled->ledToggleFlag = RESET;
  }
}

/**
  * @brief  Выключает светодиод.
  *
  * @param[in]  arg данные таймера (номер вывода GPIO MCU_LED_N)
  *
  * @retval none
  */
void ledOff( eLed led, uint32_t endure ){
  sLed * pled;

  if( led >= LED_NUM ){
    return;
  }

  pled = &(ledHandle[led]);
  gpioPinResetNow( &(pled->ledPin) );
  if(endure > 0){
    // Выставляем таймер
    pled->ledOffTout = endure;
    pled->ledTim->ARR = pled->ledOffTout - 1;
    // Включаем
    pled->ledTim->CR1 |= TIM_CR1_CEN;
  }
  else {
    pled->ledTim->CR1 &= ~TIM_CR1_CEN ;
    pled->ledToggleFlag = RESET;
  }
}

/**
  * @brief  Включает светодиод на определенное время
  *
  * @param[in]  led номер светодиода @ref enum eLed
  * @param[in]  endure длительность свечения
  *
  * @retval none
  */
void ledToggle( eLed led ){
  sLed * pled;

  if( led >= LED_NUM ){
    return;
  }

  pled = &(ledHandle[led]);
  if( (pled->ledPin.gpio->ODR & pled->ledPin.pin) == RESET ){
    ledOn( led, pled->ledOnTout );
  }
  else {
    ledOff( led, pled->ledOffTout );
  }
}

/**
  * @brief  Запускает мигание светодиодом
  *
  * @param[in]  led номер светодиода @ref enum eLed
  * @param[in]  onendure длительность свечения
  * @param[in]  offendure длительность НЕ свечения
  *
  * @retval none
  */
void ledToggleSet( eLed led, uint16_t onendure, uint16_t offendure, uint16_t globeendure, uint8_t count ){
  sLed * pled;

  if( led >= LED_NUM ){
    return;
  }

  pled = &(ledHandle[led]);
  pled->ledTim->CR1 &= ~TIM_CR1_CEN;
  pled->ledTim->EGR = TIM_EGR_UG;
//  pled->ledPin.gpio->BSRR = pled->ledPin.pin << 16;
  pled->ledOnTout = onendure;
  pled->ledOffTout = offendure;
  pled->ledBigTout = globeendure;
  pled->ledToggleCount = pled->baseToggleCount = count * 2;
//  pled->ledToggleCount = pled->baseToggleCount - 1;
  pled->ledToggleFlag = SET;
  ledOn( led, onendure );
}

/**
  * @brief  Перемигивать светодиоды
  *
  * @param[in]  endure длительность свечения и несвечения
  *
  * @retval none
  */
void ledWink( uint32_t endure){
  ledHandle[LED_G].ledOnTout = endure;
  ledHandle[LED_G].ledOffTout = endure;
  ledHandle[LED_R].ledOnTout = endure;
  ledHandle[LED_R].ledOffTout = endure;

  ledOn( LED_G, endure );
  ledHandle[LED_G].ledToggleFlag = SET;
  ledOff( LED_R, endure );
  ledHandle[LED_R].ledToggleFlag = SET;
}

void ledAllOff( void ){
  for( eLed led = 0; led < LED_NUM; led++ ){
    ledOff(led, 0);
  }
}

/**
  * @brief  Обработчик тайм-аута антидребезга выводов GPIO.
  *
  * @param[in]  led Номер светодиода
  *
  * @retval none
  */
void ledTimeout( eLed led ){
  sLed *pled = &(ledHandle[led]);

  if( pled->ledToggleFlag ){
    if( pled->ledToggleCount ){
      // Установлен счетчик моргания - обрабатываем его
      if( --pled->ledToggleCount == 0){
        // Это был последняя вспышка
        if( pled->ledBigTout != 0 ){
          uint32_t tmp;
          pled->ledToggleCount = pled->baseToggleCount + 1;
          pled->ledToggleFlag = 1;
          tmp = (pled->ledToggleCount +1) * (pled->ledOnTout + pled->ledOffTout ) / 2;
          pled->ledTim->ARR = (pled->ledBigTout - tmp) - 1;
          pled->ledTim->CR1 |= TIM_CR1_CEN;
        }
        else {
          pled->ledToggleFlag = 0;
        }
      }
      else {
        ledToggle( led );
      }
    }
    else {
      ledToggle( led );
    }
  }
  else {
    ledOff( led, 0 );
  }
}


// Обтаботчик LED_G
void TIM6_IRQHandler( void ){
  ledTimeout( LED_G );
  TIM6->SR &= ~TIM_SR_UIF;
}

// Обтаботчик LED_R
void TIM7_IRQHandler( void ){
  TIM7->SR &= ~TIM_SR_UIF;
  ledTimeout( LED_R );
}


void ledTimInit( TIM_TypeDef * ledtim ){
  // Частота счета 1000 Гц
  ledtim->PSC = (rccClocks.PCLK1_Frequency / 1000) - 1;
  // Время работы таймера 100мс
  ledtim->ARR = 100  -1;
  ledtim->CR1 |= TIM_CR1_URS | TIM_CR1_OPM;
  ledtim->EGR |= TIM_EGR_UG;
  ledtim->DIER |= TIM_DIER_UIE;

}

/**
  * @brief  Инициализация выводов и таймеров светодиодов.
  *
  * @param none
  *
  * @retval none
  */
void ledInit( void ){
  // ------------------- LEDS -----------------------
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; //LED_R_TIM_CLK_EN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

  for( eLed led = LED_G; led < LED_NUM; led++ ){
    gpioPinSetup( &(ledHandle[led].ledPin) );
    ledTimInit( ledHandle[led].ledTim );
  }

  NVIC_SetPriority( TIM6_IRQn, 4 );
  NVIC_EnableIRQ( TIM6_IRQn );
  NVIC_SetPriority( TIM7_IRQn, 4 );
  NVIC_EnableIRQ( TIM7_IRQn );

}



