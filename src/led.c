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
   .baseBigTout = 0,
   .ledToggleCount = 0,
   .baseToggleCount = 0,
   .ledToggleFlag = RESET,
  },
  // LED_R
  {
   .ledPin = {GPIOB, GPIO_PIN_9, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, AF0, Bit_RESET, Bit_RESET, RESET },
   .ledOnTout = 0,
   .ledOffTout = 0,
   .baseBigTout = 0,
   .ledToggleCount = 0,
   .baseToggleCount = 0,
   .ledToggleFlag = RESET,
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


void _ledOn( eLed led, uint32_t endure){
  sLed * pled;

  if( led >= LED_NUM ){
    return;
  }

  pled = &(ledHandle[led]);
  gpioPinSetNow( &(pled->ledPin) );
  if(endure > 0){
    pled->timTout = endure + mTick;
  }
  else {
    pled->timTout = 0;
    pled->ledToggleFlag = RESET;
//    pled->ledToggleCount = pled->baseToggleCount = 0;
//    pled->baseBigTout = 0;
  }
}


/**
  * @brief  Выключает светодиод.
  *
  * @param[in]  arg данные таймера (номер вывода GPIO MCU_LED_N)
  *
  * @retval none
  */
void _ledOff( eLed led, uint32_t endure ){
  sLed * pled;

  if( led >= LED_NUM ){
    return;
  }

  pled = &(ledHandle[led]);
  gpioPinResetNow( &(pled->ledPin) );
  if(endure > 0){
    pled->timTout = endure + mTick;
  }
  else {
    pled->timTout = 0;
    pled->ledToggleFlag = RESET;
//    pled->ledToggleCount = pled->baseToggleCount = 0;
//    pled->baseBigTout = 0;
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
void ledOn( eLed led, uint32_t endure){
  sLed * pled;

  if( led >= LED_NUM ){
    return;
  }

  pled = &(ledHandle[led]);
  gpioPinSetNow( &(pled->ledPin) );
  if(endure > 0){
    pled->timTout = endure + mTick;
  }
  else {
    pled->timTout = 0;
    pled->ledToggleFlag = RESET;
    pled->ledToggleCount = pled->baseToggleCount = 0;
    pled->ledBigTout = pled->baseBigTout = 0;
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
    pled->timTout = endure + mTick;
  }
  else {
    pled->timTout = 0;
    pled->ledToggleFlag = RESET;
    pled->ledToggleCount = pled->baseToggleCount = 0;
    pled->ledBigTout = pled->baseBigTout = 0;
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
    _ledOn( led, pled->ledOnTout );
  }
  else {
    _ledOff( led, pled->ledOffTout );
  }
}


/**
  * @brief  Повторное запускание миганием светодиодом
  *
  * @param[in]  led номер светодиода @ref enum eLed
  *
  * @retval none
  */
void ledToggleReset( eLed led ){
  sLed * pled;

  assert_param( led < LED_NUM );

  pled = &(ledHandle[led]);

  ledToggleSet( led, pled->ledOnTout, pled->ledOffTout, pled->baseToggleCount, pled->baseBigTout );
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
void ledToggleSet( eLed led, uint16_t onendure, uint16_t offendure, uint8_t count, uint16_t bigendure ){
  sLed * pled;

  assert_param( led < LED_NUM );

  pled = &(ledHandle[led]);

  if( bigendure ){
    assert_param( bigendure >= ((onendure + offendure) * count) );
    pled->baseBigTout = bigendure;
    pled->ledBigTout = bigendure + mTick;
    if( pled->ledBigTout == 0 ){
      pled->ledBigTout = 1;
    }
  }
  else {
    pled->ledBigTout = pled->baseBigTout = 0;
  }

  pled = &(ledHandle[led]);
  pled->ledOnTout = onendure;
  pled->ledOffTout = offendure;
  pled->ledToggleCount = pled->baseToggleCount = count;
  pled->ledToggleFlag = SET;
  _ledOn( led, onendure );
}

/**
  * @brief  Перемигивать светодиоды
  *
  * @param[in]  endure длительность свечения и несвечения
  *
  * @retval none
  */
void ledWink( uint32_t endure){
  ledToggleSet( LED_G, endure, endure, 0, 0);
  ledToggleSet( LED_R, endure, endure, 0, 0);
  ledOff( LED_R, endure );
}

void ledAllOff( void ){
  for( eLed led = 0; led < LED_NUM; led++ ){
    ledOff(led, 0);
  }
}

/**
  * @brief  Обработчик тайм-аута антидребезга выводов GPIO.
  *
  * @param[in]  arg данные таймера (дескриптор таймера)
  *
  * @retval none
  */
void ledTimeout( eLed l ){
  sLed *pled = &(ledHandle[l]);

  if( pled->ledToggleFlag ){
    ledToggle( l );
    if( pled->ledToggleCount ){
      // Установлен счетчик моргания - обрабатываем его
      if( --pled->ledToggleCount == 0){
        // Это был последняя вспышка
        pled->ledToggleFlag = 0;
      }
    }
  }
  else {
    _ledOff( l, 0 );
  }
}


void ledProcess( uint32_t tick ){
  for( eLed l = 0; l < LED_NUM; l++ ){
    sLed * pled = &(ledHandle[l]);
    if( (pled->timTout != 0) && (pled->timTout <= tick) ){
      ledTimeout( l );
    }
    if( (pled->ledBigTout != 0) && (pled->ledBigTout <= tick) ){
      ledToggleReset( l );
    }
  }
}

/**
  * @brief  Инициализация выводов и таймеров светодиодов.
  *
  * @param none
  *
  * @retval none
  */
void ledInit( void ){
//  sGpioPin gpioPinNWork = { GPIOF, GPIO_Pin_6, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET };
//  sGpioPin gpioPinNFail = { GPIOF, GPIO_Pin_8, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET };

  // ------------------- LEDS -----------------------
  for( eLed led = LED_G; led < LED_NUM; led++ ){
    gpioPinSetup( &(ledHandle[led].ledPin) );
  }

//  gpioPinSetup( &gpioPinNWork );
//  gpioPinSetup( &gpioPinNFail );

}



