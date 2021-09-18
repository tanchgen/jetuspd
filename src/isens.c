/*
 * isens.c
 *
 *  Created on: 30 авг. 2021 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#include "isens.h"


static int debounceTout( sISens * sens );

sISens iSens[ISENS_NUM] = {
  {
    .pinIn = {GPIOA, GPIO_PIN_0, GPIO_MODE_IT_RISING_FALLING, GPIO_PULLDOWN, GPIO_SPEED_FREQ_LOW, AF0, Bit_SET, Bit_SET, RESET },
    .isensCount = 0,
    .state = ISENS_DOWN,
  },
  {
    .pinIn = {GPIOA, GPIO_PIN_1, GPIO_MODE_IT_RISING_FALLING, GPIO_PULLDOWN, GPIO_SPEED_FREQ_LOW, AF0, Bit_SET, Bit_SET, RESET },
    .isensCount = 0,
    .state = ISENS_DOWN,
  },
  {
    .pinIn = {GPIOA, GPIO_PIN_2, GPIO_MODE_IT_RISING_FALLING, GPIO_PULLDOWN, GPIO_SPEED_FREQ_LOW, AF0, Bit_SET, Bit_SET, RESET },
    .isensCount = 0,
    .state = ISENS_DOWN,
  },
  {
    .pinIn = {GPIOA, GPIO_PIN_3, GPIO_MODE_IT_RISING_FALLING, GPIO_PULLDOWN, GPIO_SPEED_FREQ_LOW, AF0, Bit_SET, Bit_SET, RESET },
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


void isensProcess( void ){
  for( eIsens s = 0; s < ISENS_NUM; s++ ){
    if( (iSens[s].debounceTout > 0) && (iSens[s].debounceTout < mTick) ){
      if( debounceTout( &(iSens[s])) == 0 ){
        iSens[s].isensCount++;
        iSens[s].isensFlag = SET;
      }
    }
  }
}


/**
  * @brief  Обработчик тайм-аута антидребезга выводов GPIO.
  *
  * @param[in]  arg данные таймера (дескриптор таймера)
  *
  * @retval none
  */
static int debounceTout( sISens * sens ){
  int rc;

  sens->debounceTout = 0;
  // Нынешнее состояния пина
  if( ((sens->pinIn.gpio)->IDR & (sens->pinIn.pin)) ){
    // Состояние сохранилось в "1" - НЕ ложное срабатывание
    rc = 0;
  }
  else {
    rc = 1;
  }
  // Включаем прерывание
  EXTI->IMR |= sens->pinIn.pin;

  return rc;
}


void isensPinInit( sISens * sens ){
  uint8_t irqNum;
  uint8_t pinNum;

  if( sens->pinIn.gpio != NULL ){
    // ------------  Вывод Геркона -----------------------
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*Configure GPIO pin Output Level */
  //  HAL_GPIO_WritePin(sens->pinOut.gpio, sens->pinOut.pin, GPIO_PIN_RESET);

    /* Configure GPIO pin */
  //  GPIO_InitStruct.Pin = sens->pinOut.pin;
  //  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  //  GPIO_InitStruct.Pull = GPIO_NOPULL;
  //  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  //  HAL_GPIO_Init(sens->pinOut.gpio, &GPIO_InitStruct);

    /* Configure PB3 pin as input floating */
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = sens->pinIn.pin;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(sens->pinIn.gpio, &GPIO_InitStruct);
    EXTI->PR = sens->pinIn.pin;
    /* Enable and set EXTI Line0 Interrupt to the lowest priority */

    // Установим соответствующее входу прерывание
    pinNum = gpioPinNum( sens->pinIn.pin );
    if( pinNum < 5 ){
      irqNum = EXTI0_IRQn + pinNum;
    }
    else if( pinNum < 10 ){
      irqNum = EXTI9_5_IRQn + pinNum;
    }
    else {
      irqNum = EXTI15_10_IRQn;
    }
    NVIC_EnableIRQ( irqNum );
    NVIC_SetPriority( irqNum, 2 );
  }
}


void isensInit( void ){
  // Вывод
  gpioPinSetup( &gpioPinSensOn );

  for( uint8_t i = 0; i < ISENS_NUM; i++ ){
    isensPinInit( &(iSens[i]) );
  }
  gpioPinResetNow( &gpioPinSensOn );
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

