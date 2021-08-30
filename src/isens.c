/*
 * isens.c
 *
 *  Created on: 30 авг. 2021 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#include "isens.h"

sISens iSens = {
  .pinOut = { GPIOB, GPIO_PIN_4, 1, 4 },
  .pinIn = { GPIOB, GPIO_PIN_3, 1, 3 },
  .isensCount = 0,
  .state = ISENS_DOWN,
};



void isensInit( sISens * sens ){
  // ------------  Вывод Геркона -----------------------
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(sens->pinOut.gpio, sens->pinOut.pin, GPIO_PIN_RESET);

  /* Configure GPIO pin */
  GPIO_InitStruct.Pin = sens->pinOut.pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(sens->pinOut.gpio, &GPIO_InitStruct);

  /* Configure PA0 pin as input floating */
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Pin = sens->pinOut.pin;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = 0;
  HAL_GPIO_Init(sens->pinOut.gpio, &GPIO_InitStruct);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}


/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == iSens.pinIn.pin){
    if( iSens.pinIn.gpio->IDR & iSens.pinIn.pin ){
      iSens.state = ISENS_UP;
    }
    else {
      iSens.state = ISENS_DOWN;
    }
    iSens.isensCount++;
  }
}

