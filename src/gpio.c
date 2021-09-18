#include <stddef.h>

#include "main.h"

void setModeGpioPin(sGpioPin *pin, uint32_t mode, uint32_t pull, uint32_t speed, uint8_t af){
  GPIO_InitTypeDef  gpioInitStruct;

  gpioInitStruct.Pin  = pin->pin;
  gpioInitStruct.Mode = pin->mode = mode;
  gpioInitStruct.Pull = pin->pull = pull;
  gpioInitStruct.Speed = pin->speed = speed;
  gpioInitStruct.Alternate = pin->af = af;

  HAL_GPIO_Init(pin->gpio, &gpioInitStruct);
}

void gpioPinSet( sGpioPin *pin ){
  pin->gpio->BSRR = pin->pin;
  pin->newstate = Bit_SET;
}

void gpioPinSetNow( sGpioPin *pin ){
  pin->gpio->BSRR = pin->pin;
  pin->state = pin->newstate = Bit_SET;
  pin->change = SET;
}


void gpioPinReset( sGpioPin * pin ){
  pin->gpio->BSRR = (uint32_t)pin->pin << 16;
  pin->newstate = Bit_RESET;
}

void gpioPinResetNow( sGpioPin * pin ){
  pin->gpio->BSRR = (uint32_t)pin->pin << 16;
  pin->state = pin->newstate = Bit_RESET;
  pin->change = SET;
}

void gpioPinResetNowTout( uintptr_t arg ){
  sGpioPin *pin = (sGpioPin *)arg;

  gpioPinResetNow( pin );
}

void gpioPinCmd( sGpioPin *pin, BitAction act ){
  if( act == Bit_RESET ){
    gpioPinReset( pin );
  }
  else {
    gpioPinSet( pin );
  }
}

void gpioPinCmdNow( sGpioPin *pin, BitAction act ){
  if( act == Bit_RESET ){
    gpioPinResetNow( pin );
  }
  else {
    gpioPinSetNow( pin );
  }
}


void gpioPinSetup(sGpioPin *pin)
{
  GPIO_InitTypeDef  gpioInitStruct = {0};

  /* Установим начальное состояние вывода GPIO для режимов Open-Drain и Push-Pull. */
  if( (pin->mode == GPIO_MODE_OUTPUT_PP) || (pin->mode == GPIO_MODE_OUTPUT_OD) ){
    pin->gpio->BSRR = (pin->state)? (uint32_t)pin->pin : (uint32_t)pin->pin << 16;
  }


  gpioInitStruct.Pin  = pin->pin;
  gpioInitStruct.Mode = pin->mode;
  gpioInitStruct.Pull = pin->pull;
  gpioInitStruct.Speed = pin->speed;
  gpioInitStruct.Alternate = pin->af;

  HAL_GPIO_Init(pin->gpio, &gpioInitStruct);

}

void extiPinSetup(sGpioPin *pin ){
  uint8_t pinNum;
  IRQn_Type irqNum;

  gpioPinSetup( pin );

  pinNum = gpioPinNum( pin->pin );

  // Установим соответствующее входу прерывание
  if( pinNum < 5 ){
    irqNum = EXTI0_IRQn + pinNum;
  }
  else if( pinNum < 10 ){
    irqNum = EXTI9_5_IRQn;
  }
  else {
    irqNum = EXTI15_10_IRQn;
  }
  NVIC_EnableIRQ( irqNum );
  NVIC_SetPriority( irqNum, KEY_IRQ_PRIORITY );
}


bool changePinState(sGpioPin *pin ){

  if (pin->state != pin->newstate) {
    if ( (pin->mode == GPIO_MODE_OUTPUT_PP) || (pin->mode == GPIO_MODE_OUTPUT_OD) ) {
      pin->gpio->BSRR = (pin->newstate)? (uint32_t)pin->pin : (uint32_t)pin->pin << 16;
    }
    pin->state = pin->newstate;
    pin->change = SET;

    return true;
  }

  return false;
}

void changePinStateTout( uintptr_t arg ){
  sGpioPin *pin = (sGpioPin *)arg;
  changePinState( pin );
}

