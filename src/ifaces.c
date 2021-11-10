/*
 * ifaces.h
 *
 *  Created on: 3 окт. 2021 г.
 *      Author: jet
 */

#include "gpio.h"
#include "adc.h"
#include "rtc.h"
#include "usart.h"
#include "isens.h"
#include "logger.h"
#include "ifaces.h"

//void logInit( void );
void SystemClock_Config(void);

void mqttInit(void);
void fwInit( void );

void simUartEnable( void );

void ifaceInit( void ){
  SystemClock_Config();
	gpioInit();
//  MX_DMA_Init();
  simUartInit();
  termUartInit();
  adcInit();
  MX_RTC_Init();
  logInit();
  mqttInit();
  isensInit();
  fwInit();
}


void ifaceEnable( void ){
  logEnable();
  adcStart();
  simUartEnable();
//  loggerHwTest();
//  while(1)
//  {}
//

}

