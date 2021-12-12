/*
 * ifaces.h
 *
 *  Created on: 3 окт. 2021 г.
 *      Author: jet
 */

#include "gpio.h"
#include "adc.h"
#include "rtc.h"
#include "usart_arch.h"
#include "isens.h"
#include "logger.h"
#include "ifaces.h"

//void logInit( void );
void SystemClock_Config(void);

void mqttInit(void);
void fwInit( void );

void simUartEnable( void );

void mqttProcess( void );
void gsmProcess( void );
void flashProcess( void );
void logClock( void );

void ifaceInit( void ){
  SystemClock_Config();
	gpioInit();
//  MX_DMA_Init();
  simUartInit();
#if TERM_UART_ENABLE
  termUartInit();
#endif // TERM_UART_ENABLE
  adcInit();
  MX_RTC_Init();
  timeInit();
  logInit();
  mqttInit();
  isensInit();
  fwInit();
}


void ifaceEnable( void ){
  logEnable();
  adcStart();
  simUartEnable();
  isensEnable();
//  loggerHwTest();
//  while(1)
//  {}
//

}

void ifaceClock( void ){
  timersClock();
  adcProcess();
  isensProcess();
  mqttProcess();
  gsmProcess();
  flashProcess();
  logClock();
}
