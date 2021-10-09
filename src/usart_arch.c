#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "main.h"

#include "gpio_arch.h"
#include "usart_arch.h"
//#include "arch_mco.h"


/** Глобальный буфер данных телеметрии на передачу */
//sTmdataTxFrame  txDataFrame;

//uint8_t txFrameCount = 0;

///** Структура пакета по приему из USART1. */
//typedef struct __packed {
//  /** Заголовок пакета USART по приему. */
//  sXferFrameRx rxfr;
//
//  /** Адрес записываемого регистра. */
//  uint16_t  reg_addr;
//  /** Значение, записываемое в регистр. */
//  uint16_t  reg_data;
//} sUartFrameRx;


// UART_RX_DESC
GPIO_TypeDef  *gpio_rx;
uint16_t  gpio_pin_rx;
USART_TypeDef  *uart_rx;
DMA_Channel_TypeDef  *dma_rx_channel;
uint32_t  dma_rx_it_tcif;

// UART_TX_DESC
GPIO_TypeDef  *gpio_tx;
uint16_t  gpio_pin_tx;
USART_TypeDef  *uart;
DMA_Channel_TypeDef  *dma_tx_channel;
uint32_t  dma_tx_it_tcif;

GPIO_TypeDef  *gpio_te;
uint16_t  gpio_pin_te;

uint16_t enBitMask;

const sUartInitDesc uartInitDesc[] = {
#if SIM_UART_ENABLE
    // SIM_UART
    {
      .gpioRx = GPIOB,
      .gpioPinRx = GPIO_PIN_10,
      .uartRx = USART3,
      .dmaRx = DMA1,
      .dmaRxChannel = DMA1_Channel3,
      .dmaRxItTcif = DMA_IFCR_CTCIF3,
      .dmaRxItHtif = DMA_IFCR_CHTIF3,
      .uartTx = USART3,
      .gpioTx = GPIOB,
      .gpioPinTx = GPIO_PIN_11,
      .dmaTx = DMA1,
      .dmaTxChannel = DMA1_Channel2,
      .dmaTxItTcif = DMA_IFCR_CTCIF2,
      .baudrate = 9600,
    },
#endif // SIM_UART_ENABLE
#if TERM_UART_ENABLE
    // TER_UART
    {
      .gpioRx = GPIOA,
      .gpioPinRx = GPIO_PIN_10,
      .uartRx = USART1,
      .dmaRx = DMA1,
      .dmaRxChannel = DMA1_Channel5,
      .dmaRxItTcif = DMA_IFCR_CTCIF5,
      .dmaRxItHtif = DMA_IFCR_CHTIF5,
      .uartTx = USART1,
      .gpioTx = GPIOA,
      .gpioPinTx = GPIO_PIN_9,
      .dmaTx = DMA1,
      .dmaTxChannel = DMA1_Channel4,
      .dmaTxItTcif = DMA_IFCR_CTCIF4,
      .baudrate = 9600,
    },
#endif // TERM_UART_ENABLE
};

#if SIM_UART_ENABLE
/** Структура дескриптора приемного SIM_UART. */
static sUartRxHandle  simUartRxHandle;
/** Структура дескриптора передающего SIM_UART. */
sUartTxHandle  simUartTxHandle;
struct timer_list simEnTimer;
const sUartHnd simHnd = { &simUartRxHandle, &simUartTxHandle };
#endif


#if TERM_UART_ENABLE
/** Структура дескриптора приемного TERM_UART. */
static sUartRxHandle  termUartRxHandle;
/** Структура дескриптора передающего TERM_UART. */
sUartTxHandle  termUartTxHandle;
struct timer_list termEnTimer;

const sUartHnd termHnd = { &termUartRxHandle, &termUartTxHandle };
#endif

//=====================================================================================
//void simUartTxClock(sUartTxHandle *handle);



void uartEnTimeout( uintptr_t arg ){
  uartEnable( ((sUartHnd *)arg)->rxh, ((sUartHnd *)arg)->txh );
}


//void uartRxProcess( eRegCode reg, uint16_t regdata, uint8_t dev ){
//  (void)reg;
//  (void)regdata;
//  (void)dev;
//}
//

// ---------------------- SIM_UART ---------------------------------------------------
#if SIM_UART_ENABLE

/**
  * @brief  Обработчик прерываний SIM_UART (USART1).
  *
  * @param  none
  *
  * @retval none
  */
void USART1_IRQHandler(void){
  do_usart_rx_error_irq(&simUartRxHandle);
}

/**
  * @brief  Обработчик прерываний DMA SIM_UART_RX (DMA2_Stream6).
  *
  * @param  none
  *
  * @retval none
  */
void DMA1_Channel4_IRQHandler( void ){
  do_usart_dma_rx_channel_irq(&simUartRxHandle);
}

/**
  * @brief  Инициализация интерфейса SIM_UART (USART1).
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void simUartInit( void ) {

#if DEBUG_TRACE
  trace_puts("Function: Init SIM_UART");
#endif

  assert_param(uartInitDesc[UART_ID_SIM].dmaRx == DMA1);
  assert_param(uartInitDesc[UART_ID_SIM].dmaTx == DMA1);
  assert_param(uartInitDesc[UART_ID_SIM].uartRx == USART3);
  assert_param(uartInitDesc[UART_ID_SIM].uartTx == USART3);

  /* Enable the USART1 peripheral clock and clock source ****************/
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

  RCC->AHBENR |= RCC_AHBENR_DMA1EN;

  uartIfaceInit( &simUartTxHandle, &simUartRxHandle, UART_ID_SIM );
  NVIC_SetPriority( DMA1_Channel4_IRQn, 1 );
  timerSetup( &simEnTimer, uartEnTimeout, (uintptr_t)&simHnd );

#if SIM_UART_TEST_ENABLE
  simUartTxHandle.uartTest = 4;
  simUartRxHandle.uartTest = SET;
#else
  simUartTxHandle.uartTest = 0;
  simUartRxHandle.uartTest = RESET;
#endif
}

/**
  * @brief  Разрешение работы интерфейса SIM_UART (USART1).
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void simUartEnable( void ){
#if DEBUG_TRACE
  trace_puts("Function: Enable SIM_UART");
#endif

  NVIC_EnableIRQ( SIM_UART_RX_DMA_IRQn );
  NVIC_SetPriority( SIM_UART_RX_DMA_IRQn, 1);
  NVIC_EnableIRQ( SIM_UART_IRQn );
  NVIC_SetPriority( SIM_UART_IRQn, 2);

  uartEnable( simHnd.rxh, simHnd.txh );
}

/**
  * @brief  Обработка данных интерфейса  SIM_UART (USART3).
  *   - Вызывается из SysTick_Handler
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void simUartClock( void ){
#if DEBUG_TRACE
  trace_puts("Function: Clock SIM_UART");
#endif

  /* Обработаем данные SIM_UART. */
  uartRxClock(&simUartRxHandle);
//  simUartTxClock( simHnd.txh );
}

#endif
// -----------------------------------------------------------------------------------

// ---------------------- TERM_UART ---------------------------------------------------
#if TERM_UART_ENABLE

/**
  * @brief  Обработчик прерываний TERM_UART (UART2).
  *
  * @param  none
  *
  * @retval none
  */
void USART2_IRQHandler(void) {
  do_usart_rx_error_irq(&termUartRxHandle);
  do_usart_tx_irq(&termUartTxHandle);
}

/**
  * @brief  Обработчик прерываний DMA TERM_UART_RX (DMA1_Channel2).
  *
  * @param  none
  *
  * @retval none
  */
void DMA1_Channel2_IRQHandler( void ){
  do_usart_dma_rx_channel_irq(&termUartRxHandle);
}

/**
  * @brief  Инициализация интерфейса TERM_UART (USART4).
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void termUartInit( void ) {

#if DEBUG_TRACE
  trace_puts("Function: Init TERM_UART");
#endif

  MAYBE_BUILD_BUG_ON(uartInitDesc[UART_ID_TERM].dmaRx != DMA1);
  MAYBE_BUILD_BUG_ON(uartInitDesc[UART_ID_TERM].dmaTx != DMA1);
  MAYBE_BUILD_BUG_ON(uartInitDesc[UART_ID_TERM].uartRx != USART2);
  MAYBE_BUILD_BUG_ON(uartInitDesc[UART_ID_TERM].uartTx != USART2);

  /* Enable the USART1 peripheral clock and clock source ****************/
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
  /* Set clock source */
  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);

  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;

  uartIfaceInit( &termUartTxHandle, &termUartRxHandle, UART_ID_TERM );
  NVIC_SetPriority( DMA1_Channel2_IRQn, 1 );

  timerSetup( &termEnTimer, bldcUartEnable, (uintptr_t)&termHnd );

#if TERM_UART_TEST_ENABLE
  termUartTxHandle.uartTest = 4;
  termUartRxHandle.uartTest = SET;
#else
  termUartTxHandle.uartTest = 0;
  termUartRxHandle.uartTest = RESET;
#endif
}

/**
  * @brief  Разрешение работы интерфейса TERM_UART (UART4).
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void termUartEnable( void ){

#if DEBUG_TRACE
  trace_puts("Function: Enable TERM_UART");
#endif

  enable_nvic_irq( TERM_UART_RX_DMA_IRQn, CONFIG_USART_IRQ_PRIORITY);
  enable_nvic_irq( TERM_UART_IRQn, CONFIG_USART_IRQ_PRIORITY);

  timerMod( &termEnTimer, 3000 );

}

/**
  * @brief  Обработка данных интерфейса TERM_UART.
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void termUartClock( void ){

#if DEBUG_TRACE
  trace_puts("Function: Clock TERM_UART");
#endif

  /* Обработаем данные USART1. */
  uartRxClock(&termUartRxHandle);

#if TERM_UART_TEST_ENABLE
  if( termUartRxHandle.uartTest == 0){
    termUartTxHandle.uartTest = 0;
  }
#else
  termUartTxHandle.uartTest = 4;
#endif
  uartTxClock( termHnd.txh, termHnd.rxh );
}
#endif
// -------------------------------------------------------------------------------------


