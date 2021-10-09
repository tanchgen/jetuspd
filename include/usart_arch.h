#ifndef _USART_ARCH_H
#define _USART_ARCH_H

#include <stddef.h>

#include "uart.h"

/* ************************************************************************************ *
 *        Конфигурация USART.        *
 * ************************************************************************************ */

#define USART_TX_TOUT   1000

//------------------ SIM_UART -------------------------------------
/** Выбор MCU USART */
#define SIM_UART_ENABLE  1
//#define SIM_UART_ENABLE_MASK    0x1

  /* Инициализируем дескриптор передающего USART1. */
#define SIM_UART                USART3
#define SIM_UART_IRQn           USART3_IRQn
#define SIM_UART_TX_DMA_IRQn    DMA1_Channel2_IRQn
#define SIM_UART_RX_DMA_IRQn    DMA1_Channel3_IRQn

//------------------ TERM_UART -------------------------------------
/** Выбор TERM_USART */
#define TERM_UART_ENABLE  0
//#define TERM_UART_ENABLE_MASK    0x2

  /* Инициализируем дескриптор передающего USART2. */
#define TERM_UART         USART1
#define TERM_UART_IRQn    USART1_IRQn
#define TERM_UART_TX_DMA_IRQn    DMA1_Channel4_IRQn
#define TERM_UART_RX_DMA_IRQn    DMA1_Channel5_IRQn

// ----------------------------------------------------------------

#define STATUS1_FPGA_SIMON     (uint16_t)0x0001
#define STATUS1_FPGA_RST        (uint16_t)0x0002
#define STATUS1_FPGA_PROG       (uint16_t)0x0004
#define STATUS1_VM_PROG      (uint16_t)0x0008
#define STATUS1_DSW_SIMOK      (uint16_t)0x0010
#define STATUS1_CPU_CATERR     (uint16_t)0x0020
#define STATUS1_OTHER_ERR      (uint16_t)0x0040
#define STATUS1_SYS_SIMON      (uint16_t)0x0080
#define STATUS1_FPGA_DONE       (uint16_t)0x0100

#define STATUS1_FPGA_OFF       (uint16_t)0x0200
#define STATUS1_FPGA_ON        (uint16_t)0x0400

#define STATUS1_FPGA_WORK       (uint16_t)0x0800
#define STATUS1_INTEL_RST      (uint16_t)0x1000
#define STATUS1_RESERV_12      (uint16_t)0x2000

#define STATUS1_SYS_OFF        (uint16_t)0x4000
#define STATUS1_SYS_ON         (uint16_t)0x8000


#define STATUS2_FPGA_MBOOT_ADR   (uint16_t)0x0001
#define STATUS2_FPGA_BITSTR_ID   (uint16_t)0x0002
#define STATUS2_FPGA_RSRV1       (uint16_t)0x0004
#define STATUS2_FPGA_RSRV2       (uint16_t)0x0008
#define STATUS2_CMOS_CLR        (uint16_t)0x0010
#define STATUS2_RESERV_6        (uint16_t)0x0020
#define STATUS2_RESERV_7        (uint16_t)0x0040
#define STATUS2_RESERV_8        (uint16_t)0x0080


typedef enum {
  UART_ID_SIM,
  UART_ID_TERM,
} eUartId;

// Дескриптор инициализации UART
typedef struct {
  // UART_RX_DESC
  GPIO_TypeDef  *gpioRx;
  uint16_t  gpioPinRx;
  USART_TypeDef  *uartRx;
  DMA_TypeDef  *dmaRx;
  DMA_Channel_TypeDef  *dmaRxChannel;
  uint32_t  dmaRxItTcif;
  uint32_t  dmaRxItHtif;

  // UART_TX_DESC
  GPIO_TypeDef  *gpioTx;
  uint16_t  gpioPinTx;
  USART_TypeDef  *uartTx;
  DMA_TypeDef  *dmaTx;
  DMA_Channel_TypeDef  *dmaTxChannel;
  uint32_t  dmaTxItTcif;

  GPIO_TypeDef  *gpioTe;
  uint16_t  gpioPinTe;

  uint16_t enBitMask;
  uint32_t baudrate;
} sUartInitDesc;


extern const sUartInitDesc uartInitDesc[];
extern const sUartHnd simHnd;
extern const sUartHnd termHnd;

bool uartAddrTest( sUartRxHandle *handle );

/**
  * @brief  Инициализация интерфейса UART.
  *
  * @param[in]  txhandle дескриптор TX_UART
  * @param[in]  rxhandle дескриптор RX_UART
  * @param[in]  uartid  идентификатор UART-интерфейса
  *
  * @retval none
  */
void uartIfaceInit( sUartTxHandle * txhandle, sUartRxHandle * rxhandle, eUartId uartid );

#if SIM_UART_ENABLE
/**
  * @brief  Инициализация интерфейса SIM_UART.
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void simUartInit( void );

/**
  * @brief  Разрешение работы интерфейса SIM_UART.
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void simUartEnable( void );

/**
  * @brief  Обработка данных интерфейса  SIM_UART.
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void simUartClock( void );
#endif // DBG_UART_ENABLE

#if TERM_UART_ENABLE
/**
  * @brief  Инициализация интерфейса TERM_UART (USART2).
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void termUartInit( void );

/**
  * @brief  Разрешение работы интерфейса TERM_UART (USART2).
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void termUartEnable( void );

/**
  * @brief  Обработка данных интерфейса  TERM_UART (USART2).
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void termUartClock( void );
#endif // TERM_UART_ENABLE

#endif /* _USART_ARCH_H */

