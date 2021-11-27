#ifndef _USART_H
#define _USART_H

#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "stm32l1xx.h"
#include "list.h"

typedef enum {
  BAUD_9600,
  BAUD_19200,
  BAUD_38400,
  BAUD_57600,
  BAUD_115200,
  BAUD_230400,
  BAUD_460800,
  BAUD_NUM
} eBaudrate;

#pragma pack(push, 1)
/** Структура пакета USART версии 2. */
__packed struct usart_frame_v2 {
  /** Идентификатор устройства. */
  uint16_t  dev_id;

  /** Адрес устройства. */
  uint16_t  dev_addr;

  /** Размер данных. */
  uint16_t  data_size;
  /** CRC-16 CCITT заголовка. */
  uint16_t  header_crc;
//  /** Буфер опциональных данных. */
  uint8_t  data[0];
};

/** Структура пакета USART версии 2 на передачу. */
__packed struct usart_tx_frame_v2 {
  /** Заголовок пакета USART. */
  struct usart_frame_v2 fr_v2;

  /** Тип пакета USART. */
  uint16_t  type;

  /** Тип пакета USART. */
  uint16_t  tx_id;

  /** Счетчик ошибок по приему из USART. */
  uint16_t  n_e;

  /** Идентификатор последнего обработанного пакета из USART. */
  uint16_t  n_r;

//  /** Буфер опциональных данных. */
  uint8_t  tx_data[0];
};

#pragma pack(pop)

/** Структура пакета USART версии 2 по приему. */
__packed struct usart_rx_frame_v2 {
  /** Заголовок пакета USART. */
  struct usart_frame_v2 fr_v2;

  /** Тип пакета USART. */
  uint16_t  type;

  /** Идентификатор пакета USART. */
  uint16_t  n_s;

  /** Буфер опциональных данных. */
  uint8_t  rx_data[0];
};

/** Размер приемного кольцевого буфера данных USART. */
#define USART_RX_RINGBUFFER_SIZE      256
/** Маска приемного кольцевого буфера данных USART. */
#define USART_RX_RINGBUFFER_MASK  (USART_RX_RINGBUFFER_SIZE - 1)

/** Размер буфера данных на передачу через USART. */
#define USART_TX_BUFFER_SIZE  (256)


/** Структура дескриптора передающего USART. */
typedef struct {
  /** Дескриптор USART. */
  USART_TypeDef  *uart;

  /** Дескриптор передающего DMA. */
  DMA_TypeDef  *dma_tx;
  DMA_Channel_TypeDef  *dma_tx_channel;
  /** Флаг DMA channel transfer complete interrupt передающего DMA. */
  uint32_t  dma_tx_it_tcif;

  /** Дескриптор выводов GPIO UART_TX. */
  GPIO_TypeDef  *gpio_tx;

  /** Номер вывода GPIO UART_TX. */
  uint16_t  gpio_pin_tx;

  uint32_t baudrate;

  /** Очередь пакетов на передачу. */
  struct list_head  queue;
  /** Нод списка интерфейсов */
  struct list_head ifNode;
  /** Флаг обновления данных */
  FlagStatus txNew;

  /** ИД исходящего пакета */
  uint16_t id;
  // Флаг прямой передачи данных
  FlagStatus rawTx;

  /** Текущее смещение данных относительно начала пакета. */
  size_t  frame_offset;
  /** Текущее значение CRC. */
  uint16_t  crc;
  /** Текущее значение CRC. */
  uint16_t  crc2;
  /** Флаг активного интерфейса */
  uint16_t enBitMask;
  uint8_t uartTest;    // Флаг тестирования UART (отправка пустого пакета - прием эха)

  /** Данные на передачу. */
  uint8_t * data;
} sUartTxHandle;

/** Структура дескриптора приемного USART. */
typedef struct uartRxHandle{
  /** Дескриптор USART. */
  USART_TypeDef  *uart;
  /** Дескриптор приемного DMA. */
  DMA_TypeDef  *dma_rx;
  DMA_Channel_TypeDef  *dma_rx_channel;
  /** Флаг DMA channel transfer complete interrupt приемного DMA. */
  uint32_t  dma_rx_it_htif;
  uint32_t  dma_rx_it_tcif;

  /** Дескрипторы выводов GPIO SDA и SCL. */
  GPIO_TypeDef  *gpio_rx;

  /** Номера выводов GPIO SDA и SCL. */
  uint16_t  gpio_pin_rx;

  uint32_t baudrate;

  volatile FlagStatus half;   // Свободная для записи половина буфера
  /** Текущие положения головы и хвоста кольцевого буфера. */
  volatile size_t  head, tail;
//  /** Текущее положение бегунка кольцевого буфера. */
//  size_t  rb_rover;

  /** Данные кольцевого буфера. */
  uint8_t  rxBuf[USART_RX_RINGBUFFER_SIZE];

  /** Возможность обработки принимаемых данных. */
  FlagStatus rxProcFlag;
  /** Признак обнаруженного байт-стаффинга пакета. */
  FlagStatus  crcWait;

  uint16_t size;

//  /** Буфер для сохранения принимаемых байт потока. */
//  uint8_t  next_byte;

  /** Текущее значение CRC. */
  uint32_t  crc;

  /** Буфер для размещения структуры дескриптора текущего обрабатываемого пакета USART. */
  uint8_t rxFrame[256] __aligned(4);

  /** Текущее смещение данных относительно начала пакета. */
  uint32_t  frame_offset;

  char * reply;
  char * replyBuf;      // Буфер для сохранения отклика от SIM800
  FlagStatus replyFlag;    // Отклик, на который указывает reply, получен
  void (*replyCb)( struct uartRxHandle * rxhnd );
} sUartRxHandle;

typedef struct {
  sUartRxHandle  * rxh;
  sUartTxHandle  * txh;
} sUartHnd;


/**
  * @brief  Обработчик прерываний по ошибкам приема данных подсистемой USART.
  *
  * @param[in]  handle  дескриптор приемного USART
  *
  * @retval none
  */
void do_usart_rx_error_irq(sUartRxHandle *handle);

/**
  * @brief  Обработчик прерываний передачи данных подсистемой USART (RS-485).
  *
  * @param[in]  handle  дескриптор приемного USART
  *
  * @retval none
  */
void do_usart_tx_irq(sUartTxHandle *handle);

/**
  * @brief  Обработчик прерываний приема данных подсистемой USART.
  *
  * @param[in]  handle  дескриптор приемного USART
  *
  * @retval none
  */
void do_usart_rx_irq(sUartRxHandle *handle);

/**
  * @brief  Обработчик прерываний приемного DMA подсистемы USART.
  *
  * @param[in]  handle    дескриптор приемного USART
  *
  * @retval none
  */
void do_usart_dma_rx_channel_irq(sUartRxHandle *handle);

/**
  * @brief  Обработка принятых данных подсистемой USART версии 2.
  *
  * @param[in]  handle  дескриптор приемного USART
  *
  * @retval none
  */
void clock_uart_rx_data_v2(sUartRxHandle *handle);

/**
  * @brief  Обработка передаваемых данных подсистемой USART версии 1.
  *
  * @param[in]  handle  дескриптор передающего USART
  *
  * @retval none
  */
void clock_usart_tx_data_v1(sUartTxHandle *handle);

/**
  * @brief  Обработка передаваемых данных подсистемой USART версии 2.
  *
  * @param[in]  handle  дескриптор передающего USART
  *
  * @retval none
  */
void clock_uart_tx_data_v2(sUartTxHandle  *handle);

/**
  * @brief  Инициализация структуры дескриптора передающего USART.
  *
  * @param[in]  handle  дескриптор передающего USART
  *
  * @retval none
  */
void init_uart_tx_handle(sUartTxHandle  *handle);

/**
  * @brief  Сброс приемного USART версии 2.
  *
  * @param[in]  handle  дескриптор приемного USART
  *
  * @retval none
  */
void _reset_uart_rx_handle_v2(sUartRxHandle *handle);

void uartGpioInit( sUartRxHandle * rxuart, sUartTxHandle * txuart);
void uartInit( sUartRxHandle * rxuart, sUartTxHandle * txuart);
void uartEnable( sUartRxHandle *rxuart, sUartTxHandle *txuart );
void uartDisable( sUartRxHandle *rxuart, sUartTxHandle *txuart );

/*
 *  Конфигурация DMA_UART:
 */
void uartDmaInit( sUartRxHandle * rxuart, sUartTxHandle * txuart );
void uartRxClock(sUartRxHandle *handle);
void uartTxClock(sUartTxHandle *handle, sUartRxHandle *rxHandle);
uint16_t uartTransmit( sUartTxHandle * handle, uint32_t size, uint32_t tout );

#endif /* _USART_H */

