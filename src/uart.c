#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "main.h"
#include "stm32l1xx_ll_usart.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_dma.h"
#include "usart_arch.h"
#include "MQTTSim800.h"

//extern SIM800_t SIM800;
//
//void mqttConnectCb( FlagStatus conn );
void simUartRxProc( sUartRxHandle * handle, uint8_t byte );


void uartEnable( sUartRxHandle *rxuart, sUartTxHandle *txuart ){
  rxuart->dma_rx_channel->CCR |= DMA_CCR_EN;
  rxuart->uart->CR1 |= USART_CR1_UE;
  txuart->uart->CR1 |= USART_CR1_UE;
}

void uartDisable( sUartRxHandle *rxuart, sUartTxHandle *txuart ) {
  txuart->dma_tx_channel->CCR &= ~DMA_CCR_EN;
  rxuart->dma_rx_channel->CCR &= ~DMA_CCR_EN;

  rxuart->uart->CR1 &= ~USART_CR1_UE;
  txuart->uart->CR1 &= ~USART_CR1_UE;
}


void do_usart_rx_error_irq(sUartRxHandle *handle)
{
  /* Overrun error interrupt. */
  if( (handle->uart->SR & USART_SR_ORE) != RESET){
    (void)handle->uart->DR;
  }

  if( (handle->uart->SR & USART_SR_NE) != RESET ){
    (void)handle->uart->DR;
  }
  if( (handle->uart->SR & USART_SR_FE) != RESET ){
    (void)handle->uart->DR;
  }
}


void do_usart_dma_tx_channel_irq(sUartTxHandle *handle){

  if( (handle->dma_tx->ISR & handle->dma_tx_it_tcif) != RESET ) {
    handle->dma_tx->IFCR = handle->dma_tx_it_tcif;

  }
}


void do_usart_dma_rx_channel_irq(sUartRxHandle *handle){
  if( (handle->dma_rx->ISR & handle->dma_rx_it_tcif) != RESET ) {
    handle->dma_rx->IFCR = handle->dma_rx_it_tcif;
      // Первая половина не свободна - останавливаем прием
      handle->dma_rx_channel->CCR &= ~DMA_CCR_EN;
  }
}


void do_usart_tx_irq(sUartTxHandle *handle){
  if( (handle->uart->CR1 & USART_CR1_TCIE) && (handle->uart->SR & USART_SR_TC) ){
    handle->uart->CR1 &= ~USART_CR1_TCIE;
    handle->uart->CR1 |= USART_CR1_RE;
  }
}




void uartRxClock(sUartRxHandle *handle){
  uint32_t n_bytes;
//  uint32_t size;
  uint8_t crc = 0;
  uint8_t byte;
  static FlagStatus nzero;

  MAYBE_BUILD_BUG_ON(sizeof(handle->crc) < sizeof(crc));

  if( handle->rxProcFlag == SET ){
    return;
  }

  handle->dma_rx_channel->CCR &= ~DMA_CCR_EN;

  handle->head = (USART_RX_RINGBUFFER_SIZE - handle->dma_rx_channel->CNDTR);

  /* Определим заполненность буфера. */
  nzero = n_bytes = handle->head - handle->tail;

  if ((ptrdiff_t)n_bytes < 0){
    n_bytes += USART_RX_RINGBUFFER_SIZE;
  }

  /* Ограничим число обрабатываемых байт. */
  n_bytes = min(n_bytes, max(USART_RX_RINGBUFFER_SIZE / 4, 1));

  while (n_bytes--) {
    /* Извлекаем очередной принятый байт. */
    byte = handle->rxBuf[handle->tail++];
    handle->tail &= USART_RX_RINGBUFFER_MASK;

    handle->rxFrame[handle->frame_offset++] = byte;

    if( handle == simHnd.rxh ){
      simUartRxProc( handle, byte );
      if( handle->rxProcFlag ){
        // Временно приостоновим обработку принятых данных - до обработки принятого топика
        n_bytes = 0;
      }

    }

    continue;
  }

  if( (nzero == 0) && (handle->rxProcFlag == RESET) ){
    /* Обработали все. Освобождаем буфер пакета. */
    handle->dma_rx_channel->CNDTR = USART_RX_RINGBUFFER_SIZE;
    handle->dma_rx_channel->CCR |= DMA_CCR_EN;
    handle->tail = 0;
  }

  return;
}


/**
  * @brief  Постановка в очередь на передачу данных по интерфейсу UART.
  *
  * @param[in]  sUartTxHandle  дескриптор интерфейса UART TX
  * @param[in]  buf Указатель на буфер данных
  * @param[in]  size Размер буфера данных
  *
  * @retval количество отправленных на передачу байт данных
  */
uint16_t uartSend( sUartTxHandle * handle, uint8_t * buf, size_t size ){
  sTxnode * txnode;

  if(size == 0){
    return size;
  }

  if( (txnode = malloc( sizeof(sTxnode)) ) == NULL ){
    ErrHandler( NON_STOP );
    return 0;
  }

//  trace_printf( "a_node_%x\n", txnode );

  txnode->txbuf = buf;
  txnode->size = size;

  __disable_irq();
  list_add_tail(&txnode->node, &handle->queue);
  __enable_irq();

  return size;
}


/**
  * @brief  Передача данных по интерфейсу UART.
  *
  * @param[in]  sUartTxHandle  дескриптор интерфейса UART TX
  * @param[in]  txnode Указатель на нод буфера данных
  *
  * @retval количество отправленных на передачу байт данных
  */
uint16_t uartTransmit( sUartTxHandle * handle, sTxnode * txnode ){
  /*
   * In order to reload a new number of data items to be transferred
   * into the DMA_CNDTRx register, the DMA channel must be disabled.
   */

  handle->data = txnode->txbuf;

  // Выключаем UART_TX
  handle->uart->CR3 &= ~USART_CR3_DMAT;
  handle->dma_tx_channel->CCR &= ~DMA_CCR_EN;
  handle->dma_tx_channel->CMAR = (uint32_t)handle->data;
  /* Укажем число передаваемых байт. */
  handle->dma_tx_channel->CNDTR = txnode->size;
  handle->dma_tx->IFCR = handle->dma_tx_it_tcif;
  /* Clear the TC bit in the SR register by writing 0 to it. */
  handle->uart->SR &= ~USART_SR_TC;
  /* Activate the channel in the DMA register. */
  handle->dma_tx_channel->CCR |= DMA_CCR_EN;
  handle->uart->CR3 |= USART_CR3_DMAT;

  return txnode->size;
}


void uartTxClock(sUartTxHandle *handle ){

  sTxnode * tx_node;

  if( (handle->uart->SR & USART_SR_TC) == RESET ){
    // Интерфейс НЕ свободен на передачу
    return;
  }

  tx_node = list_empty(&handle->queue) ? NULL : list_first_entry(&handle->queue, sTxnode, node);

  if( tx_node != NULL ){
    uartTransmit( handle, tx_node );
    list_del( &(tx_node->node) );
 //   trace_printf( "f_node_%x\n", tx_node );
    free( tx_node );
  }
}


void uartGpioPinInit( GPIO_TypeDef * gpio, uint16_t pin, USART_TypeDef * uart ){
  LL_GPIO_InitTypeDef gpio_init_struct;

  RCC->AHBENR |= RCC_AHBENR_GPIOAEN << gpioPortNum( gpio );

  gpio_init_struct.Pin  = pin;
  gpio_init_struct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  gpio_init_struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init_struct.Pull = LL_GPIO_PULL_NO;
  gpio_init_struct.Mode = LL_GPIO_MODE_ALTERNATE;
  if( (uart == USART1) || (uart == USART2) || (uart == USART3) ){
    gpio_init_struct.Alternate = LL_GPIO_AF_7;
  }
  else {
    assert_param(0);
  }

  LL_GPIO_Init( gpio, &gpio_init_struct);
}

void uartGpioInit( sUartRxHandle * rxuart, sUartTxHandle * txuart) {

  /* Настройка вывода GPIO TX_UART. */
  if(txuart->gpio_tx != NULL){
    uartGpioPinInit( txuart->gpio_tx, txuart->gpio_pin_tx, txuart->uart );
  }

  /* Настройка вывода GPIO RX_UART. */
  if(rxuart->gpio_rx != NULL){
    uartGpioPinInit( rxuart->gpio_rx, rxuart->gpio_pin_rx, rxuart->uart );
  }
}

void uartInit( sUartRxHandle * rxuart, sUartTxHandle * txuart) {
  LL_USART_InitTypeDef  usartInitStruct;

  // Инициализация RX_UART

  usartInitStruct.BaudRate = rxuart->baudrate;
  usartInitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  usartInitStruct.StopBits = LL_USART_STOPBITS_1;
  usartInitStruct.Parity = LL_USART_PARITY_NONE;
  usartInitStruct.TransferDirection = LL_USART_DIRECTION_NONE;
  if( rxuart != NULL ){
    usartInitStruct.TransferDirection |= LL_USART_DIRECTION_RX;
  }
  if( (txuart != NULL) && (txuart->uart == rxuart->uart) ){
    usartInitStruct.TransferDirection |= LL_USART_DIRECTION_TX;
  }

  usartInitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  usartInitStruct.OverSampling = LL_USART_OVERSAMPLING_16;

  LL_USART_Init( rxuart->uart, &usartInitStruct);

  LL_USART_ConfigAsyncMode(rxuart->uart);

  rxuart->uart->CR3 |= USART_CR3_EIE;
  LL_USART_Enable(rxuart->uart);

  if( (txuart != NULL) && (txuart->uart != rxuart->uart) ){
    // Инициализация TX_UART
    usartInitStruct.TransferDirection |= LL_USART_DIRECTION_TX;
    usartInitStruct.BaudRate = txuart->baudrate;
    LL_USART_Init( txuart->uart, &usartInitStruct);
    txuart->uart->CR3 |= USART_CR3_EIE;
    LL_USART_ConfigAsyncMode(txuart->uart);
    LL_USART_Enable(txuart->uart);
  }


}

/*
 *  Конфигурация DMA_UART:
 */
void uartDmaInit( sUartRxHandle * rxuart, sUartTxHandle * txuart ){
  LL_DMA_InitTypeDef    dmaInitStruct;
  uint8_t chnum;
  DMA_Channel_TypeDef * ch1;

  if( txuart->dma_tx_channel != NULL ){
    /* Настройка передающего DMA. */

//    uint32_t NbData;
//    uint32_t Priority;


    dmaInitStruct.PeriphOrM2MSrcAddress = (uint32_t)(&(txuart->uart->DR));
    dmaInitStruct.MemoryOrM2MDstAddress = (uint32_t)(txuart->data);
    dmaInitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    dmaInitStruct.Mode = LL_DMA_MODE_NORMAL;
    dmaInitStruct.PeriphOrM2MSrcIncMode = 0;
    dmaInitStruct.MemoryOrM2MDstIncMode = DMA_CCR_MINC;
    dmaInitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    dmaInitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    dmaInitStruct.Priority = LL_DMA_PRIORITY_MEDIUM;
    dmaInitStruct.NbData = 0;

    assert_param( txuart->dma_tx == DMA1 );
    ch1 = DMA1_Channel1;
    chnum = txuart->dma_tx_channel - ch1 +1;

    LL_DMA_Init(txuart->dma_tx, chnum, &dmaInitStruct);
    // Стираем флаги прерываний канала 1
    txuart->dma_tx->IFCR = txuart->dma_tx_it_tcif;
    txuart->dma_tx_channel->CCR |= DMA_CCR_TCIE;

    txuart->uart->CR3 |= USART_CR3_DMAT;
  }
  else {
    // DMA не используется
    txuart->uart->CR1 |= USART_CR1_TXEIE;
  }

  if( rxuart->dma_rx_channel != NULL ){
    /* Настройка приемного DMA. */
    dmaInitStruct.PeriphOrM2MSrcAddress = (uint32_t)(&(rxuart->uart->DR));
    dmaInitStruct.MemoryOrM2MDstAddress = (uint32_t)&(rxuart->rxBuf);
    dmaInitStruct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    dmaInitStruct.Mode = DMA_CCR_CIRC;
    dmaInitStruct.PeriphOrM2MSrcIncMode = 0;
    dmaInitStruct.MemoryOrM2MDstIncMode = DMA_CCR_MINC;
    dmaInitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    dmaInitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    dmaInitStruct.Priority = LL_DMA_PRIORITY_HIGH;

    dmaInitStruct.NbData = 0;

    assert_param( txuart->dma_tx == DMA1 );
    ch1 = DMA1_Channel1;
    chnum = rxuart->dma_rx_channel - ch1 + 1;

    LL_DMA_Init(rxuart->dma_rx, chnum, &dmaInitStruct);
    // Принимаем побайтнож
    rxuart->dma_rx_channel->CNDTR = USART_RX_RINGBUFFER_SIZE;
    // Стираем флаги прерываний канала 2 и включаем прерывание
    rxuart->dma_rx->IFCR = rxuart->dma_rx_it_tcif;
   rxuart->dma_rx_channel->CCR |= DMA_CCR_TCIE;

    rxuart->uart->CR3 |= USART_CR3_DMAR;
    rxuart->dma_rx_channel->CCR |= DMA_CCR_EN;
  }
  else {
    // DMA не используется
    rxuart->uart->CR1 |= USART_CR1_RXNEIE;
  }
}

void init_uart_tx_handle(sUartTxHandle *handle){
  INIT_LIST_HEAD(&handle->queue);
}

/**
  * @brief  Инициализация интерфейса UART.
  *
  * @param[in]  txhandle дескриптор TX_UART
  * @param[in]  rxhandle дескриптор RX_UART
  * @param[in]  uartid  идентификатор UART
  *
  * @retval none
  */
void uartIfaceInit( sUartTxHandle * txhandle, sUartRxHandle * rxhandle, eUartId uartid ){
  /* Инициализируем дескриптор USART_RX. */
  rxhandle->uart = uartInitDesc[uartid].uartRx;
  rxhandle->gpio_rx = uartInitDesc[uartid].gpioRx;
  rxhandle->gpio_pin_rx = uartInitDesc[uartid].gpioPinRx;
  rxhandle->dma_rx = uartInitDesc[uartid].dmaRx;
  rxhandle->dma_rx_channel = uartInitDesc[uartid].dmaRxChannel;
  rxhandle->dma_rx_it_tcif = uartInitDesc[uartid].dmaRxItTcif;
  rxhandle->tail = 0;
  rxhandle->baudrate = uartInitDesc[uartid].baudrate;
  rxhandle->rxProcFlag = RESET;

  /* Инициализируем дескриптор USART_TX. */
  txhandle->uart = uartInitDesc[uartid].uartTx;
  txhandle->gpio_tx = uartInitDesc[uartid].gpioTx;
  txhandle->gpio_pin_tx = uartInitDesc[uartid].gpioPinTx;
  txhandle->dma_tx = uartInitDesc[uartid].dmaTx;
  txhandle->dma_tx_channel = uartInitDesc[uartid].dmaTxChannel;
  txhandle->dma_tx_it_tcif = uartInitDesc[uartid].dmaTxItTcif;
  rxhandle->baudrate = uartInitDesc[uartid].baudrate;

  uartGpioInit( rxhandle, txhandle );

  /**USART3 GPIO Configuration
  PB10     ------> USART3_TX
  PB11     ------> USART3_RX
  */
//  PB13     ------> USART3_CTS
  uartGpioPinInit( GPIOB, GPIO_PIN_13, USART3 );
//  PB14     ------> USART3_RTS
  uartGpioPinInit( GPIOB, GPIO_PIN_14, USART3 );

  uartInit( rxhandle, txhandle );

  init_uart_tx_handle(txhandle);
  rxhandle->frame_offset = 0;

  uartDmaInit( rxhandle, txhandle );
}
