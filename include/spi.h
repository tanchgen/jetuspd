/*
 * spi.h
 *
 *  Created on: 22 марта 2019 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#ifndef SPI_H_
#define SPI_H_

//#include "stm32f2xx.h"

#define PLL_SPI_DMA_ENABLE    0
#define MRAM_SPI_DMA_ENABLE    0

/** Приоритет прерываний от SPI */
#define SPI_IRQ_PRIORITY   (0)

#define SPI_NSS_HARD          0

/** Структура дескриптора приемного USART. */
typedef struct {
  /** Дескриптор SPI. */
  SPI_TypeDef  *spi;
#if PLL_SPI_DMA_ENABLE
  /** Дескриптор приемного DMA. */
  DMA_Stream_TypeDef  *dmaRxStream;
  /** Флаг DMA channel transfer complete interrupt приемного DMA. */
  uint32_t  dmaRxTcif;
  IRQn_Type  dmaRxIRQn;

  /** Дескриптор передающего DMA. */
  DMA_Stream_TypeDef  *dmaTxStream;
  /** Флаг DMA channel transfer complete interrupt приемного DMA. */
  uint32_t  dmaTxTcif;
  IRQn_Type  dmaTxIRQn;
#else
  uint16_t xFerLen;
  uint16_t txCount;
#endif // PLL_SPI_DMA_ENABLE

  /** Дескрипторы выводов GPIO. */
  GPIO_TypeDef  *gpioMosi;
  uint16_t  gpioPinMosi;
  GPIO_TypeDef  *gpioMiso;
  uint16_t  gpioPinMiso;
  GPIO_TypeDef  *gpioSck;
  uint16_t  gpioPinSck;
  GPIO_TypeDef  *gpioNss;
  uint16_t  gpioPinNss;

  uint16_t SPI_FirstBit;
  uint16_t SPI_DataSize;
  /** Данные RX-буфера. */
  uint8_t  *rxBuf __aligned(2);
  /** Данные TX-буфера. */
  uint8_t *txBuf __aligned(2);

  void (*txCallback)(void);
  void (*rxCallback)(void);

} sSpiHandle;

extern sSpiHandle * hspiNow;

/**
  * @brief  Configures the GPIO pins SPI Peripheral.
  * @param  hspi Pointer SPI Handle
  * @retval None
  */
void spiGpioInit( sSpiHandle * hspi);

/**
  * @brief  Configures the SPI Peripheral.
  * @param  hspi Pointer SPI Handle
  * @retval None
  */
void spiInit( sSpiHandle * hspi);

#if PLL_SPI_DMA_ENABLE
/**
  * @brief  Configures the DMA for SPI Peripheral.
  * @param  hspi Pointer SPI Handle
  * @retval None
  */
void spiDmaInit( sSpiHandle * hspi);
#endif //PLL_SPI_DMA_ENABLE

/**
  * @brief  Передача-прием данных по SPI
  * @param  hspi Указатель на дескриптор SPI-интерфейса
  * @retval None
  */
void spiXfer( sSpiHandle * hspi, uint16_t len, uint8_t * txbuf, uint8_t * rxbuf );

#if  PLL_SPI_DMA_ENABLE
/**
  * @brief  Обработчик прерывания по SPI
  * @param  hspi Указатель на дескриптор SPI-интерфейса
  * @retval None
  */
void spiIrqHandler( sSpiHandle * hspi );
#endif  // PLL_SPI_DMA_ENABLE

#endif /* SPI_H_ */
