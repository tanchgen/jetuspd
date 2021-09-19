/*
 * spi.h
 *
 *  Created on: 22 марта 2019 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#include "main.h"
#include "logger.h"
#include "spi.h"

/**
  * @brief  Передача-прием данных по SPI
  * @param  hspi Указатель на дескриптор SPI-интерфейса
  * @retval None
  */
void spiXfer( sSpiHandle * hspi, uint16_t len, uint8_t * txbuf, uint8_t * rxbuf ){
  if(len == 0 ){
    return;
  }
#if PLL_SPI_DMA_ENABLE
  hspi->dmaRxStream->CR &= ~DMA_SxCR_EN;
  hspi->dmaTxStream->CR &= ~DMA_SxCR_EN;
  if( txbuf != NULL ){
    hspi->dmaTxStream->M0AR = (uint32_t)txbuf;
  }
  else {
    hspi->dmaTxStream->M0AR = (uint32_t)nullbuff;
  }

  if( rxbuf != NULL ){
    hspi->dmaRxStream->M0AR = (uint32_t)rxbuf;
  }
  else {
    hspi->dmaRxStream->M0AR = (uint32_t)nullbuff;
  }
  hspi->dmaTxStream->NDTR = len;
  hspi->dmaRxStream->NDTR = len;

  // Запускаем отправку данных на SPI
#if !SPI_NSS_HARD
  // NSS -> "0"
  hspi->gpioNss->BSRRH = hspi->gpioPinNss;
#endif // SPI_NSS_HARD

  SPI_Cmd( hspi->spi, ENABLE);
  hspi->dmaTxStream->CR |= DMA_SxCR_EN;
  hspi->dmaRxStream->CR |= DMA_SxCR_EN;
}
#else

  // Ждем окончания работы PLL_SPI чтоб выключить PLL_SPI
  while( hspi->spi->SR & SPI_SR_BSY )
  {}

  // Разрешаем прерывания SPI
  while( hspi->spi->SR & SPI_SR_RXNE ){
    (void)hspi->spi->DR;
  }
  if(rxbuf != NULL){
    hspi->rxBuf = rxbuf;
  }
  else {
    hspi->rxBuf = NULL;
  }
  hspi->xFerLen = len;
  hspi->spi->CR2 |= SPI_CR2_RXNEIE;
  if(txbuf != NULL){
    hspi->txBuf = txbuf;
  }
  else {
    hspi->txBuf = NULL;
  }
  hspi->txCount = len;
  hspi->spi->CR2 |= SPI_CR2_TXEIE;
  // Запускаем отправку данных на SPI
#if !SPI_NSS_HARD
  // NSS -> "0"
  hspi->gpioNss->BSRR = hspi->gpioPinNss << 16;
#endif // SPI_NSS_HARD

  hspi->spi->CR1 |= SPI_CR1_SPE;
}

void spiIrqHandler( sSpiHandle * hspi ){
  SPI_TypeDef * SPIx = hspi->spi;

  if( (SPIx->CR2 & SPI_CR2_TXEIE) && (SPIx->SR & SPI_SR_TXE) ){
    if( hspi->txBuf != NULL ){
      SPIx->DR = *((uint8_t*)hspi->txBuf++);
    }
    else {
      SPIx->DR = 0;
    }
    if( --hspi->txCount == 0 ){
      // Передано все
      SPIx->CR2 &= ~SPI_CR2_TXEIE;
      if(hspi->txCallback != NULL){
        hspi->txCallback();
      }
    }
  }
  if( (SPIx->SR & SPI_SR_RXNE) && (SPIx->CR2 & SPI_CR2_RXNEIE) ){
    if(hspi->rxBuf != NULL){
      *hspi->rxBuf++ = SPIx->DR;
    }
    else {
      SPIx->DR;
    }
    if( --(hspi->xFerLen) == 0 ){
      // Принято все
      SPIx->CR2 &= ~SPI_CR2_RXNEIE;
      // Ждем окончания работы PLL_SPI чтоб выключить PLL_SPI
      while( SPIx->SR & SPI_SR_BSY )
      {}
#if !SPI_NSS_HARD
      // NSS -> "1"
      hspi->gpioNss->BSRR = hspi->gpioPinNss;
#endif // SPI_NSS_HARD
      SPIx->CR1 &= ~SPI_CR1_SPE;
      if(hspi->rxCallback != NULL){
        hspi->rxCallback();
      }
    }
  }

}


#endif // PLL_SPI_DMA_ENABLE

