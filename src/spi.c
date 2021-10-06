/*
 * spi.h
 *
 *  Created on: 22 марта 2019 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#include "main.h"
#include "logger.h"
#include "spi.h"

#include "stm32l1xx_hal.h"
/**
  * @brief  Initializes SPI GPIO.
  * @retval None
  */
void spiGpioInit( sSpiHandle * hspi ){
  GPIO_InitTypeDef  gpioinitstruct = {0};

  /*** Configure the GPIOs ***/
  /* Enable GPIO clock */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

  /* configure SPI SCK */
  gpioinitstruct.Pin        = hspi->gpioPinSck;
  gpioinitstruct.Mode       = GPIO_MODE_AF_PP;
  gpioinitstruct.Pull       = GPIO_NOPULL;
  gpioinitstruct.Speed      = GPIO_SPEED_FREQ_VERY_HIGH;
  gpioinitstruct.Alternate  = GPIO_AF5_SPI1;
  HAL_GPIO_Init(hspi->gpioSck, &gpioinitstruct);
  /* configure SPI MISO */
  gpioinitstruct.Pin        = hspi->gpioPinMiso;
  gpioinitstruct.Mode       = GPIO_MODE_AF_PP;
  gpioinitstruct.Pull       = GPIO_NOPULL;
  gpioinitstruct.Speed      = GPIO_SPEED_FREQ_VERY_HIGH;
  gpioinitstruct.Alternate  = GPIO_AF5_SPI1;
  HAL_GPIO_Init(hspi->gpioMiso, &gpioinitstruct);
  /* configure SPI MOSI */
  gpioinitstruct.Pin        = hspi->gpioPinMosi;
  gpioinitstruct.Mode       = GPIO_MODE_AF_PP;
  gpioinitstruct.Pull       = GPIO_NOPULL;
  gpioinitstruct.Speed      = GPIO_SPEED_FREQ_VERY_HIGH;
  gpioinitstruct.Alternate  = GPIO_AF5_SPI1;
  HAL_GPIO_Init(hspi->gpioMosi, &gpioinitstruct);

  /* configure SPI HSS */
  hspi->gpioNss->BSRR = hspi->gpioPinNss;
  gpioinitstruct.Pin = hspi->gpioPinNss;
  gpioinitstruct.Mode = GPIO_MODE_OUTPUT_PP;
  gpioinitstruct.Pull = GPIO_NOPULL;
  gpioinitstruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpioinitstruct.Alternate  = GPIO_AF5_SPI1;
  HAL_GPIO_Init( hspi->gpioNss, &gpioinitstruct);


}

/**
  * @brief  Initializes SPI HAL.
  * @retval None
  */
void spiInit( sSpiHandle * hspi ){
  SPI_HandleTypeDef heval_Spi;

  /* DeInitializes the SPI peripheral */
  heval_Spi.Instance = hspi->spi;

  /** Дескрипторы выводов GPIO. */

  /* SPI Config */
  /* SPI baudrate is set to 32 MHz (PCLK2/SPI_BaudRatePrescaler = 32/2 = 16 MHz) */
  heval_Spi.Init.BaudRatePrescaler  = SPI_BAUDRATEPRESCALER_2;
  heval_Spi.Init.Direction          = SPI_DIRECTION_2LINES;
  heval_Spi.Init.CLKPhase           = SPI_PHASE_1EDGE;
  heval_Spi.Init.CLKPolarity        = SPI_POLARITY_LOW;
  heval_Spi.Init.CRCCalculation     = SPI_CRCCALCULATION_DISABLE;
  heval_Spi.Init.CRCPolynomial      = 7;
  heval_Spi.Init.DataSize           = hspi->SPI_DataSize;
  heval_Spi.Init.FirstBit           = hspi->SPI_FirstBit;
  heval_Spi.Init.NSS                = SPI_NSS_SOFT;
  heval_Spi.Init.TIMode             = SPI_TIMODE_DISABLE;
  heval_Spi.Init.Mode               = SPI_MODE_MASTER;

  if (HAL_SPI_Init(&heval_Spi) != HAL_OK)
  {
    /* Should not occur */
    while(1) {};
  }
}



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

}


#endif // PLL_SPI_DMA_ENABLE

