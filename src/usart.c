/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stdio.h"

#include "usart.h"

/* USER CODE BEGIN 0 */
void gpioSimUartInit( void );
void gpioTermUartInit( void );
/* USER CODE END 0 */

UART_HandleTypeDef simUart;
DMA_HandleTypeDef simUartDmaRx;
DMA_HandleTypeDef simUartDmaTx;

UART_HandleTypeDef termUart;
DMA_HandleTypeDef termUartDmaRx;
DMA_HandleTypeDef termUartDmaTx;


/* SIM_USART init function */
void simUartInit(void){

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
//  huart3.Instance = USART3;
//  huart3.Init.BaudRate = 115200;
//  huart3.Init.WordLength = UART_WORDLENGTH_8B;
//  huart3.Init.StopBits = UART_STOPBITS_1;
//  huart3.Init.Parity = UART_PARITY_NONE;
//  huart3.Init.Mode = UART_MODE_TX_RX;
//  huart3.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
//  huart3.Init.OverSampling = UART_OVERSAMPLING_16;

  gpioSimUartInit();

  simUart.Instance = USART3;
  simUart.Init.BaudRate = 9600;
  simUart.Init.WordLength = UART_WORDLENGTH_8B;
  simUart.Init.StopBits = UART_STOPBITS_1;
  simUart.Init.Parity = UART_PARITY_NONE;
  simUart.Init.Mode = UART_MODE_TX_RX;
//  simUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  simUart.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  simUart.Init.OverSampling = UART_OVERSAMPLING_16;

  /* USART3 clock enable */
  assert_param( simUart.Instance == USART3 );
  __HAL_RCC_USART3_CLK_ENABLE();

  if (HAL_UART_Init(&simUart) != HAL_OK) {
    Error_Handler( STOP );
  }

  /* USER CODE BEGIN USART3_Init 2 */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);

  /* USER CODE END USART3_Init 2 */

}


void termUartInit(void){

  gpioTermUartInit();

  /* USART1 clock enable */
  termUart.Instance = USART1;
  termUart.Init.BaudRate = 230400;
  termUart.Init.WordLength = UART_WORDLENGTH_8B;
  termUart.Init.StopBits = UART_STOPBITS_1;
  termUart.Init.Parity = UART_PARITY_NONE;
  termUart.Init.Mode = UART_MODE_TX_RX;
  termUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart3.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  termUart.Init.OverSampling = UART_OVERSAMPLING_16;

  assert_param( termUart.Instance == USART1 );
  __HAL_RCC_USART1_CLK_ENABLE();

  if (HAL_UART_Init(&termUart) != HAL_OK) {
    Error_Handler( STOP );
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USART1 DMA Init */
  /* USART1_RX Init */
  termUartDmaRx.Instance = DMA1_Channel5;
  termUartDmaRx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  termUartDmaRx.Init.PeriphInc = DMA_PINC_DISABLE;
  termUartDmaRx.Init.MemInc = DMA_MINC_ENABLE;
  termUartDmaRx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  termUartDmaRx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  termUartDmaRx.Init.Mode = DMA_NORMAL;
  termUartDmaRx.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&termUartDmaRx) != HAL_OK)
  {
    Error_Handler( STOP );
  }

  __HAL_LINKDMA( &termUart, hdmarx, termUartDmaRx);

  /* USART3_TX Init */
  termUartDmaTx.Instance = DMA1_Channel4;
  termUartDmaTx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  termUartDmaTx.Init.PeriphInc = DMA_PINC_DISABLE;
  termUartDmaTx.Init.MemInc = DMA_MINC_ENABLE;
  termUartDmaTx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  termUartDmaTx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  termUartDmaTx.Init.Mode = DMA_NORMAL;
  termUartDmaTx.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&termUartDmaTx) != HAL_OK)
  {
    Error_Handler( STOP );
  }

  __HAL_LINKDMA( &termUart, hdmatx, termUartDmaTx);

  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE END USART1_Init 2 */

}


void MX_DMA_Init(void){

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}


void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle) {
  (void)uartHandle;
}

void gpioSimUartInit( void ){
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  /**USART3 GPIO Configuration
  PB10     ------> USART3_TX
  PB11     ------> USART3_RX
  PB13     ------> USART3_CTS
  PB14     ------> USART3_RTS
  */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11 | GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void gpioTermUartInit( void ){
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  /**USART1 GPIO Configuration
  PA9     ------> USART1_TX
  PA10     ------> USART1_RX
  */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    PB13     ------> USART3_CTS
    PB14     ------> USART3_RTS
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
  if(uartHandle->Instance==USART1)  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PA9     ------> USART3_TX
    PA10     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void){
  HAL_DMA_IRQHandler(&simUartDmaTx);
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void) {
  HAL_DMA_IRQHandler(&simUartDmaRx);
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void) {
  HAL_DMA_IRQHandler(&termUartDmaTx);
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void) {
  HAL_DMA_IRQHandler(&termUartDmaRx);
}


void termSendTime( void ){
  char timeStr[27];

  getRtcTime();
  sprintf( (char*)timeStr, "%02d.%02d.20%02d %02d:%02d:%02d\n", \
                    rtc.date, rtc.month, rtc.year, \
                    rtc.hour, rtc.min, rtc.date );
  HAL_UART_Transmit_DMA( &termUart, (uint8_t*)timeStr, 20 );
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
