/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l1xx_it.c
  * @brief   Interrupt Service Routines.
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <isens.h>
#include "main.h"
#include "stm32l1xx_it.h"

#include "gpio_arch.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
//extern UART_HandleTypeDef simUart;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
//extern DMA_HandleTypeDef hdma_adc;
//extern DMA_HandleTypeDef simUartDmaRx;
//extern DMA_HandleTypeDef simUartDmaTx;
//extern DMA_HandleTypeDef termUartDmaRx;
//extern DMA_HandleTypeDef termUartDmaTx;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/******************************************************************************/
/* STM32L1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l1xx.s).                    */
/******************************************************************************/


void EXTI0_IRQHandler( void ){           // EXTI Line 0
  if( iSens[ISENS_1].pinIn.gpio->IDR & iSens[ISENS_1].pinIn.pin ){
    // Положительный фронт датчика
    iSens[ISENS_1].isensCount++;
    iSens[ISENS_1].isensFlag = SET;
//    iSens[ISENS_1].debounceTout = mTick + DEBOUNCE_TOUT;
  }
  EXTI->PR = GPIO_PIN_0;
}


void EXTI1_IRQHandler( void ){           // EXTI Line 1
  if(EXTI->PR == extiPinSb2Key.pin){
    SB2_KEY_TIM->CR1 |= TIM_CR1_CEN;
    // Запрещаем прерывание на период таймаута дребезга
    EXTI->IMR &= ~(extiPinSb2Key.pin);
    EXTI->PR = extiPinSb2Key.pin;
  }

  EXTI->PR = GPIO_PIN_1;
}


void EXTI2_IRQHandler( void ){           // EXTI Line 2
  EXTI->PR = GPIO_PIN_2;
}


void EXTI3_IRQHandler( void ){           // EXTI Line 3
  if(EXTI->PR == extiPinSb2Key.pin){
    SB1_KEY_TIM->CR1 |= TIM_CR1_CEN;
    // Запрещаем прерывание на период таймаута дребезга
    EXTI->IMR &= ~(extiPinSb1Key.pin);
    EXTI->PR = extiPinSb1Key.pin;
  }

  EXTI->PR = GPIO_PIN_3;
}


void EXTI4_IRQHandler( void ){           // EXTI Line 4
  EXTI->PR = GPIO_PIN_4;
}


/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* Overrun error interrupt. */
  if( (USART3->SR & USART_SR_ORE) != RESET){
    while( USART3->SR & USART_SR_RXNE ){
      (void)USART3->DR;
    }
  }

  /*
   * Noise error interrupt and Framing error interrupt.
   *
   * The NE bit is reset by a USART_SR register read operation followed by a USART_DR
   * register read operation.
   *
   * The FE bit is reset by a USART_SR register read operation followed by a USART_DR
   * register read operation.
   */
  if( ((USART3->SR & USART_SR_NE) != RESET) || ((USART3->SR & USART_SR_FE) != RESET) ){
    (void)USART3->DR;
  }
}


/* USER CODE BEGIN 1 */
void WWDG_IRQHandler( void ) { while(1){} };
void PVD_IRQHandler( void ) { while(1){} };
void TAMPER_STAMP_IRQHandler( void ) { while(1){} };
void RTC_WKUP_IRQHandler( void ) { while(1){} };
void FLASH_IRQHandler( void ) { while(1){} };
void RCC_IRQHandler( void ) { while(1){} };
//void EXTI0_IRQHandler( void ) { while(1){} };
//void EXTI1_IRQHandler( void ) { while(1){} };
//void EXTI2_IRQHandler( void ) { while(1){} };
//void EXTI3_IRQHandler( void ) { while(1){} };
//void EXTI4_IRQHandler( void ) { while(1){} };
void DMA1_Channel1_IRQHandler( void ) { while(1){} };
//void DMA1_Channel2_IRQHandler( void ) { while(1){} };
//void DMA1_Channel3_IRQHandler( void ) { while(1){} };
//void DMA1_Channel4_IRQHandler( void ) { while(1){} };
void DMA1_Channel5_IRQHandler( void ) { while(1){} };
void DMA1_Channel6_IRQHandler( void ) { while(1){} };
void DMA1_Channel7_IRQHandler( void ) { while(1){} };
//void ADC1_IRQHandler( void ) { while(1){} };
void USB_HP_IRQHandler( void ) { while(1){} };
void USB_LP_IRQHandler( void ) { while(1){} };
void DAC_IRQHandler( void ) { while(1){} };
void COMP_IRQHandler( void ) { while(1){} };
void EXTI9_5_IRQHandler( void ) { while(1){} };
void LCD_IRQHandler( void ) { while(1){} };
void TIM9_IRQHandler( void ) { while(1){} };
//void TIM10_IRQHandler( void ) { while(1){} };
//void TIM11_IRQHandler( void ) { while(1){} };
//void TIM2_IRQHandler( void ) { while(1){} };
void TIM3_IRQHandler( void ) { while(1){} };
void TIM4_IRQHandler( void ) { while(1){} };
void I2C1_EV_IRQHandler( void ) { while(1){} };
void I2C1_ER_IRQHandler( void ) { while(1){} };
void I2C2_EV_IRQHandler( void ) { while(1){} };
void I2C2_ER_IRQHandler( void ) { while(1){} };
//void SPI1_IRQHandler( void ) { while(1){} };
void SPI2_IRQHandler( void ) { while(1){} };
//void USART1_IRQHandler( void ) { while(1){} };
void USART2_IRQHandler( void ) { while(1){} };
//void USART3_IRQHandler( void ) { while(1){} };
void EXTI15_10_IRQHandler( void ) { while(1){} };
void RTC_Alarm_IRQHandler( void ) { while(1){} };
void USB_FS_WKUP_IRQHandler( void ) { while(1){} };
//void TIM6_IRQHandler( void ) { while(1){} };
//void TIM7_IRQHandler( void ) { while(1){} };
void SDIO_IRQHandler( void ) { while(1){} };
void TIM5_IRQHandler( void ) { while(1){} };
void SPI3_IRQHandler( void ) { while(1){} };
void UART4_IRQHandler( void ) { while(1){} };
void UART5_IRQHandler( void ) { while(1){} };
void DMA2_Channel1_IRQHandler( void ) { while(1){} };
void DMA2_Channel2_IRQHandler( void ) { while(1){} };
void DMA2_Channel3_IRQHandler( void ) { while(1){} };
void DMA2_Channel4_IRQHandler( void ) { while(1){} };
void DMA2_Channel5_IRQHandler( void ) { while(1){} };
void AES_IRQHandler( void ) { while(1){} };
void COMP_ACQ_IRQHandler( void ) { while(1){} };

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
