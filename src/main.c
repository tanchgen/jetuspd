/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include <gsm.h>
#include <ifaces.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"

#include "isens.h"

#include "MQTTSim800.h"
/* USER CODE END Includes */


/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define TALLOC_ARRAY_SIZE   8192

uint8_t tallocArray[TALLOC_ARRAY_SIZE] __aligned(4);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern eGsmRunPhase gsmRunPhase;

//extern const sUartHnd simHnd;

RCC_ClocksTypeDef RCC_Clocks;
#define VER_MAJOR     0x00
#define VER_MINOR     0x00
#define VER_RELEASE   0x0001

#define FW_VERSION    ((VER_MAJOR<<24)|(VER_MINOR<<16)|(VER_RELEASE))

const uint32_t  _fw_ver __attribute__ ((section(".fw_ver_section"))) = FW_VERSION;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void adcProcess( void );

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // Tiny memory allocated init
  ta_init( tallocArray, tallocArray+TALLOC_ARRAY_SIZE, 256, 16, sizeof(int) );

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Initialize all configured peripherals */
  ifaceInit();

//  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM6_STOP | DBGMCU_APB1_FZ_DBG_TIM7_STOP;

  // Красный светодиод - пока запускается
  ledToggleSet( LED_R, LED_BLINK_ON_TOUT, LED_BLINK_OFF_TOUT, 0, 0 );

  ifaceEnable();

  trace_puts("Hello USPD!");

    gsmRunPhase = PHASE_NON;

//    SIM800.mqttReceive.mqttData = simHnd.rxh->rxFrame;
//    gsmState = GSM_WORK;
//    SIM800.mqttServer.mqttconn = SET;
//    SIM800.mqttServer.tcpconn = SET;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
      timersClock();
      adcProcess();
      isensProcess();
      mqttProcess();
      gsmProcess();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler( int stop ){
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    GPIOB->BSRR = GPIO_PIN_9;
    while (stop) {
    }
  /* USER CODE END Error_Handler_Debug */
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
