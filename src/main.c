/* Includes ------------------------------------------------------------------*/
#include <gsm.h>
#include <ifaces.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"

#include "isens.h"
#include "stm32l1xx_ll_iwdg.h"
#include "stm32l1xx_ll_rcc.h"
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
extern eGsmRunPhase gsmRunPhase;

//extern const sUartHnd simHnd;

RCC_ClocksTypeDef RCC_Clocks;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void adcProcess( void );

void Configure_IWDG(void);
void Check_IWDG_Reset(void);

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

    // Запускаем watchdog на случай зависания прошивки
//    Check_IWDG_Reset();
//    Configure_IWDG();

//    SIM800.mqttReceive.mqttData = simHnd.rxh->rxFrame;
//    gsmState = GSM_WORK;
//    SIM800.mqttServer.mqttconn = SET;
//    SIM800.mqttServer.tcpconn = SET;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
      /* Refresh IWDG down-counter to default value */
      LL_IWDG_ReloadCounter(IWDG);
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
/**
* @brief  This function configures IWDG
* @param  None
* @retval None
*/
void Configure_IWDG(void){
  /* Enable the peripheral clock of DBG register (uncomment for debug purpose) */
  /* ------------------------------------------------------------------------- */
  /*  LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_APB1_GRP1_IWDG_STOP); */

  /* Enable the peripheral clock IWDG */
  /* -------------------------------- */
  LL_RCC_LSI_Enable();
  while (LL_RCC_LSI_IsReady() != 1)
  {}

  /* Configure the IWDG with window option disabled */
  /* ------------------------------------------------------- */
  /* (1) Enable the IWDG by writing 0x0000 CCCC in the IWDG_KR register */
  /* (2) Enable register access by writing 0x0000 5555 in the IWDG_KR register */
  /* (3) Write the IWDG prescaler by programming IWDG_PR from 0 to 7 - LL_IWDG_PRESCALER_4 (0) is lowest divider*/
  /* (4) Write the reload register (IWDG_RLR) */
  /* (5) Wait for the registers to be updated (IWDG_SR = 0x0000 0000) */
  /* (6) Refresh the counter value with IWDG_RLR (IWDG_KR = 0x0000 AAAA) */
  LL_IWDG_Enable(IWDG);                             /* (1) */
  LL_IWDG_EnableWriteAccess(IWDG);                  /* (2) */
  LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_32);  /* (3) */
  LL_IWDG_SetReloadCounter(IWDG, 0xFEE);            /* (4) */
  while (LL_IWDG_IsReady(IWDG) != 1)                /* (5) */
  {}
  LL_IWDG_ReloadCounter(IWDG);                      /* (6) */
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
}


/**
  * @brief  This function check if the system has resumed from IWDG reset
  * @param  None
  * @retval None
  */
void Check_IWDG_Reset(void)
{
  if (LL_RCC_IsActiveFlag_IWDGRST())
  {
    /* clear IWDG reset flag */
    LL_RCC_ClearResetFlags();
  }
}

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
