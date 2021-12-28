/* Includes ------------------------------------------------------------------*/
#include "gsm.h"
#include "ifaces.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"

#include "isens.h"
#include "stm32l1xx_ll_iwdg.h"
#include "stm32l1xx_ll_rcc.h"
#include "mqtt.h"
#include "uspd.h"
#include "logger.h"

#define VER_MAJOR     0x05
#define VER_MINOR     0x01
#define VER_RELEASE   0x0001

#define FW_VERSION    ((VER_MAJOR<<24)|(VER_MINOR<<16)|(VER_RELEASE))

/* Private typedef -----------------------------------------------------------*/
//uint8_t tallocArray[TALLOC_ARRAY_SIZE] __aligned(4);

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern eGsmRunPhase gsmRunPhase;

extern const sUartHnd simHnd;

RCC_ClocksTypeDef RCC_Clocks;


/* Private function prototypes -----------------------------------------------*/
//void adcProcess( void );

void Configure_IWDG(void);
void Check_IWDG_Reset(void);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {

  // Tiny memory allocated init
//  ta_init( tallocArray, tallocArray+TALLOC_ARRAY_SIZE, 256, 16, sizeof(int) );
//  (void)_fw_ver;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Initialize all configured peripherals */
  ifaceInit();

//  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM6_STOP | DBGMCU_APB1_FZ_DBG_TIM7_STOP;

  // Красный светодиод - пока запускается
  ledToggleSet( LED_R, LED_BLINK_ON_TOUT, LED_BLINK_OFF_TOUT, 0, 0 );

  ifaceEnable();

//  flashHwTest();

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
    while (1) {
      /* Refresh IWDG down-counter to default value */
      LL_IWDG_ReloadCounter(IWDG);
      ifaceClock();
    }
}

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


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler( int stop, char * file, int line ){
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    trace_printf( "ErrorHandler: file \"%s\",line %d\n", (uint8_t *)file, line );
    GPIOB->BSRR = GPIO_PIN_9;
    while (stop) {
    }
  /* USER CODE END Error_Handler_Debug */
}


