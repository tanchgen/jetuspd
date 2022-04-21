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
#include "lowpwr.h"

#define VER_MAJOR     0x00
#define VER_MINOR     0x02
#define VER_RELEASE   0x0002

#define FW_VERSION    ((VER_MAJOR<<24)|(VER_MINOR<<16)|(VER_RELEASE))

/* Private typedef -----------------------------------------------------------*/
//uint8_t tallocArray[TALLOC_ARRAY_SIZE] __aligned(4);

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern eGsmRunPhase gsmRunPhase;
//extern const sUartHnd simHnd;
//extern struct timer_list tIsArchTimer;

//extern uint32_t tmpCount;
//static uint32_t tmpTick;

RCC_ClocksTypeDef RCC_Clocks;


/* Private function prototypes -----------------------------------------------*/
//void adcProcess( void );

void Configure_IWDG(void);
void Check_IWDG_Reset(void);

//void rtcTimProcess( void );
//void isensProcess( void );
//void mqttCtlProc( SIM800_t * sim );
//void isensTimCorr( void );

void pwrInit( void );

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
  // Tiny memory allocated init
//  ta_init( tallocArray, tallocArray+TALLOC_ARRAY_SIZE, 256, 16, sizeof(int) );

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  pwrInit();
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

    ledOn( LED_R, 0 );
    mDelay( 5000 );

//    timerModArg( &gsmOnToutTimer, SEC_10*1e3, GSM_MQTT_START );

//-----------------------
//    SIM800.mqttReceive.mqttData = simHnd.rxh->rxFrame;

/*
    uspdCfgInit( RESET );
    mqttPubInit();

    gsmState = GSM_WORK;
    SIM800.mqttServer.mqttconn = SET;
    SIM800.mqttServer.tcpconn = SET;

    SIM800.mqttClient.pubFlags.announceEnd = RESET;
    uspdCfg.arxTout = 360;    // Интервал записи сенсоров
//    cfgCalProc( &(uspd.arxCal), defCal, uspdCfg.arxCalStr );
    cfgCalProc( &(uspd.arxCal), "0-50/10 0-23 9 * *", uspdCfg.arxCalStr );
    sensPubAlrmSet( &(uspd.arxCal) );

    setRtcTime( 1649356639 );
    // Переустанавливаем RTC-таймеры
    isensTimCorr();
    SIM800.mqttClient.pubFlags.archPubEnd = SET;
*/

/*
    GPIOB->BSRR = GPIO_PIN_9 << 16;
    getRtcTime();
    for( uint8_t i = 0; i < 10; i++ ){
      wutSleep( 1000e3 );
      rtcTimProcess();
//      GPIOB->ODR ^= GPIO_PIN_9;
    }
    trace_printf( "t: %u.%u\n", getRtcTime(), tmpCount );
    mDelay( 1000 );
    while(1){
      trace_printf( "t: %u.%u\n", getRtcTime(), tmpCount );
      gsmSleep( 10, RESET );
      rtcTimProcess();
      trace_printf( "t: %u.%u\n", getRtcTime(), tmpCount );
      GPIOB->ODR ^= GPIO_PIN_9;
    }
*/

//------------------------------
    while (1) {
      /* Refresh IWDG down-counter to default value */
      LL_IWDG_ReloadCounter(IWDG);
      ifaceClock();
/*
      if( (mTick >30000) && (SIM800.mqttClient.pubFlags.announceEnd == RESET) ){
        SIM800.mqttClient.pubFlags.announceEnd = SET;
        uspd.readArchSensQuery = SET;
        uspd.readArchEvntQuery = SET;
      }

      if( tmpTick == 0) {
        if( (int16_t)uspd.archPktId > 0 ){
          tmpTick = mTick + 100;
        }
      }
      else if(tmpTick < mTick ){
        SIM800.mqttReceive.pktId = uspd.archPktId;
        SIM800.mqttReceive.msgType = MQTT_PUBCOMP;
        mqttCtlProc( &SIM800 );
        tmpTick = 0;
      }
*/
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
    if( stop ){
      __disable_irq();
      while(1)
      {}
    }
  /* USER CODE END Error_Handler_Debug */
}


