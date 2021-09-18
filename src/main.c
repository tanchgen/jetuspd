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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "isens.h"
#include "my_ntp.h"
#include "adc.h"
#include "dma.h"
#include "rtc.h"
#include "usart.h"

#include "MQTTSim800.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ADC parameters */
#define ADCCONVERTEDVALUES_BUFFER_SIZE ((uint32_t)    5)    /* Size of array containing ADC converted values: set to ADC sequencer number of ranks converted, to have a rank in each address */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RCC_ClocksTypeDef RCC_Clocks;

/* Variable containing ADC conversions results */
__IO uint16_t   aADCxConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE];

/* USER CODE BEGIN PV */
SIM800_t SIM800;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
int clkSet(void);
int simStartInit(void);
int gprsConn( void );
int ntpInit(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == UART_SIM800) {
        Sim800_RxCallBack();
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  gpioInit();

//  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM6_STOP | DBGMCU_APB1_FZ_DBG_TIM7_STOP;

  // Красный светодиод - пока запускается
  ledToggleSet( LED_R, LED_BLINK_ON_TOUT, LED_BLINK_OFF_TOUT, 0, 0 );
  /* USER CODE END SysInit */
//
//  mDelay( 1000 );
//
//  while(1){
//  ledToggleSet( LED_R, LED_BLINK_ON_TOUT, LED_TOGGLE_TOUT, TOUT_3000, 2 );
//
//  mDelay( 7000 );
//  ledOff( LED_R, 0 );
//  mDelay( 2000 );
//  ledToggleSet( LED_R, LED_BLINK_ON_TOUT, LED_TOGGLE_TOUT, TOUT_3000, 2 );
//  }

  /* Initialize all configured peripherals */
  MX_DMA_Init();
  simUartInit();
  termUartInit();
  MX_ADC_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  if (HAL_ADC_Start_DMA(&hadc,
                         (uint32_t *)aADCxConvertedValues,
                         ADCCONVERTEDVALUES_BUFFER_SIZE
                        ) != HAL_OK)
  {
     /* Start Error */
    Error_Handler( STOP );
  }

  isensInit();

  // On SIM800 power if use
  gpioPinResetNow( &gpioPinSimPwr );
  gpioPinSetNow( &gpioPinPwrKey );
  mDelay(2000);
  gpioPinResetNow( &gpioPinPwrKey );
  mDelay(10000);

  // Две вспышки красного цвета с интервалом в 3 сек
  ledToggleSet( LED_R, LED_BLINK_ON_TOUT, LED_TOGGLE_TOUT, TOUT_3000, 2);
  mqttInit();

  simStartInit();

  if( gprsConn() != 0 ){
    Error_Handler( STOP );
  }

  ntpInit();

  if( clkSet() == RESET ){
    // Время установлено - включаем интефейс Терминала и отправляем время
    gpioPinSetNow( &gpioPinVccTerm );
    termSendTime();
  }

  mqttStart();


  uint8_t sub = 0;

  //Test data
//  uint8_t pub_uint8 = 1;
//  uint16_t pub_uint16 = 2;
//  uint32_t pub_uint32 = 3;
//  float pub_float = 1.1;
//  double pub_double = 2.2;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
      static uint32_t minTick;

      isensProcess();
      if (SIM800.mqttServer.connect == 0) {
        MQTT_Connect();
        sub = 0;
      }
      if( SIM800.mqttServer.connect == 1 ) {
        if( sub == 0 ){
          MQTT_Sub("imei/test/#");
          sub = 1;
        }
        else if( (iSens[ISENS_1].isensFlag && (minTick < mTick))
                  || SIM800.mqttClient.toutTick < mTick ){
          MQTT_Pub( "imei/test/string", "String message" );
//          MQTT_PubUint8( "imei/test/uint8", pub_uint8 );
//          MQTT_PubUint16( "imei/test/uint16", pub_uint16 );
//          MQTT_PubUint32( "imei/test/uint32", pub_uint32 );
//          MQTT_PubFloat( "imei/test/float", pub_float );
//          MQTT_PubDouble( "imei/test/double", pub_double );
          MQTT_PubUint32( "imei/test/isens", iSens[ISENS_1].isensCount );

          iSens[ISENS_1].isensFlag = RESET;
          minTick = mTick + 1000;
          SIM800.mqttClient.toutTick = mTick + SIM800.mqttClient.keepAliveInterval / 2 * 1000;
        }

        if( SIM800.mqttReceive.newEvent ){
          unsigned char *topic = SIM800.mqttReceive.topic;
          int payload = atoi( (char*)SIM800.mqttReceive.payload );
          SIM800.mqttReceive.newEvent = 0;
          trace_printf( "mqttReceive: %s: %d\n", topic, payload );
        }
      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler( STOP );
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler( STOP );
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler( STOP );
  }

  rccClocks.SYSCLK_Frequency = HAL_RCC_GetSysClockFreq();
  rccClocks.HCLK_Frequency = HAL_RCC_GetHCLKFreq();
  rccClocks.PCLK1_Frequency = HAL_RCC_GetPCLK1Freq();
  rccClocks.PCLK2_Frequency = HAL_RCC_GetPCLK2Freq();
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
    __disable_irq();
    GPIOB->BSRR = GPIO_PIN_9;
    while (stop) {
    }
  /* USER CODE END Error_Handler_Debug */
}


int simWaitReady( int stop ){
  for( uint8_t i = 0; i < 60; i++ ){
    if( strstr(mqtt_buffer, "SMS Ready\r\n" ) != NULL ) {
      clearRxBuffer();
      mDelay(2000);
      return RESET;
    }
    mDelay(1000);
  }

  Error_Handler( stop );

  return SET;
}


/**
 * initialization SNTP.
 * @param NONE
 * @return error status, 0 - OK
 */
int clkSet( void ) {
  uint8_t errCount;

  for( errCount = 0; errCount < 4; errCount++ ){
    if( SIM800_SendCommand("AT+CCLK?\r\n", "+CCLK: \"", CMD_DELAY_30) == 0 ){
      int8_t tz;
      // Получили дату-время
      sscanf(mqtt_buffer, "+CCLK: \"%d/%d/%d,%d:%d:%d%d\"", \
                        (int*)&rtc.year, (int*)&rtc.month, (int*)&rtc.date, \
                         (int*)&rtc.hour, (int*)&rtc.min, (int*)&rtc.sec, (int *)&tz );
//      tz /= 4;
      // Переходим в Локальное время
      setRtcTime( xTm2Utime( &rtc ) + tz * 3600 );
      return RESET;
    }
    else {
      if( errCount >= 2) {
        // Никак не получается включить GPRS
        SIM800_SendCommand("AT+CFUN=1,1\r\n", "OK\r\n", CMD_DELAY_50);
        mDelay(15000);
      }
    }
  }

  return SET;
}


/**
 * initialization SIM800.
 * @param NONE
 * @return error status, 0 - OK
 */
int simStartInit(void) {
    int error = SET;
    uint8_t i;

    HAL_UART_Receive_IT(UART_SIM800, &rx_data, 1);

//    simWaitReady( NON_STOP );

    for( i = 0; i < 60; i++ ){
//      SIM800_SendCommand("AT+IPR=115200\r\n", "OK\r\n", CMD_DELAY_2);
      if( SIM800_SendCommand("AT\r\n", "OK\r\n", CMD_DELAY_2) == 0 ){
        // Есть контакт!
        break;
      }
      else {
        // Нет отклика от GSM
        mDelay(1000);
      }
    }

    if( i >= 60 ){
      Error_Handler( STOP );
    }

    SIM800_SendCommand("ATE1\r\n", "OK\r\n", CMD_DELAY_2);

    if( SIM800_SendCommand("AT+CLTS?\r\n", "+CLTS: 1\r\n", CMD_DELAY_2) != 0 ){
      SIM800_SendCommand("AT+CLTS=1;&W\r\n", "OK\r\n", CMD_DELAY_2);
      SIM800_SendCommand("AT+CFUN=1,1\r\n", "OK\r\n", CMD_DELAY_2);
      simWaitReady( NON_STOP );
    }

    return error;
}

int getClk( void ){
  int8_t tz;
  int rc = 0;

  if( SIM800_SendCommand("AT+CCLK?\r\n", "+CCLK: \"", CMD_DELAY_30) == 0 ){
    // Получили дату-время
    sscanf(mqtt_buffer, "[^:]*: \"%u/%u/%u,%u:%u:%u%d\"", \
                       (unsigned int*)&rtc.date, (unsigned int*)&rtc.month, (unsigned int*)&rtc.year, \
                       (unsigned int*)&rtc.hour, (unsigned int*)&rtc.min, (unsigned int*)&rtc.sec, (int *)&tz );
    tz /= 4;
    // Переходим в Локальное время
    setRtcTime( xTm2Utime( &rtc ) + tz * 3600 );
    rc = 1;
  }

  return rc;
}


int gprsConn( void ){
  for( uint8_t errCount = 0; errCount < 3; errCount++ ){
    uint8_t step;


    if( SIM800_SendCommand("AT+SAPBR=2,1\r\n", "+SAPBR:", CMD_DELAY_5) == 0){
      if( mqtt_buffer[10] <= '1' ){
        // Есть соединение GPRS;
        // TODO: Получение IP
        mDelay( 1000 );
        return 0;
      }
    }
    else {
      Error_Handler( NON_STOP );
    }

    step = 0;
    switch( step ){
      case 0:
        if( SIM800_SendCommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n", "OK\r\n", CMD_DELAY_5) ){
          break;
        }
        mDelay( 2000 );
        step++;
        // FALL-THROUGH
        /* no break */
      case 1:
        if( SIM800_SendCommand("AT+SAPBR=3,1,\"APN\",\"internet\"\r\n", "OK\r\n", CMD_DELAY_5) ){
          break;
        }
        mDelay( 2000 );
        step++;
        // FALL-THROUGH
        /* no break */
      case 2:{
        uint8_t i;

        for ( i = 0; i < 3; i++ ){
          if( SIM800_SendCommand("AT+SAPBR=1,1\r\n", "OK\r\n", CMD_DELAY_50) == 0){
            // Есть соединение GPRS;
            // TODO: Получение IP
            // Две вспышки оранжевого цвета с интервалом в 3 сек
            ledOff( LED_R, 0 );
            ledToggleSet( LED_R, LED_BLINK_ON_TOUT, TOUT_3000, 0, 0);
            ledToggleSet( LED_G, LED_BLINK_ON_TOUT, TOUT_3000, 0, 0);

            return 0;
          }
        }
        break;
      }
      default:
        break;
    }
  }
  return 0;
}


int ntpInit(void) {
  uint8_t ntpFlag = RESET;
  SIM800.mqttServer.connect = 0;

  timeInit();

  if( SIM800_SendCommand("AT+CNTPCID=1\r\n", "OK\r\n", CMD_DELAY_5) ){
    return ntpFlag;
  }
  while( ntpFlag == RESET ){

    if( SIM800_SendCommand("AT+CNTP=\""NTP_SERVER"\",12\r\n", "OK\r\n", CMD_DELAY_5) == 0){;
      if( SIM800_SendCommand("AT+CNTP\r\n", "+CNTP: 1\r\n", CMD_DELAY_50 * 10) == 0 ){
        ntpFlag = SET;
      }
      else {
        Error_Handler( 2000 );
      }
    }
  }

  return ntpFlag;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
