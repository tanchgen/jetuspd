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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"
#include "isens.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
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
void SystemClock_Config(void);
int ntpInit(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_ADC_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  if (HAL_ADC_Start_DMA(&hadc,
                         (uint32_t *)aADCxConvertedValues,
                         ADCCONVERTEDVALUES_BUFFER_SIZE
                        ) != HAL_OK)
   {
     /* Start Error */
     Error_Handler();
   }

    // On SIM800 power if use
  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  	HAL_Delay(15000);
//    HAL_GPIO_WritePin(SIM_PWR_GPIO_Port, SIM_PWR_Pin, GPIO_PIN_SET);
//    HAL_Delay(2000);
//    HAL_GPIO_WritePin(SIM_PWR_GPIO_Port, SIM_PWR_Pin, GPIO_PIN_RESET);
//    HAL_Delay(10000);

    // MQQT settings
    SIM800.sim.apn = "internet";
    SIM800.sim.apn_user = "";
    SIM800.sim.apn_pass = "";
    SIM800.mqttServer.host = "test.mosquitto.org";
    SIM800.mqttServer.port = 1883;
    SIM800.mqttClient.username = "";
    SIM800.mqttClient.pass = "";
    SIM800.mqttClient.clientID = "";
    SIM800.mqttClient.keepAliveInterval = 120;


    MQTT_Init();

    ntpInit();

    for( uint8_t i = 0; i < ISENS_NUM; i++ ){
      isensInit( &(iSens[i]) );
    }

    uint8_t sub = 0;

    //Test data
    uint8_t pub_uint8 = 1;
    uint16_t pub_uint16 = 2;
    uint32_t pub_uint32 = 3;
    float pub_float = 1.1;
    double pub_double = 2.2;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
      if (SIM800.mqttServer.connect == 0) {
        MQTT_Connect();
        sub = 0;
      }
      if( SIM800.mqttServer.connect == 1 ) {
        if( sub == 0 ){
          MQTT_Sub("imei/test");
          sub = 1;
        }

        MQTT_Pub( "imei/test/string", "string" );
        MQTT_PubUint8( "imei/test/uint8", pub_uint8 );
        MQTT_PubUint16( "imei/test/uint16", pub_uint16 );
        MQTT_PubUint32( "imei/test/uint32", pub_uint32 );
        MQTT_PubFloat( "imei/test/float", pub_float );
        MQTT_PubDouble( "imei/test/double", pub_double );
        MQTT_PubUint32( "imei/test/isens", iSens[ISENS_SB1].isensCount );

        if( SIM800.mqttReceive.newEvent ){
          unsigned char *topic = SIM800.mqttReceive.topic;
          int payload = atoi( (char*)SIM800.mqttReceive.payload );
          SIM800.mqttReceive.newEvent = 0;
          trace_printf( "mqttReceive: %s: %d\n", topic, payload );
        }
      }

      HAL_Delay( 1000 );
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
    Error_Handler();
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
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    GPIOB->BSRR = GPIO_PIN_9;
    while (1) {
    }
  /* USER CODE END Error_Handler_Debug */
}

/**
 * initialization SNTP.
 * @param NONE
 * @return error status, 0 - OK
 */
int ntpInit(void) {
  uint8_t errCount = 0;
  uint8_t ntpFlag = 0;
  int8_t tz;
  SIM800.mqttServer.connect = 0;
  int error = 0;

  timeInit();

  while( ntpFlag == RESET ){
    error = SIM800_SendCommand("AT+CNTPCID=1\r\n", "OK\r\n", CMD_DELAY_5);
    error += SIM800_SendCommand("AT+CNTP=\"37.230.228.40\",32\r\n", "OK\r\n", CMD_DELAY_5);
    error += SIM800_SendCommand("AT+CNTP\r\n", "+CNTP: 1\r\n", CMD_DELAY_50);
    error += SIM800_SendCommand("AT+CCLK?\r\n", "+CCLK: \"", CMD_DELAY_30);
    error = 0;
    if( error == 0 ){
      // Получили дату-время
      sscanf(mqtt_buffer, "[^:]*: \"%u/%u/%u,%u:%u:%u%d\"", \
                           (unsigned int*)&rtc.date, (unsigned int*)&rtc.month, (unsigned int*)&rtc.year, \
                           (unsigned int*)&rtc.hour, (unsigned int*)&rtc.min, (unsigned int*)&rtc.sec, (int *)&tz );
      tz /= 4;
      // Переходим в Локальное время
      setRtcTime( xTm2Utime( &rtc ) + tz * 3600 );
      ntpFlag = SET;
    }
    else {
      if( ++errCount >= 3 ){
        while(1) {
          error = MQTT_Deinit();
          if( error == 0 ){
            error = MQTT_Init();
          }
          if( error ){
            if( ++errCount == 4 ){
              // Никак не получается включить GPRS
              errCount = 0;
              // On SIM800 power if use
              HAL_GPIO_WritePin(SIM_PWR_GPIO_Port, SIM_PWR_Pin, GPIO_PIN_RESET);
              HAL_Delay(3000);
              HAL_GPIO_WritePin(SIM_PWR_GPIO_Port, SIM_PWR_Pin, GPIO_PIN_SET);
              HAL_Delay(10000);
            }
          }
          else {
            break;
          }
        }
        errCount = 0;
      }
    }
  }
  return error;
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
