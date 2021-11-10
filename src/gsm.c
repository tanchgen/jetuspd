/*
 * gsm.c
 *
 *  Created on: 3 окт. 2021 г.
 *      Author: jet
 */
#include <gsm.h>
#include <gsm.h>
#include <stdio.h>
#include <string.h>

#include "my_ntp.h"
#include "MQTTSim800.h"
#include "usart_arch.h"
#include "gpio_arch.h"
#include "gsm.h"

extern const uint32_t baudrate[BAUD_NUM];
extern const sUartHnd simHnd;
extern struct timer_list mqttPubTimer;

static uint32_t tmpTick;

SIM800_t SIM800;
eGsmState gsmState = GSM_OFF;

FlagStatus gsmRun = SET;
eGsmRunPhase gsmRunPhase = PHASE_OFF_OK;

FlagStatus gsmFinal = RESET;

FlagStatus ntpFlag;

uint8_t reconnCount = 0;
uint16_t gprsConnTout[] = {
  2000,
  3000,
  3000,
  3000
};

// -------------- Function prototype ------------------------------------------
void simUartBaud( uint32_t baudrate );
void simUartHwFlow( void );

int simWaitReady( void );
int gprsConnTest( void );
int simStartInit(void);
int gprsConnTest( void );
int gprsConnBreak( void );
int gprsConn( void );
int ntpInit(void);
int TCP_Connect(void);
int clkSet( void );
// ----------------------------------------------------------------------------
// ----------------  GSM PROCCESS FUNCTIONS -----------------------------------
// Включение питания SIM800
void gsmOffFunc( void ){
  if( gsmRun ){
    switch( gsmRunPhase ){
      case PHASE_NON:
        // On SIM800 power if use
        gpioPinResetNow( &gpioPinSimPwr );
        gpioPinSetNow( &gpioPinPwrKey );
        gsmRunPhase = PHASE_ON;
        // TODO: MCU засыпает на 2с, пока sim800 Сбрасывается.
        mDelay(2000);
        break;
      case PHASE_ON:
        gpioPinResetNow( &gpioPinPwrKey );
        gsmRunPhase = PHASE_ON_OK;
        // TODO: MCU засыпает на 10с, пока sim800 включается.
        mDelay(10000);
        break;
      case PHASE_ON_OK:
        // Две вспышки красного цвета с интервалом в 3 сек
        ledToggleSet( LED_R, LED_BLINK_ON_TOUT, LED_TOGGLE_TOUT, TOUT_3000, 2);
        gsmState++;
        gsmRunPhase = PHASE_NON;
        break;
      default:
        break;
    }
  }
  else {
    // Выключаем питание GSM
    switch( gsmRunPhase ){
      case PHASE_NON:
        gpioPinResetNow( &gpioPinPwrKey );
        gpioPinSetNow( &gpioPinSimPwr );
        gsmRunPhase = PHASE_OFF;
        break;
      case PHASE_OFF:
        // TODO: Выставляем будильник, если надо. Усыпляем контроллер
        if( mcuReset ){
          NVIC_SystemReset();
        }
        else {
          gsmRunPhase = PHASE_OFF_OK;
          tmpTick = mTick + 10000;
        }
        break;
      case PHASE_OFF_OK:
        if( tmpTick < mTick ){
          gsmRun = SET;
        }
        break;
      default:
        break;
    }
  }
}


// Состояние "GSM PWR ON": Установка сохраненной конфигурации
void gsmSimOnFunc( void ){
  if( gsmRun ){
    switch( gsmRunPhase ){
      case PHASE_NON:
//        mqttInit();
        ntpFlag = RESET;
        gsmState++;
        break;
      default:
        break;
    }
  }
  else {
    // Выключаем питание GSM
    gsmState--;
  }
}


// Состояние "GSM PWR ON": Установка сохраненной конфигурации
void gsmInitFunc( void ){
  if( gsmRun ){
    switch( gsmRunPhase ){
      case PHASE_NON:
        simStartInit();
        tmpTick = mTick + (TOUT_1000 * 30);
        gsmRunPhase = PHASE_ON;
        tmpTick = 0;
        break;
      case PHASE_ON:
        if( (simWaitReady() == RESET) || (tmpTick < mTick) ) {
//          // TODO: Усыпить на 2с
//          mDelay(2000);
          gsmRunPhase = PHASE_ON_OK;
        }
        else if( tmpTick == 0 ){
          tmpTick = mTick + 7000;
        }
        else if( tmpTick < mTick ){
          gsmRunPhase = PHASE_ON_OK;
        }
        break;
      case PHASE_ON_OK:
        gsmRunPhase = PHASE_NON;
        gsmState++;
        break;
      default:
        break;
    }
  }
  else {
    // Выключаем питание GSM
    gsmState--;
  }
}


// Состояние "GSM PWR ON": Установка сохраненной конфигурации
void gsmStartInitFunc( void ){
  if( gsmRun ){
    switch( gsmRunPhase ){
      case PHASE_NON:
        switch( gprsConnTest() ){
          case '0':   // Bearer is connecting
          case '2':   // Bearer is closing
            break;
          case '1':   // Bearer is connect
            gsmRunPhase = PHASE_ON_OK;
            break;
          case '3':   // Bearer is closed
            gsmRunPhase = PHASE_ON;
            break;
          default:
//            Error_Handler( NON_STOP );
            mDelay(10);
            break;
        }
        break;
      case PHASE_ON:
        if( gprsConn() != RESET ){
          gprsConnBreak();
        }
        gsmRunPhase = PHASE_NON;
        break;
      case PHASE_ON_OK:
        // Есть соединение GPRS;
        // TODO: Получить IP-адрес

        SIM800.mqttServer.tcpconn = 0;

        // Две вспышки оранжевого цвета с интервалом в 3 сек
        ledOff( LED_R, 0 );
        ledToggleSet( LED_R, LED_BLINK_ON_TOUT, TOUT_3000, 0, 0);
        ledToggleSet( LED_G, LED_BLINK_ON_TOUT, TOUT_3000, 0, 0);
        gsmRunPhase = PHASE_NON;
        gsmState++;
        break;
      default:
        break;
    }
  }
  else {
    // Выключаем питание GSM
    if( gsmFinal ){
      // Продолжаем выключать
      gprsConnBreak();
      gsmState--;
    }
    else {
      // Пробуем переподключить
      if( gsmRunPhase == PHASE_NON) {
        tmpTick = 0;
        if( gprsConnBreak() == 0) {
          gsmRun = SET;
          gsmRunPhase = PHASE_NON;
        }
        else {
          tmpTick = mTick + TOUT_1000;
          gsmRunPhase = PHASE_OFF;
        }
      }
      else if (tmpTick < mTick ){
        gsmRunPhase = PHASE_NON;
        gsmState--;
      }
    }

  }
}


// Состояние "GSM PWR ON": Установка сохраненной конфигурации
void gsmGprsConnFunc( void ){
  if( gsmRun ){
    switch( gsmRunPhase ){
      case PHASE_NON:
        ntpInit();
        gsmRunPhase = PHASE_ON;
        break;
      case PHASE_ON:
        if( clkSet() == RESET ){
          // Время установлено - включаем интефейс Терминала и отправляем время
#if TERM_UART_ENABLE
          gpioPinSetNow( &gpioPinTermOn );
          termSendTime();
#endif //TERM_UART_ENABLE
          gsmRunPhase = PHASE_NON;
          gsmState++;
        }
        break;
      default:
        break;
    }
  }
  else {
    // Выключаем питание GSM
    gsmState--;
  }
}


// Состояние "GSM PWR ON": Установка сохраненной конфигурации
void gsmNtpInitFunc( void ){
  if( gsmRun ){
    if( mqttStart() == RESET ){
      gsmRunPhase = PHASE_NON;
      gsmState++;
    }
  }
  else {
    // Продолжаем выключать
    MQTT_Deinit();
    gsmState--;
  }
}


// Состояние "GSM PWR ON": Установка сохраненной конфигурации
void gsmMqttStartFunc( void ){
  if( gsmRun ){
    switch( gsmRunPhase ){
      case PHASE_NON:
        if( SIM800.mqttServer.tcpconn == 0 ){
          trace_puts("tcp conn");
          TCP_Connect();
        }
        else if( SIM800.mqttServer.mqttconn == 0 ){
          MQTT_Connect();
          gsmRunPhase = PHASE_ON;
          tmpTick = mTick + 10000;
        }
        else {
          gsmRunPhase = PHASE_ON_OK;
        }
        break;
      case PHASE_ON:
        if( SIM800.mqttServer.mqttconn == 1 ){
          gsmRunPhase = PHASE_ON_OK;
        }
        else if(tmpTick < mTick ){
          // Не дождались соединения MQTT
          trace_puts( "mqtt_deinit" );
//          MQTT_Deinit();
          mDelay(TOUT_1000 * 15);
          gsmRunPhase = PHASE_NON;
        }
        break;
      case PHASE_ON_OK:
        gsmState++;
        gsmRunPhase = PHASE_NON;
        break;
      default:
        break;
    }
  }
  else {
    // Выключаем питание GSM
    gsmState--;
    gsmRunPhase = PHASE_NON;
  }
}


// Состояние "GSM PWR ON": Установка сохраненной конфигурации
void gsmMqttConnFunc( void ){
  if( gsmRun ){
    if( mqttSubFlag ) {
      trace_puts( "mqtt sub" );
      MQTT_Sub("imei/#", 2);
      timerMod( &mqttPubTimer, 0 );
      mqttSubFlag = RESET;
      gsmState++;
      gsmRunPhase = PHASE_NON;
    }
  }
  else {
    // Выключаем питание GSM
    gsmState--;
  }
}


// Состояние "GSM PWR ON": Установка сохраненной конфигурации
void gsmServConnFunc( void ){
  if( gsmRun ){
    gsmState++;
    gsmRunPhase = PHASE_NON;
  }
  else {
    // Выключаем питание GSM
    gsmState--;
  }
}


// Состояние "GSM PWR ON": Установка сохраненной конфигурации
void gsmCfgOnFunc( void ){
  if( gsmRun ){
    gsmState++;
    gsmRunPhase = PHASE_NON;
  }
  else {
    // Выключаем питание GSM
    gsmState--;
  }
}


// Состояние "GSM PWR ON": Установка сохраненной конфигурации
void gsmWorkFunc( void ){
  if( gsmRun ){
    gsmRunPhase = PHASE_NON;
    if( SIM800.mqttServer.tcpconn == 0 ){
      // Закрыто соединение с MQTT-сервером - откатываемся до стадии GSM_MQTT_START
      gsmRun = RESET;
    }
  }
  else {
    // Выключаем питание GSM
    gsmState--;
  }
}


// ----------------------------------------------------------------------------
// Машина состояния break;
void gsmProcess( void ){
  switch( gsmState ){
    case GSM_OFF:
      gsmOffFunc();
      break;
    case GSM_SIM_ON:
      gsmSimOnFunc();
      break;
    case GSM_INIT:
      gsmInitFunc();
      break;
    case GSM_START_INIT:
      gsmStartInitFunc();
      break;
    case GSM_GPRS_CONN:
      gsmGprsConnFunc();
      break;
    case GSM_NTP_INIT:
      gsmNtpInitFunc();
      break;
    case GSM_MQTT_START:
      gsmMqttStartFunc();
      break;
    case GSM_MQTT_CONN:
      gsmMqttConnFunc();
      break;
    case GSM_SERV_CONN:
      gsmServConnFunc();
      break;
    case GSM_CFG_ON:
      gsmCfgOnFunc();
      break;
    case GSM_WORK:
      gsmWorkFunc();
      break;
    default:
      break;
  }


}


/**
 * initialization SNTP.
 * @param NONE
 * @return error status, 0 - OK
 */
int clkSet( void ) {
  uint8_t errCount;

  for( errCount = 0; errCount < 4; errCount++ ){
    simHnd.rxh->replyBuf = mqtt_buffer;
    *mqtt_buffer = '\0';
    if( SIM800_SendCommand("AT+CCLK?\r\n", "+CCLK: \"", CMD_DELAY_30, saveSimReply ) == 0){
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
        SIM800_SendCommand("AT+CFUN=1,1\r\n", "OK\r\n", CMD_DELAY_50, NULL );
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
    eBaudrate baud = BAUD_NUM;

//    HAL_UART_Receive_IT(UART_SIM800, &rx_data, 1);

//    simWaitReady( NON_STOP );
    for( eBaudrate i = BAUD_9600; baud == BAUD_NUM; ){
      for( uint8_t j = 0; j < 3; j++ ){
        if( SIM800_SendCommand("AT\r\n", "OK\r\n", CMD_DELAY_2, NULL ) == 0 ){
          // Есть контакт!
          baud = i;
          break;
        }
      }
      if( i != baud ){
        // Нет отклика от GSM
        mDelay(1000);
        // Увиличиваем скорость порта
        i++;
        if(i < BAUD_NUM){
          simUartBaud( baudrate[i] );
        }
        else {
          break;
        }
      }
    }

    if( baud == BAUD_NUM ){
      Error_Handler( STOP );
    }

    if( SIM800_SendCommand("AT+IFC=2,2\r\n", "OK\r\n", CMD_DELAY_2, NULL ) == 0){
      simUartHwFlow();
    }
//    if( baud != BAUD_460800 ){
//      if( SIM800_SendCommand("AT+IPR=460800\r\n", "OK\r\n", CMD_DELAY_2) == 0){
//        simUartBaud(460800);
//      }
//    }

    if( SIM800_SendCommand("AT\r\n", "OK\r\n", CMD_DELAY_2, NULL ) != 0 ){
      Error_Handler( STOP );
    }

    SIM800_SendCommand("ATE1\r\n", "OK\r\n", CMD_DELAY_2, NULL );

    return error;
}

/*
int getClk( void ){
  int8_t tz;
  int rc = 0;

  simHnd.rxh->replyBuf = mqtt_buffer;
  if( SIM800_SendCommand("AT+CCLK?\r\n", "+CCLK: \"", CMD_DELAY_30) == 0, saveSimReply ){
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
*/


int gprsConnTest( void ){
  int rc = -1;
  simHnd.rxh->replyBuf = mqtt_buffer;
  *mqtt_buffer = '\0';
    if( SIM800_SendCommand("AT+SAPBR=2,1\r\n", "+SAPBR:", CMD_DELAY_5, saveSimReply ) == 0){
      rc = mqtt_buffer[10];
    }
    else {
      Error_Handler( NON_STOP );
    }

  return rc;
}


int gprsConnBreak( void ){
  if( SIM800.mqttServer.tcpconn ){
    return MQTT_Deinit();
  }
  else {
    return SIM800_SendCommand("ATE1\r\n", "OK\r\n", CMD_DELAY_2, NULL );
  }
}


int gprsConn( void ){
  SIM800_SendCommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n", "OK\r\n", CMD_DELAY_5, NULL );
//  mDelay( 2000 );
  SIM800_SendCommand("AT+SAPBR=3,1,\"APN\",\"internet\"\r\n", "OK\r\n", CMD_DELAY_5, NULL);
//  mDelay( 2000 );

  if( SIM800_SendCommand("AT+SAPBR=1,1\r\n", "OK\r\n", CMD_DELAY_50, NULL) == 0){
    // Есть соединение GPRS;
    // TODO: Получение IP
  }
  return 0;
}


int ntpInit(void) {
  if( ntpFlag == RESET ){
    timeInit();

    if( SIM800_SendCommand("AT+CNTPCID=1\r\n", "OK\r\n", CMD_DELAY_5, NULL) ){
      return ntpFlag;
    }
    while( ntpFlag == RESET ){

      if( SIM800_SendCommand("AT+CNTP=\""NTP_SERVER"\",12\r\n", "OK\r\n", CMD_DELAY_5, NULL) == 0){;
        if( SIM800_SendCommand("AT+CNTP\r\n", "+CNTP: 1\r\n", CMD_DELAY_50 * 10, NULL) == 0 ){
          ntpFlag = SET;
        }
        else {
          Error_Handler( NON_STOP );
        }
      }
    }
  }

  return ntpFlag;
}


int simWaitReady( void ){
  if( SIM800.ready ){
    clearRxBuffer( (char *)(simHnd.rxh->rxFrame), &(simHnd.rxh->frame_offset) );
    return RESET;
  }

  return SET;
}


