/*
 * gsm.c
 *
 *  Created on: 3 окт. 2021 г.
 *      Author: jet
 */
#include <pos.h>
#include <stdio.h>
#include <string.h>

#include "MQTTSim800.h"
#include "usart_arch.h"
#include "gpio_arch.h"
#include "my_mqtt.h"
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
void pppInit( void );

void simUartBaud( uint32_t baudrate );
void simUartHwFlow( void );

int simWaitReady( void );
int gprsConnStart( void );
int simStartInit(void);
int gprsConnBreak( void );
int gprsConn( void );
int ntpInit(void);
void mqttServPrep( void * sim );

// ----------------------------------------------------------------------------

// ----------------  GSM PROCCESS FUNCTIONS -----------------------------------
int gsmCmdMode( void ){
  mDelay(1000);
  simHnd.txh->data = (uint8_t *)"+++\r\n";
  if( uartTransmit(simHnd.txh, 5, 1000) == 0 ){
    trace_puts( "uart err" );
  }

  mDelay(1000);

  return 0;
}

/**
 * Send AT command to SIM800 over UART.
 * @param command the command to be used the send AT command
 * @param reply to be used to set the correct answer to the command
 * @param delay to be used to the set pause to the reply
 * @return error, 0 is OK
 */
int gsmSendCommand(char *command, char *reply, uint16_t delay, void (*simreplycb)( sUartRxHandle *) ){
  uint32_t tmptick;
  tmptick = mTick + delay;
  uint8_t rc = 1;

  simHnd.rxh->replyCb = simreplycb;

  simHnd.txh->data = (uint8_t*)command;
  simHnd.rxh->reply = reply;
  simHnd.rxh->replyFlag = RESET;
  trace_write( command, strlen(command) );
  if( uartTransmit(simHnd.txh, (uint16_t)strlen(command), 1000) == 0 ){
    trace_puts( "uart err" );
  }

  if( reply == NULL ){
    mDelay(delay);
    return 0;
  }

  while( tmptick >= mTick ) {
    if( simHnd.rxh->replyFlag ) {
      rc = 0;
      break;
    }
  }
  return rc;
}


int _disconnect(uint8_t rfOff){
  int res;
  if ( (res = gsmSendCommand("AT\r\n", "OK\r\n", CMD_DELAY_2, NULL )) != 0 ) {
    if (rfOff) {
      res = gsmSendCommand("AT+CFUN=4\r\n", "OK\r\n", CMD_DELAY_50, NULL); // disable RF function
    }
    return res;
  }

  #if GSM_DEBUG
  printf("GSM: ONLINE, DISCONNECTING...\r\n");
  #endif
  for( int n = 0; (res = gsmSendCommand("ATH\r\n", "OK\r\n", CMD_DELAY_30, NULL ) != 0); n++ ) {
    if (n > 9) {
      n = 0;
      gsmCmdMode();
    }
    mDelay(100);
  }
  SIM800.mqttServer.gprsconn = RESET;
  mDelay(100);
  if (rfOff) {
    res = gsmSendCommand("AT+CFUN=4\r\n", "OK\r\n", CMD_DELAY_50, NULL); // disable RF function
  }
  #if GSM_DEBUG
  printf("GSM: DISCONNECTED.\r\n");
  #endif

  return res;
}
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------

/**
 * initialization SIM800.
 * @param NONE
 * @return error status, 0 - OK
 */
int simStartInit(void) {
    int error = RESET;
    eBaudrate baud = BAUD_NUM;

//    HAL_UART_Receive_IT(UART_SIM800, &rx_data, 1);

//    simWaitReady( NON_STOP );
    for( eBaudrate i = BAUD_9600; baud == BAUD_NUM; ){
      for( uint8_t j = 0; j < 3; j++ ){
        if( gsmSendCommand("AT\r\n", "OK\r\n", CMD_DELAY_2, NULL ) == 0 ){
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

    if( gsmSendCommand("AT+IFC=2,2\r\n", "OK\r\n", CMD_DELAY_2, NULL ) ){
      return -1;
    }

    simUartHwFlow();
//    if( baud != BAUD_460800 ){
//      if( gsmSendCommand("AT+IPR=460800\r\n", "OK\r\n", CMD_DELAY_2) == 0){
//        simUartBaud(460800);
//      }
//    }

    if( gsmSendCommand("AT\r\n", "OK\r\n", CMD_DELAY_2, NULL ) != 0 ){
      Error_Handler( STOP );
    }

    error = gsmSendCommand("ATZ\r\n", "OK\r\n", CMD_DELAY_2, NULL );
    error = gsmSendCommand("ATE1\r\n", "OK\r\n", CMD_DELAY_2, NULL );

    return error;
}


void gpsPppInit( void ){
  pppInit();
}


int gprsConnStart( void ){

//  _disconnect( RESET );

//  if( gsmSendCommand("AT+CFUN=1,1\r\n", "OK\r\n", CMD_DELAY_5, NULL ) ){
//    return -1;
//  }

  return 0;
}


int gprsConn( void ){
//  if( gsmSendCommand("AT+CSQ=1,1\r\n", "OK\r\n", CMD_DELAY_5, csqSimReply ) ){
//    return -1;
//  }
  if( gsmSendCommand("AT+CGDCONT=1,\"IP\",\"internet\"\r\n", "OK\r\n", CMD_DELAY_5, NULL ) ){
    return -1;
  }
  if( gsmSendCommand("ATD*99#\r\n", "CONNECT\r\n", CMD_DELAY_30, NULL ) ){
    return -1;
  }

  return 0;
}


int gprsConnBreak( void ){
  if( mqtt_client_is_connected( &SIM800.mqttClient.client ) ){
    mqtt_disconnect( &SIM800.mqttClient.client );
    return -1;
  }
  else {
    if( _disconnect( SET ) == ERR_OK ){
      return gsmSendCommand("ATE1\r\n", "OK\r\n", CMD_DELAY_2, NULL );
    }
    else {
      return -1;
    }
  }
}


//int gprsConn( void ){
//  gsmSendCommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n", "OK\r\n", CMD_DELAY_5, NULL );
////  mDelay( 2000 );
//  gsmSendCommand("AT+SAPBR=3,1,\"APN\",\"internet\"\r\n", "OK\r\n", CMD_DELAY_5, NULL);
////  mDelay( 2000 );
//
//  if( gsmSendCommand("AT+SAPBR=1,1\r\n", "OK\r\n", CMD_DELAY_50, NULL) == 0){
//    // Есть соединение GPRS;
//    // TODO: Получение IP
//  }
//  return 0;
//}


int ntpInit(void) {
  if( ntpFlag == RESET ){
    timeInit();

    if( gsmSendCommand("AT+CNTPCID=1\r\n", "OK\r\n", CMD_DELAY_5, NULL) ){
      return ntpFlag;
    }
    while( ntpFlag == RESET ){

      if( gsmSendCommand("AT+CNTP=\""NTP_SERVER"\",12\r\n", "OK\r\n", CMD_DELAY_5, NULL) == 0){;
        if( gsmSendCommand("AT+CNTP\r\n", "+CNTP: 1\r\n", CMD_DELAY_50 * 10, NULL) == 0 ){
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
        simStartInit();
        tmpTick = mTick + (TOUT_1000 * 30);
        gsmRunPhase = PHASE_ON;
        tmpTick = 0;
        break;
      case PHASE_ON:
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


// Состояние "GSM START_INIT": Запуск PPP
void gsmStartInitFunc( void ){
  if( gsmRun ){
    switch( gsmRunPhase ){
      case PHASE_NON:
        if( tmpTick < mTick ){
          tmpTick = mTick + 3000;
          if( gprsConnStart() == 0 ){
            gsmRunPhase = PHASE_ON;
          }
        }
        break;
      case PHASE_ON:
        if( tmpTick >= mTick ){
          break;
        }

        simHnd.rxh->replyBuf = mqtt_buffer;
        *mqtt_buffer = '\0';

        if( gsmSendCommand("AT+CREG?\r\n", "+CREG:", CMD_DELAY_10, saveSimReply )
            || (mqtt_buffer[9] != '1') ){
          gsmRunPhase = PHASE_NON;
          break;
        }

//        if( simWaitReady() != RESET ) {
//          break;
//        }

        if( gprsConn() != RESET ){
          tmpTick = mTick + 3000;
          gsmRunPhase = PHASE_NON;
        }
        else {
          SIM800.mqttServer.gprsconn = SET;
          gsmRunPhase = PHASE_ON_OK;
        }
        break;
      case PHASE_ON_OK:
        // Есть соединение GPRS;
        // TODO: Получить IP-адрес
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
      if( gprsConnBreak() == ERR_OK){
        gsmRunPhase = PHASE_NON;
        gsmState--;
      }
    }
    else {
      // Пробуем переподключить
      if( gsmRunPhase == PHASE_NON) {
        tmpTick = 0;
        if( gprsConnBreak() == ERR_OK ) {
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
        ppp_connect( ppp, 0 );
        gsmRunPhase = PHASE_ON;
        break;
      case PHASE_ON:
        if( SIM800.mqttServer.pppconn == 0 ){
          break;
        }
        sntp_setservername( 0, NTP_SERVER );
        sntp_init();
        gsmRunPhase = PHASE_ON_OK;
        break;
      case PHASE_ON_OK:
        if( rtcSetFlag ){
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
    switch ( gsmRunPhase ){
      case PHASE_NON:
        mqttServPrep( &SIM800 );
        gsmRunPhase = PHASE_ON;
        break;
      case PHASE_ON:
        if( SIM800.mqttServer.addr.addr != 0 ){
          mqtt_client_connect( &SIM800.mqttClient.client, &SIM800.mqttServer.addr, SIM800.mqttServer.port, \
                               mqttConnectCb, &SIM800, &SIM800.mqttClient.clientInfo );
        }
        break;
      default:
        break;
    }
  }
  else {
    // Продолжаем выключать
    if( gsmFinal ){
      // Продолжаем выключать
      mqtt_disconnect( &SIM800.mqttClient.client );
      gsmState--;
    }
    else if ( SIM800.mqttServer.pppconn == 0 ){
      // Требуется перезапуск PPP
      gsmState--;
    }
    else {
      // Требуется переподключение к MQTT-серверу
      gsmRun = SET;
    }
    gsmRunPhase = PHASE_NON;
  }
}


// Состояние "GSM PWR ON": Установка сохраненной конфигурации
void gsmMqttConnFunc( void ){
  if( gsmRun ){
    if( mqttSubFlag ) {
      mqttSubStart("imei/fw/#");
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
    if( SIM800.mqttClient.client.conn_state != MQTT_CONNECTED ){
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
      gsmOffFunc();         // Включение питание
      break;
    case GSM_SIM_ON:
      gsmSimOnFunc();       // Настройка интерфейса и ожидение готовности SIM
      break;
    case GSM_IFACE_INIT:
      gsmStartInitFunc();
      break;
    case GSM_GPRS_CONN:
      gsmGprsConnFunc();
      break;
    case GSM_NTP_INIT:
      gsmNtpInitFunc();
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



