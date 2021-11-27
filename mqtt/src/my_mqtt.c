/*
 * mqtt.c
 *
 *  Created on: 21 окт. 2021 г.
 *      Author: jet
 */
#include <my_mqtt.h>
#include <string.h>

#include "gpio_arch.h"
#include "uart.h"
#include "MQTTSim800.h"
#include "adc.h"
#include "fw.h"
#include "gsm.h"
#include "isens.h"
#include "logger.h"

extern struct timer_list mqttPubTimer;
extern uint16_t logRdBufFill;
extern const sUartHnd simHnd;

const char * topicStr[TOPIC_NUM] = {
  "r/device",     		//  TOPIC_DEVID,
  "d/imei",        		//  TOPIC_DEV_IMEI,
  "info",         		  //  TOPIC_INFO,
  "temp",       		    //  TOPIC_TEMP,
  "volt",       		    //  TOPIC_VOLT,
  "cmdi",           		//  TOPIC_CMD_I,
  "cmdo",           		//  TOPIC_CMD_O,
  "cfi",       	    	//  TOPIC_CFG_I,
  "cfo",       		    //  TOPIC_CFG_O,
  "alrm",       		    //  TOPIC_ALRM,
  "log",       		    //  TOPIC_LOG,
  "i/%d",       		    //  TOPIC_ISENS,
  "i/%d/arx",      		//  TOPIC_ISENS_ARX,
  "i/%d/state",     		//  TOPIC_ISENS_STATE,
  "i/%d/adc",       		//  TOPIC_ISENS_ADC,
  "i/%d/temp",     		//  TOPIC_ISENS_TEMP,
  "o/%d",       		//  TOPIC_OUT,
  "o/%d/state",       		//  TOPIC_OUT_STATE,
  "fw",       		//  TOPIC_FW,
  "fw/man",       		//  TOPIC_FW_MAN,
  "fw/bin",       		//  TOPIC_FW_BIN,
  "rs",       		//  TOPIC_RS,
  "rs/tx",       		//  TOPIC_RS_TX,
  "rs/rx",       		//  TOPIC_RS_RX,
  "gsm",       		//  TOPIC_GSM,
  //  TOPIC_NUM
};


void mqttServPrep( void * arg );

static void mqtt_dns_retry(void* arg){
  (void)arg;

  /* set up a timer to send a retry and increase the retry delay */
  sys_timeout(3000, mqttServPrep, (void*)&SIM800);
}

void mqtt_dns_found(const char *name, const ip_addr_t *ipaddr, void *arg){
  (void)name;
  (void)arg;

  if (ipaddr == NULL) {
    /* DNS resolving failed -> try another server */
    mqtt_dns_retry(NULL);
  }
}

void mqttServPrep( void * arg ){
  SIM800_t * sim = arg;

  if( sim->mqttServer.addr.addr == 0 ){
    dns_gethostbyname( sim->mqttServer.host, &(sim->mqttServer.addr), mqtt_dns_found, NULL);
  }
  sim->mqttClient.clientInfo.will_topic = "imei";  // TODO: Подставить настоящий IMEI
  sim->mqttClient.clientInfo.will_qos = 2;
}


void mqttSubCb(void *arg, err_t err){
  (void)arg;

  if( err == ERR_OK ){
    trace_puts( "Sub OK" );
  }
  else if( err == ERR_TIMEOUT ){
    sys_timeout( 1000, mqttSubStart, &SIM800 );
  }
  else if( err == ERR_ABRT ){
    trace_puts( "Sub denied" );
  }
}


void mqttSubStart( void * arg){
  SIM800_t * sim = arg;

  trace_puts( "mqtt sub" );
  // TODO: Оформляем подписки
  mqtt_sub_unsub( &(sim->mqttClient.client), "imei/fw/#", 2, mqttSubCb, &SIM800, SET);
}


void mqttPubTout( uintptr_t arg ){
  (void)arg;

  if( mqtt_client_is_connected( &SIM800.mqttClient.client ) ) {
    mqttPubFlag = SET;
  }
}


void mqttPubTopicCb( void *arg, const char *topic, u32_t tot_len ){
  (void)tot_len;
  SIM800_t  *sim = arg;
  eTopicId tp;
  char * tpc;

  // Сохраняем только значимую часть топика, без "<IMEI>/"
  for( tp = TOPIC_DEVID; tp < TOPIC_NUM; tp++ ){
    if( (tpc = strstr( topic, topicStr[tp] )) != NULL ){
      break;
    }
  }

  sim->mqttClient.topicId = tp;
  sim->mqttClient.payLen = 0;

}

// Callback incomming publish data
void mqttPubDataCb( void * arg, const uint8_t * data, uint16_t len, uint8_t last ){
  SIM800_t * sim = arg;

  if( last == MQTT_DATA_FLAG_LAST ){
    // Payload получен в полном объеме
    if( sim->mqttClient.topicId == TOPIC_NUM ){
      // Топик не наш - Не сохраняем payload
      mqttMsgReset( simHnd.rxh, &SIM800 );
    }
    else {
      ledOn( LED_G, 100 );
      // Получено сообщение целиком
      // TODO: Обработка полученых данных - сделать не в ПРЕРЫВАНИИ
      simHnd.rxh->rxProcFlag = SET;
    }
  }
  else {
    memcpy( &(sim->mqttClient.payload[sim->mqttClient.payLen]), data, len );
    sim->mqttClient.payLen += len;
    if( sim->mqttClient.payLen >= 512 ){
      if( sim->mqttClient.topicId == TOPIC_FW_BIN ){
        // Прием прошивки - запись во флеш
        sim->mqttClient.payOffset = 0;
        simHnd.rxh->rxProcFlag = SET;
      }
      else {
        // Топик не наш - Не сохраняем payload
        mqttMsgReset( simHnd.rxh, &SIM800 );
      }
    }
  }
}


void mqttPubProc( sUartRxHandle * handle ){
  if( handle->rxProcFlag == RESET ){
    return;
  }

  // Надо обработать полученное PUBLISH
  switch( SIM800.mqttClient.topicId ){
    case TOPIC_DEVID:
      mqttMsgReset( handle, &SIM800 );
      break;
    case TOPIC_DEV_IMEI:
			break;
    case TOPIC_INFO:
			break;
    case TOPIC_TEMP:
      mqttMsgReset( handle, &SIM800 );
      break;
    case TOPIC_VOLT:
      mqttMsgReset( handle, &SIM800 );
			break;
    case TOPIC_CMD_I:
			break;
    case TOPIC_CMD_O:
			break;
    case TOPIC_CFG_I:
			break;
    case TOPIC_CFG_O:
			break;
    case TOPIC_ALRM:
			break;
    case TOPIC_LOG:
			break;
    case TOPIC_ISENS:
			break;
    case TOPIC_ISENS_ARX:
			break;
    case TOPIC_ISENS_STATE:
			break;
    case TOPIC_ISENS_ADC:
			break;
    case TOPIC_ISENS_TEMP:
			break;
    case TOPIC_OUT:
			break;
    case TOPIC_OUT_STATE:
			break;
    case TOPIC_FW:
			break;
    case TOPIC_FW_MAN:
      fwManProc(  handle, &(SIM800.mqttClient) );
			break;
    case TOPIC_FW_BIN:
      // Приняли/принимаем Обновление прошивки
      fwUpProc( handle, &(SIM800.mqttClient) );
			break;
    case TOPIC_RS:
			break;
    case TOPIC_RS_TX:
			break;
    case TOPIC_RS_RX:
			break;
    case TOPIC_GSM:
			break;
    default:
      //Сюда не должны попасть
      break;
  }

}


/**
 * initialization SIM800.
 * @param NONE
 * @return error status, 0 - OK
 */
void mqttInit(void) {
  SIM800.mqttServer.pppconn = RESET;
  SIM800.mqttServer.gprsconn = RESET;
    SIM800.mqttClient.client.conn_state = TCP_DISCONNECTED;
//    char str[32] = {0};

    // MQQT settings
    SIM800.sim.apn = "internet";
    SIM800.sim.apn_user = "";
    SIM800.sim.apn_pass = "";
    SIM800.mqttServer.host = "test.mosquitto.org";
    SIM800.mqttServer.addr.addr = 0;
    SIM800.mqttServer.port = 1883;
    SIM800.mqttClient.clientInfo.client_user = NULL;
    SIM800.mqttClient.clientInfo.client_pass = NULL;
    SIM800.mqttClient.clientInfo.client_id = "";
    SIM800.mqttClient.clientInfo.keep_alive = 60;
    SIM800.mqttClient.clientInfo.will_topic = NULL;
    SIM800.ready = RESET;

    timerSetup( &mqttPubTimer, mqttPubTout, (uintptr_t)NULL );
}


void mqttConnectCb( mqtt_client_t *client, void *arg, mqtt_connection_status_t conn ){
  (void)client;
  (void)arg;
  if( conn ){
    mqttSubFlag = SET;
    mqttPubFlag = SET;
    ledOff( LED_R, 0 );
  }
  else {
    mqttPubFlag = RESET;
    mqttSubFlag = RESET;
    // Две вспышки оранжевого цвета с интервалом в 3 сек
    ledToggleSet( LED_R, LED_BLINK_ON_TOUT, LED_SLOW_TOGGLE_TOUT, TOUT_3000, 2);
    ledToggleSet( LED_G, LED_BLINK_ON_TOUT, LED_SLOW_TOGGLE_TOUT, TOUT_3000, 2);
  }
}


/**
 * Starting MQTT process.
 * @param NONE
 * @return error status, 0 - OK
 */
int mqttStart(void) {
  gsmSendCommand("ATE0\r\n", "OK\r\n", CMD_DELAY_2, NULL );
  return gsmSendCommand("AT+CIPMODE=1\r\n", "OK\r\n", CMD_DELAY_2, NULL );
}


void mqttProcess( void ){

  if(gsmState < GSM_WORK){
    return;
  }

  if( mqttPubFlag ) {
    mqttPubFlag = RESET;
  //    MQTT_Pub( "imei/test/string", "String message" );
  //    MQTT_PubUint8( "imei/test/uint8", pub_uint8 );
  //    MQTT_PubUint16( "imei/test/uint16", pub_uint16 );
  //    MQTT_PubUint32( "imei/test/uint32", pub_uint32 );
  //    MQTT_PubFloat( "imei/test/float", pub_float );
  //    MQTT_PubDouble( "imei/test/double", pub_double );
    if( iSens[ISENS_1].isensFlag ){
      char str[64];
      uint32_t ut = getRtcTime();

      sprintf( str, "{\"state\":[{time\":%ul,\"pls\":%ul}]}", (unsigned int)ut, (unsigned int)iSens[ISENS_1].isensCount );
      MQTT_Pub( "imei/i/1", str );
      iSens[ISENS_1].isensFlag = RESET;
    }
    else {
      // Считываем из ЛОГа
      if( logRdBufFill == 0 ){
        logQueryProcess();
      }
      if( logRdBufFill ){
        for( uint8_t i = 0; i < logRdBufFill; i++ ){
          char str[64];
          sLogRec * logrec = &(logRdBuf[i]);

          sprintf( str, "{\"arx\":[{time\":%ul,\"pls\":%ul}]}", (unsigned int)logrec->utime, (unsigned int)logrec->data );
          MQTT_Pub( "imei/i/1", str );
          logRdBufFill = 0;
        }
      }
      else {
        MQTT_PingReq();
//        char str[64];
//        int tu, td;
//        uint32_t ut = getRtcTime();
//
//        tu = adcHandle.adcVbat / 1000;
//        td = adcHandle.adcVbat - (tu * 1000);
//        if( td < 0 ){
//          td = -td;
//        }
//        sprintf( str, "{time\":%u,\"volt\":%d.%d}", (unsigned int)ut, tu, td );
//        MQTT_Pub( "imei/volt", str );
//
//        tu = adcHandle.adcTemp / 10;
//        td = adcHandle.adcTemp - (tu * 10);
//        if( td < 0 ){
//          td = -td;
//        }
//        sprintf( str, "{time\":%u,\"temp\":%d.%d}", (unsigned int)ut, tu, td );
//        MQTT_Pub( "imei/temp", str );
      }
    }
    timerMod( &mqttPubTimer, MQTT_PUB_TOUT );
  }

  // Обработка принятых сообщений
  mqttPubProc( simHnd.rxh );

}
