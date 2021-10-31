/*
 * mqtt.c
 *
 *  Created on: 21 окт. 2021 г.
 *      Author: jet
 */
#include <string.h>

#include "uart.h"
#include "MQTTSim800.h"
#include "adc.h"
#include "fw.h"
#include "gsm.h"
#include "isens.h"
#include "logger.h"
#include "mqtt.h"

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


void mqttConnectCb( FlagStatus conn );


void mqttPubTout( uintptr_t arg ){
  (void)arg;

  if( SIM800.mqttServer.mqttconn == 1 ) {
    mqttPubFlag = SET;
  }
}


// Получение  флагов сообщения
void msgFlagSet( uint8_t flags, mqttReceive_t * receive ){
  receive->dup = flags & 0x08;
  receive->qos = (flags >> 1) & 0x06;
  receive->retained = flags & 0x01;
}


void mqttMsgProc( sUartRxHandle * handle, SIM800_t * sim ){
  uint8_t * msgptr = sim->mqttReceive.mqttData;
  uint8_t len0;
  const uint32_t len = ((handle->rxFrame + handle->frame_offset) - msgptr) + 1;

  len0 = len;

  while ( len0 ){
    switch ( sim->mqttReceive.msgState ){
      case MSG_NULL:
        msgptr = handle->rxFrame;

        sim->mqttReceive.msgType = (*msgptr & 0xF0) >> 4;
        switch( sim->mqttReceive.msgType ) {
          case MQTT_PINGRESP:
            break;
          case MQTT_PUBLISH:
            msgFlagSet( *msgptr, &(sim->mqttReceive) );
            break;
          case MQTT_CONNACK:
            sim->mqttServer.mqttconn = 1;
            mqttConnectCb( SIM800.mqttServer.mqttconn );
            break;
          case MQTT_SUBACK:
//              this->subs = TRUE;
            break;
          case MQTT_PUBACK:
            // Отклик на отправку пакета с QOS1
            // Отключить таймер для пакета с этим PKT_ID
            break;
          case MQTT_PUBREC:
            // Отклик на отправку пакета с QOS2
            // TODO: Send packet PUBREL
            break;
          case MQTT_PUBCOMP:
            // Отключить таймер для пакета с этим PKT_ID (QOS 2)
            break;
          default:
            // Топик не наш - Не сохраняем payload
            mqttBufClean( handle, &SIM800 );
            return;
        }
        sim->mqttReceive.msgState = MSG_TYPE;
        sim->mqttReceive.remLenMp = 1;
        sim->mqttReceive.remLen = 0;
        len0--;
        msgptr++;
        break;
      case MSG_TYPE: {
        // Длина пакета (2097152 = 128*128*128)
        for( ; len0 && (sim->mqttReceive.remLenMp <= 2097152); len0-- ){
          // Получаем длину данных
          sim->mqttReceive.remLen += (*msgptr % 128) * sim->mqttReceive.remLenMp;
          if( (*msgptr++ & 0x80) == 0 ){
            // Принята вся Remaining Lenght
            if( sim->mqttReceive.msgType != MQTT_PUBLISH ){
              // Сообщение принято полностью
              sim->mqttReceive.msgState = MSG_NULL;
            }
            else if( (sim->mqttReceive.remLen < 3) || (sim->mqttReceive.remLen > 268435455) ){
              // Топик не наш - Не сохраняем payload
              mqttBufClean( handle, &SIM800 );
            }
            else {
              sim->mqttReceive.msgState = MSG_REMAINING_LEN;
            }
            len0--;
            break;
          }
          sim->mqttReceive.remLenMp *= 128;
        }

        break;
      }
      case MSG_REMAINING_LEN:
        if( len0 >= 2 ){
          // Принята длина топика
          assert_param( msgptr != NULL );
          sim->mqttReceive.topicLen = (*msgptr++ << 8)
                                      | *msgptr++;
          len0 -= 2;
          sim->mqttReceive.msgState = MSG_TOP_LEN;
        }
        else {
          len0 = 0;
        }
        break;
      case MSG_TOP_LEN: {
        uint8_t size = sim->mqttReceive.topicLen;

        if( len >= size ){
          // Приняn топик
          uint8_t offset = strlen("imei") + 1;
          eTopicId tp;
          uint8_t * tpc = msgptr + offset;
          uint16_t tpsize = size - offset;

          // Сохраняем только значимую часть топика, без "<IMEI>/"
          for( tp = TOPIC_DEVID; tp < TOPIC_NUM; tp++ ){
            if( memcmp( tpc, topicStr[tp], tpsize ) == 0 ){
              memcpy(sim->mqttReceive.topic, tpc, tpsize );
              break;
            }
          }

          sim->mqttReceive.topicId = tp;
          len0 -= size;
          msgptr += size;
          if( sim->mqttReceive.qos ){
            sim->mqttReceive.msgState = MSG_TOPIC;
          }
          else {
            sim->mqttReceive.msgState = MSG_PKT_ID;
          }
        }
        else {
          len0 = 0;
        }
        break;
      }
      case MSG_TOPIC:
        if( len0 >= 2 ){
          // Принят PACKET_ID
          assert_param( msgptr != NULL );
          sim->mqttReceive.msgId = (*msgptr++ << 8)
                                      | *msgptr++;
          len0 -= 2;
          sim->mqttReceive.msgState = MSG_PKT_ID;
        }
        else {
          len0 = 0;
        }
        break;
      case MSG_PKT_ID:
        if( sim->mqttReceive.qos ){
          // Учтем размер полей topicLen + pktId
          sim->mqttReceive.payloadLen = sim->mqttReceive.remLen - (2 + 2) - sim->mqttReceive.topicLen;
        }
        else {
          // Учтем размер поля topicLen
          sim->mqttReceive.payloadLen = sim->mqttReceive.remLen - 2 - sim->mqttReceive.topicLen;
        }

        if( sim->mqttReceive.payloadLen == 0 ){
          sim->mqttReceive.msgState = MSG_NULL;
        }
        else {
          sim->mqttReceive.msgState = MSG_PAY_LEN;
        }
        break;
      case MSG_PAY_LEN:
        if( len >= sim->mqttReceive.payloadLen ){
          // Payload получен в полном объеме
          if( sim->mqttReceive.topicId == TOPIC_NUM ){
            // Топик не наш - Не сохраняем payload
            mqttBufClean( handle, &SIM800 );
            len0 = 0;
          }
          else {
            // Получено сообщение целиком
            // TODO: Обработка полученых данных - сделать не в ПРЕРЫВАНИИ
            handle->rxProcFlag = SET;
            sim->mqttReceive.payOffset = msgptr - handle->rxFrame;
            if( sim->mqttReceive.qos == 1 ){
              // TODO: Send PUBACK packet (QOS 1)
            }
            else {
              // TODO: Send PUBREC packet (QOS 2)
            }
          }
          len0 = 0;
          sim->mqttReceive.msgState = MSG_NULL;
        }
        else if( len >= 1024 ){
          if( sim->mqttReceive.topicId == TOPIC_FW_BIN ){
            // Прием прошивки - запись во флеш
            // TODO: Запись во Флеш сделать не в ПРЕРЫВАНИИ
            sim->mqttReceive.payOffset = msgptr - handle->rxFrame;
            handle->rxProcFlag = SET;
          }
          else {
            mqttBufClean( handle, &SIM800 );
            len0 = 0;
          }
        }
        else {
          len0 = 0;
        }

        break;
      case MSG_PAYLOAD:
        break;
      default:
        break;
    }
  }

  sim->mqttReceive.mqttData = msgptr;

}


void mqttPubProc( sUartRxHandle * handle ){
  if( handle->rxProcFlag == RESET ){
    return;
  }

  // Надо обработать полученное PUBLISH
  switch( SIM800.mqttReceive.topicId ){
    case TOPIC_DEVID:
      mqttBufClean( handle, &SIM800 );
      break;
    case TOPIC_DEV_IMEI:
			break;
    case TOPIC_INFO:
			break;
    case TOPIC_TEMP:
			break;
    case TOPIC_VOLT:
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
      fwManProc(  handle, &(SIM800.mqttReceive) );
			break;
    case TOPIC_FW_BIN:
      // Приняли/принимаем Обновление прошивки
      fwUpProc( handle, &(SIM800.mqttReceive) );
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
  SIM800.mqttServer.tcpconn = 0;
    SIM800.mqttServer.mqttconn = 0;
//    char str[32] = {0};

    // MQQT settings
    SIM800.sim.apn = "internet";
    SIM800.sim.apn_user = "";
    SIM800.sim.apn_pass = "";
    SIM800.mqttServer.host = "test.mosquitto.org";
    SIM800.mqttServer.port = 1883;
    SIM800.mqttReceive.mqttData = simHnd.rxh->rxFrame;
    SIM800.mqttClient.username = NULL;
    SIM800.mqttClient.pass = NULL;
    SIM800.mqttClient.clientID = "";
    SIM800.mqttClient.keepAliveInterval = 60;

    timerSetup( &mqttPubTimer, mqttPubTout, (uintptr_t)NULL );
}


/**
 * Starting MQTT process.
 * @param NONE
 * @return error status, 0 - OK
 */
int mqttStart(void) {
  SIM800_SendCommand("ATE0\r\n", "OK\r\n", CMD_DELAY_2);
  return SIM800_SendCommand("AT+CIPMODE=1\r\n", "OK\r\n", CMD_DELAY_2);
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
        char str[64];
        int tu, td;
        uint32_t ut = getRtcTime();

        tu = adcHandle.adcVbat / 1000;
        td = adcHandle.adcVbat - (tu * 1000);
        if( td < 0 ){
          td = -td;
        }
        sprintf( str, "{time\":%ul,\"volt\":%d.%d}", (unsigned int)ut, tu, td );
        MQTT_Pub( "imei/volt", str );

        tu = adcHandle.adcTemp / 10;
        td = adcHandle.adcTemp - (tu * 10);
        if( td < 0 ){
          td = -td;
        }
        sprintf( str, "{time\":%ul,\"temp\":%d.%d}", (unsigned int)ut, tu, td );
        MQTT_Pub( "imei/temp", str );
      }
    }
    timerMod( &mqttPubTimer, MQTT_PUB_TOUT );
  }

  // Обработка принятых сообщений
  mqttPubProc( simHnd.rxh );

}
