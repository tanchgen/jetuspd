/*
 * mqtt.c
 *
 *  Created on: 21 окт. 2021 г.
 *      Author: jet
 */

#include "../inc/mqtt.h"

#include "uart.h"

#include "../inc/MQTTSim800.h"

const char * topicStr[TOPIC_NUM] = {
  "r/device",     		//  TOPIC_DEVID,
  "d/imei"        		//  TOPIC_DEV_IMEI,
  "info"         		  //  TOPIC_INFO,
  "temp"       		    //  TOPIC_TEMP,
  "volt"       		    //  TOPIC_VOLT,
  "cmdi"           		//  TOPIC_CMD_I,
  "cmdo"           		//  TOPIC_CMD_O,
  "cfi"       	    	//  TOPIC_CFG_I,
  "cfo"       		    //  TOPIC_CFG_O,
  "alrm"       		    //  TOPIC_ALRM,
  "log"       		    //  TOPIC_LOG,
  "i/%d"       		    //  TOPIC_ISENS,
  "i/%d/arx"      		//  TOPIC_ISENS_ARX,
  "i/%d/state"     		//  TOPIC_ISENS_STATE,
  "i/%d/adc"       		//  TOPIC_ISENS_ADC,
  "i/%d/temp"     		//  TOPIC_ISENS_TEMP,
  "o/%d"       		//  TOPIC_OUT,
  "o/%d/state"       		//  TOPIC_OUT_STATE,
  "fw"       		//  TOPIC_FW,
  "fw/man"       		//  TOPIC_FW_MAN,
  "fw/bin"       		//  TOPIC_FW_BIN,
  "rs"       		//  TOPIC_RS,
  "rs/tx"       		//  TOPIC_RS_TX,
  "rs/rx"       		//  TOPIC_RS_RX,
  "gsm"       		//  TOPIC_GSM,
  //  TOPIC_NUM
};


// Получение  флагов сообщения
void msgFlagSet( uint8_t flags, mqttReceive_t * receive ){
  receive->dup = flags & 0x08;
  receive->qos = (flags >> 1) & 0x06;
  receive->retained = flags & 0x01;
}


void mqttMsgParse( sUartRxHandle * handle, SIM800_t * sim ){
  static uint8_t * mqttData;
  uint8_t len0;
  const len = (handle + handle->frame_offset) - mqttData;

  len0 = len;

  while ( len0 ){
    switch ( sim->mqttReceive.msgState ){
      case MSG_NULL:
        mqttData = handle->rxFrame;
        mqtt_receive = SET;

        sim->mqttReceive.msgType = *mqttData & 0xF0;
        switch( sim->mqttReceive.msgType ) {
          case MQTT_MSGT_PINGRESP:
            break;
          case MQTT_MSGT_PUBLISH:
            msgFlagSet( *mqttData, &(sim->mqttReceive) );
            break;
          case MQTT_MSGT_CONACK:
            sim->mqttServer.mqttconn = 1;
            mqttConnectCb( SIM800.mqttServer.mqttconn );
            break;
          case MQTT_MSGT_SUBACK:
//              this->subs = TRUE;
            break;
          case MQTT_MSGT_PUBACK:
//              this->pubFree = TRUE;
            break;
          default:
//              this->nextActivity = tmpTime;
        }
        sim->mqttReceive.msgState = MSG_TYPE;
        len0--;
        mqttData++;
        break;
      case MSG_TYPE:
        if( sim->mqttReceive.msgType != MQTT_MSGT_PUBLISH ){
          // Сообщение принято полностью
          mqtt_receive = RESET;
          sim->mqttReceive.msgState = MSG_NULL;
        }
        len0--;
        mqttData++;
        sim->mqttReceive.msgState = MSG_REMAINING_LEN;
        break;
      case MSG_REMAINING_LEN:
        if( len >= 2 ){
          // Принята длина топика
          sim->mqttReceive.topicLen = (*mqttData++ << 8)
                                      | *mqttData;
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
          uint8_t * tpc = mqttData + offset;
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
          mqttData += size;
          sim->mqttReceive.msgState = MSG_TOPIC;
        }
        else {
          len = 0;
        }
        break;
      }
      case MSG_TOPIC:
        sim->mqttReceive.payloadLen = 0;
        uint8_t i;
        for( i = 0; len0 && (i < 4); i++, len0-- ){
          // Получаем длину данных
          sim->mqttReceive.payloadLen = (sim->mqttReceive.payloadLen * 128) +  (*mqttData % 128);
          if( (*mqttData++ & 0x80) == 0 ){
            break;
          }
        }

        if( i == 4){
          // Ошибка длины данных
          mqtt_receive = RESET;
          sim->mqttReceive.msgState = MSG_NULL;
        }
        else if( sim->mqttReceive.payloadLen == 0 ){
          mqtt_receive = RESET;
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
            handle->frame_offset = 0;
          }
          else {
            // TODO: Обработка полученых данных - сделать не в ПРЕРЫВАНИИ
            handle->rxProcFlag = SET;
          }
          mqtt_receive = RESET;
          sim->mqttReceive.msgState = MSG_NULL;
        }
        else if( (len >= 1024) && (sim->mqttReceive.topicId == TOPIC_FW_BIN) ){
          // Прием прошивки - запись во флеш
          // TODO: Запись во Флеш сделать не в ПРЕРЫВАНИИ
          handle->rxProcFlag = SET;
        }

        break;
      case MSG_PAYLOAD:
        break;
      default:
        break;
    }
  }

}


void simUartRxProc( sUartRxHandle * handle ){
  if( handle->rxProcFlag == RESET ){
    return;
  }

  // Надо обработать полученное PUBLISH
  switch( SIM800.mqttReceive.topicId ){
    case TOPIC_DEVID:
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
			break;
    case TOPIC_FW_BIN:
      // Приняли/принимаем Обновление прошивки

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
