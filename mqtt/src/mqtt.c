/*
 * mqtt.c
 *
 *  Created on: 21 окт. 2021 г.
 *      Author: jet
 */
#include <string.h>
#include <stddef.h>

#include "gpio_arch.h"
#include "usart_arch.h"
#include "MQTTSim800.h"
#include "adc.h"
#include "fw.h"
#include "gsm.h"
#include "isens.h"
#include "buffer.log.h"
#include "uspd.h"
#include "events.h"
#include "mqtt.h"

// Для ТЕСТА
extern struct timer_list bigOnToutTimer;

extern sFwHandle fwHandle;
extern FlagStatus fwUpdFlag;
extern FlagStatus fwUpdTimFlag;

extern struct timer_list mqttSubTimer;
extern uint16_t logRdBufFill;
extern logBuf_t logRdSensBuffer;
extern logBuf_t logRdEvntBuffer;

const char * tpcTempl[TOPIC_NUM] = {
  "r/device",   		      //  TOPIC_DEV_IMEI,
  "d/%s",                 //  TOPIC_DEV_UID,
  "%s/info",         		  //  TOPIC_INFO,
  "%s/temp",       		    //  TOPIC_TEMP,
  "%s/volt",       		    //  TOPIC_VOLT,
  "%s/cmdi",           		//  TOPIC_CMD_I,      *
  "%s/cmdo",           		//  TOPIC_CMD_O,
  "%s/cfgi",       	    	//  TOPIC_CFG_I,      *
  "%s/cfgo",       		    //  TOPIC_CFG_O,
  "%s/alrm",       		    //  TOPIC_ALRM,
  "%s/log",       		    //  TOPIC_LOG,
  "%s/i/%d",       		    //  TOPIC_ISENS,
  "%s/i/%d/arx",      		  //  TOPIC_ISENS_ARX,
  "%s/i/%d/state",     		//  TOPIC_ISENS_STATE,
  "%s/i/%d/adc",       		//  TOPIC_ISENS_ADC,
  "%s/i/%d/temp",     		  //  TOPIC_ISENS_TEMP,
  "%s/o/%d",       		    //  TOPIC_OUT,        *
  "%s/o/%d/state",       	//  TOPIC_OUT_STATE,
  "%s/fw",       		      //  TOPIC_FW,
  "%s/fw/man",       		  //  TOPIC_FW_MAN,     *
  "%s/fw/bin",       		  //  TOPIC_FW_BIN,     *
  "%s/rs",       		      //  TOPIC_RS,
  "%s/rs/tx",       		  //  TOPIC_RS_TX,      *
  "%s/rs/rx",       		  //  TOPIC_RS_RX,      *
  "%s/gsm",       		    //  TOPIC_GSM,
  //  TOPIC_NUM
};

SIM800_t SIM800;

// Список для Подписки
const struct {
  char * subtpc;
  eTopicId tpid;
  uint8_t qos;
} subList[] = {
  { "%s/cmdi", TOPIC_CMD_I, 2 },
  { "%s/cfgi", TOPIC_CFG_I, 1 },
  { "%s/o/1", TOPIC_OUT, 1 },
  { "%s/fw/man", TOPIC_FW_MAN, 2 },
  { "%s/fw/bin", TOPIC_FW_BIN, 2 },
  { "%s/rs/tx", TOPIC_RS_TX, 1 },
  { "%s/rs/rx", TOPIC_RS_RX, 1 },
};

// --------------------- Private Functions prototype ----------------------------
void mqttConnectCb( FlagStatus conn );
HAL_StatusTypeDef   stmEeRead( uint32_t addr, uint32_t * data, uint32_t datalen);
int8_t evntPubProc( sLogRec * rec );

// ------------------------------------------------------------------------------

void mqttPingTout( uintptr_t arg ){
  (void)arg;

  if( SIM800.mqttServer.mqttconn == 1 ) {
    mqttPingFlag = SET;
  }
}


void mqttSubTout( uintptr_t arg ){
  (void)arg;

  if( SIM800.mqttServer.mqttconn == 1 ) {
    mqttSubFlag = SUB_SET;
//    mqttSubFlag = SET;
  }
  else {
    mqttSubFlag = SUB_NONE;
  }
//  timerMod( &mqttSubTimer, MQTT_SUB_TOUT );
}


// Таймаут: не приняли отклик на публикацию с QOS != 0
void mqttPubTout( uintptr_t arg ){
  (void)arg;

  switch( uspd.runMode ){
    case RUN_MODE_FIRST:
      gsmStRestart = GSM_OFF;
      break;
    case RUN_MODE_KEY:
      // TODO: Организовать ожидание, прием, выполнение команд и последующее выключение
      break;
    case RUN_MODE_SENS_SEND:
      // Засыпаем до следующего включения по календарю
      gsmStRestart = GSM_OFF;
      trace_puts( "SENS send sleep ");
      toSleep( SET );
      break;
    default:
      break;
  }

  gsmRunPhase = PHASE_NON;
  // Все сделано - засыпаем до следующего раза
  sensPubAlrmSet( &(uspd.arxCal) );
  gsmReset = MCU_SLEEP;
  gsmRun = RESET;
}


// --------------- Public functions ----------------------------------------
FlagStatus cfgoPubFunc( void ){
  FlagStatus rc = SET;
  char str[64];
  char * cfgomsg;

  sprintf( str, tpcTempl[TOPIC_CFG_O], SIM800.sim.imei );
  if( (cfgomsg = cfgoMsgCreate()) == NULL ){
    ErrHandler( NON_STOP );
    return rc;
  }
  else {
    if( MQTT_Pub( str, cfgomsg, QOS1, SIM800.mqttReceive.pktIdo )){
      uspd.cfgoPktId = SIM800.mqttReceive.pktIdo;
      SIM800.mqttReceive.pktIdo++;
      rc = RESET;
    }
    else {
      ErrHandler( NON_STOP );
    }
    // Передали на отправку в UART. Удачно-нет - освобождаем;
//    trace_printf( "f_cfgo_%x\n", cfgomsg );
    free( cfgomsg );
  }

  return rc;
}

FlagStatus uspdAnnouncePub( void ){
  char tpc[32];
  char pay[64] = "{\"imei\":";
  uint32_t ut = getRtcTime();
  int tu, td;
  sFwHandle * eeFwh = (sFwHandle *)FW_HANDLE_ADDR_0;
  uint32_t fw1;
  uint32_t fw2;

  stmEeRead( (uint32_t)&(eeFwh->fw[0]), (uint32_t*)&fw1, sizeof(fw1) );
  stmEeRead( (uint32_t)&(eeFwh->fw[1]), (uint32_t*)&fw2, sizeof(fw2) );

  // Передача IMEI
  strcat( pay, SIM800.sim.imei );
  strcat( pay, "}" );

  if( MQTT_Pub( tpcTempl[TOPIC_DEV_IMEI], pay, QOS1, SIM800.mqttReceive.pktIdo++ ) == 0){
    goto err_exit;
  }

  // Передача UID MCU
  memset( tpc, 0, 32);
  memset( pay, 0, 64);
  sprintf( tpc, tpcTempl[TOPIC_DEV_UID], SIM800.sim.imei );
  sprintf( pay, "{\"uid\":\"%08x%08x%08x\"}", (uint)UID_0, (uint)UID_1, (uint)UID_2 );
  if( MQTT_Pub( tpc, pay, QOS1, SIM800.mqttReceive.pktIdo++ ) == 0 ){
    goto err_exit;
  }

  // Передача RealTime
  memset( tpc, 0, 32);
  memset( pay, 0, 64);
  sprintf( tpc, tpcTempl[TOPIC_INFO], SIM800.sim.imei );
  sprintf( pay, "{time\":%u,\"state\":\"con\"}", (unsigned int)ut );
  if( MQTT_Pub( tpc, pay, QOS1, SIM800.mqttReceive.pktIdo ) != 0 ){
//    uspd.announcePktId = SIM800.mqttReceive.pktIdo;
    SIM800.mqttReceive.pktIdo++;
  }
  else {
    goto err_exit;
  }

  // Передача прошивки
  memset( tpc, 0, 32);
  memset( pay, 0, 64);
  sprintf( tpc, tpcTempl[TOPIC_FW], SIM800.sim.imei );
  sprintf( pay, "{fwf:\"%u.%u.%u\",fws:\"%u.%u.%u\",boot:\"%u\"}", \
      (unsigned int)((fw1 >> 16) & 0xFF), (unsigned int)((fw1 >> 8) & 0xFF), (unsigned int)(fw1 & 0xFF),
      (unsigned int)((fw2 >> 16) & 0xFF), (unsigned int)((fw2 >> 8) & 0xFF), (unsigned int)(fw2 & 0xFF),
      fwHandle.fwActive + 1 );
  if( MQTT_Pub( tpc, pay, QOS1, SIM800.mqttReceive.pktIdo ) != 0 ){
    uspd.annPktId = SIM800.mqttReceive.pktIdo;
    SIM800.mqttReceive.pktIdo++;
  }
  else {
    goto err_exit;
  }


  tu = adcHandle.adcVbat / 1000;
  td = adcHandle.adcVbat - (tu * 1000);
  if( td < 0 ){
    td = -td;
  }
  memset( tpc, 0, 32);
  memset( pay, 0, 64);
  sprintf( tpc, tpcTempl[TOPIC_VOLT], SIM800.sim.imei );
  sprintf( pay, "{time\":%u,\"volt\":%d.%d}", (unsigned int)ut, tu, td );
  if( MQTT_Pub( tpc, pay, QOS0, 0 ) == 0 ){
    goto err_exit;
  }

  tu = adcHandle.adcTemp / 10;
  td = adcHandle.adcTemp - (tu * 10);
  if( td < 0 ){
    td = -td;
  }
  memset( tpc, 0, 32);
  memset( pay, 0, 64);
  sprintf( tpc, tpcTempl[TOPIC_TEMP], SIM800.sim.imei );
  sprintf( pay, "{time\":%u,\"temp\":%d.%d}", (unsigned int)ut, tu, td );
  if( MQTT_Pub( tpc, pay, QOS0, 0 ) == 0 ){
    goto err_exit;
  }

  return RESET;

err_exit:
  ErrHandler( NON_STOP );
  uspd.annPktId = -1;
  return SET;
}


int archPubFunc( void ){
  char tpc[32];
  char pay[11 + 36*8];
  sLogRec rec[8];
//  uint8_t quant = 8;
//  uint8_t quantEv = 4;

  uspd.pubEvQuant = logBuf_Read( &logRdEvntBuffer, rec, 1 );
  if( uspd.pubEvQuant ){
    for( uint8_t qu = 0; qu < uspd.pubEvQuant; qu++){
      logRdBufFill--;
      if( logRdBufFill == 0 ){
        SIM800.mqttClient.pubFlags.archPub = RESET;
        if( (uspd.readArchSensQuery == RESET)
            && (uspd.readArchEvntQuery == RESET) )
        {
          // Это публикация последнего сенсора и событий для публикации нет.
          uspd.archPktId = SIM800.mqttReceive.pktIdo;
          trace_printf("ArchPktId %d", uspd.archPktId);
        }
      }
      if( (evntPubProc( rec ) == -1)
          && (uspd.archPktId == SIM800.mqttReceive.pktIdo) ){
        // Отправка последней записи безуспешная
        SIM800.mqttClient.pubFlags.archPubEnd = SET;
      }
    }

    return uspd.pubEvQuant;
  }

  uspd.pubQuant = logBuf_Read( &logRdSensBuffer, rec, 8);
  if( uspd.pubQuant ){
    // Есть запись из архива
    logRdBufFill -= uspd.pubQuant;
    for( eIsens is = ISENS_1; is < ISENS_NUM; ){
      uint8_t l;

      sprintf( tpc, tpcTempl[TOPIC_ISENS_ARX], SIM800.sim.imei, is + 1);
      // Начало сообщения
      strcpy( pay, "{\"arch\":" );
      for( uint8_t qu = 0; qu < uspd.pubQuant; qu++){
        char d[11];

        // Данные датчика
        assert_param( rec[qu].devid <= DEVID_ISENS_4 );

        strcat( pay, "[\"time\":" );
        memset(d, 0, 11);
        itoa( rec[qu].utime, d, 10 );
        strcat( pay, d);
        strcat( pay, ",\"pls\":" );
        memset(d, 0, 11);
        itoa( rec[qu].data[is], d, 10 );
        strcat( pay, d);
        strcat(pay, "],");
      }
      // Убираем последнюю ','
      l = strlen( pay ) - 1;
      pay[l++] = '}';
      pay[l] = '\0';
      // ----------- Публикуем -----------------------
      is++;
      if( (is == ISENS_NUM) && (logRdBufFill == 0) ){
        if( (uspd.readArchSensQuery == RESET)
            && (uspd.readArchEvntQuery == RESET)){
          // Это публикация последнего сенсора и событий для публикации нет.
          uspd.archPktId = SIM800.mqttReceive.pktIdo;
          trace_printf("ArchPktId %d", uspd.archPktId);
          SIM800.mqttClient.pubFlags.archPub = RESET;
        }
      }
      if( MQTT_Pub( tpc, pay, QOS2, SIM800.mqttReceive.pktIdo ) == 0 ){
        ErrHandler( NON_STOP );
        return 0;
      }
      SIM800.mqttReceive.pktIdo++;
    }
    mDelay(1000);

    return uspd.pubQuant;
  }

  // Больше записей не осталось
  assert_param( logRdBufFill == 0);
  if( (uspd.archPktId == 0)
      && (uspd.readArchSensQuery == RESET)
      && (uspd.readArchEvntQuery == RESET))
  {
    // ARCH не  отправляли.
    trace_puts("PubArch0 end");
    SIM800.mqttClient.pubFlags.archPubEnd = SET;
    SIM800.mqttClient.pubFlags.archPub = RESET;
  }

  return -1;
}


// -------------------------------------------------------------------------

int mqttSubProcess(void){
  uint8_t sc = SIM800.mqttClient.subCount;

  if( mqttSubFlag == SUB_SET ){
//  if( mqttSubFlag ){
    char topicstr[23];

    mqttSubFlag = SUB_NONE;
    if( sc == ARRAY_SIZE(subList) ){
      timerDel( &mqttSubTimer );
      return 0;
    }
    else {
      timerMod( &mqttSubTimer, MQTT_SUB_TOUT );
    }
    sprintf(topicstr, subList[sc].subtpc, SIM800.sim.imei);
    // Подписываемся, пока не получим подтверждение на ВСЕ подписки
    MQTT_Sub( topicstr, subList[sc].qos );
  }

  return 1;
}

// Получение  флагов сообщения
void msgFlagSet( uint8_t flags, mqttReceive_t * receive ){
  receive->dup = flags & 0x08;
  receive->qos = (flags >> 1) & 0x03;
  receive->retained = flags & 0x01;
}


// Обработка принятого контрольного пакета (НЕ PUBLISH)
void mqttCtlProc( SIM800_t * sim ){
  uint16_t pktid = sim->mqttReceive.pktId;

  switch( sim->mqttReceive.msgType ) {
    case MQTT_CONNACK:
      trace_printf( "CONACK: %d\n", pktid );
      sim->mqttServer.mqttconn = SET;
      sim->mqttClient.subCount = 0;
      mqttConnectCb( SIM800.mqttServer.mqttconn );
      break;
    case MQTT_SUBACK:
      // TODO: Сбросить таймер сообветствующей подписки
      sim->mqttClient.subCount++;
      mqttSubFlag = SUB_SET;
//      timerStack( &mqttSubTimer, 0, TIMER_MOD );
      trace_printf( "SUBACK: %d\n", pktid );
      break;
    case MQTT_PUBACK:
      // Отклик на отправку пакета с QOS1
      trace_printf( "PUBACK: %d\n", pktid );
      // Можно публиковать следующее
      SIM800.mqttClient.pubReady--;
      if(uspd.cfgoPktId == pktid ){
        // Получили подтверждение успешной отправки <imei/cfgo>
        uspdCfg.updateFlag = SET;
        uspd.cfgoPktId = -1;
      }
      else if(uspd.annPktId == pktid ){
        // Получили подтверждение успешной отправки "announce"
        SIM800.mqttClient.pubFlags.uspdAnnounce = RESET;
        SIM800.mqttClient.pubFlags.announceEnd = SET;
#if DEBUG_GSM_TRACE
          trace_puts("Annon end");
        // trace_puts("Del BIG_TOUT");
#endif
        if( uspd.runMode == RUN_MODE_SENS_SEND ){
          uspd.readArchSensQuery = SET;
          uspd.readArchEvntQuery = SET;
        }
        timerDel( &bigOnToutTimer );
        uspd.annPktId = -1;
      }
      else if( uspd.archPktId == pktid ){
        uspd.archPktId = -1;
#if DEBUG_GSM_TRACE
        trace_puts("PubArch end");
        SIM800.mqttClient.pubFlags.archPubEnd = SET;
        // trace_puts("Del BIG_TOUT");
#endif
      }
      // Отключить таймер для пакета с этим PKT_ID
      break;
    case MQTT_PUBREC:
      // Отклик на получение пакета с QOS2 (фаза1 QOS2)
      trace_printf( "PUBREC: %d\n", pktid );
      SIM800.mqttClient.pubReady--;
      MQTT_Pubrel( pktid );
      break;
    case MQTT_PUBREL:
      // Отклик на отправку пакета PUBREC (фаза2 QOS2)
      trace_printf( "PUBREL: %d\n", pktid );
      MQTT_Pubcomp( pktid );
      if( fwHandle.fwUp.fwUpOk ){
        // TODO: Запуск выключения и перезагрузки
        mDelay(50);
        gsmStRestart = GSM_OFF;
        gsmReset = MCU_RESET;
        gsmRun = RESET;
      }
      break;
    case MQTT_PUBCOMP:
      trace_printf( "PUBCOMP: %d\n", pktid );
      if(uspd.archPktId == pktid ){
#if DEBUG_GSM_TRACE
        trace_puts("PubArch end");
        // trace_puts("Del BIG_TOUT");
#endif
        SIM800.mqttClient.pubFlags.archPubEnd = SET;
        uspd.archPktId = -1;
      }
      // Отключить таймер для пакета с этим PKT_ID (QOS 2)
      break;
    case MQTT_PINGRESP:
      trace_puts( "PINGRESP" );
      break;
    case MQTT_DISCONNECT:
      sim->mqttServer.mqttconn = RESET;
      mqttConnectCb( SIM800.mqttServer.mqttconn );
      break;
    default:
      return;
  }
}


void mqttMsgProc( sUartRxHandle * handle, SIM800_t * sim ){
  uint8_t * msgptr;
  uint16_t len0;
  uint32_t len;

  msgptr = sim->mqttReceive.mqttData;
  len  = ((handle->rxFrame + handle->frame_offset) - msgptr);
  len0 = len;

  while ( len0 ){
    switch ( sim->mqttReceive.msgState ){
      case MSG_NULL:
        msgptr = handle->rxFrame;

        sim->mqttReceive.msgType = (*msgptr & 0xF0) >> 4;
        msgFlagSet( *msgptr, &(sim->mqttReceive) );

        switch( sim->mqttReceive.msgType ) {
          case MQTT_PINGRESP:
          case MQTT_PUBLISH:
          case MQTT_CONNACK:
          case MQTT_PUBACK:
          case MQTT_PUBREC:
          case MQTT_PUBREL:
          case MQTT_PUBCOMP:
          case MQTT_SUBACK:
            break;
          default:
            // Топик не наш - Не сохраняем payload
            goto msg_reset;
        }
        sim->mqttReceive.msgState = MSG_TYPE;
        sim->mqttReceive.remLenMp = 1;
        sim->mqttReceive.remLen = 0;
        len0--;
        msgptr++;
        break;
      case MSG_TYPE: {
        // Длина пакета (2097152 = 128*128*128)
        for( ; len0 ; len0-- ){
          // Получаем длину данных
          sim->mqttReceive.remLen += (*msgptr % 128) * sim->mqttReceive.remLenMp;
          if( (*msgptr++ & 0x80) == 0 ){
            // Принята вся Remaining Lenght
            if( sim->mqttReceive.remLen > 268435455 ){
              trace_printf( "REMLEN BIG, PKT_ID: %X\n", sim->mqttReceive.msgType );
              goto msg_reset;
            }
            else  if (sim->mqttReceive.remLen == 0) {
              // Ошибка длины или Нулевая длина
              if( sim->mqttReceive.msgType == 0xD ){
                mqttCtlProc( sim );
              }
              else {
                trace_printf( "REMLEN = 0, PKT_ID: %X\n", sim->mqttReceive.msgType );
              }
              goto msg_reset;
            }
            else {
              sim->mqttReceive.msgState = MSG_REMAINING_LEN;
            }
            len0--;
            break;
          }
          sim->mqttReceive.remLenMp *= 128;
          if( sim->mqttReceive.remLenMp > 2097152){
            goto msg_reset;
          }
        }

        break;
      }
      case MSG_REMAINING_LEN:
        if( sim->mqttReceive.remLen <= 3 ) {
          if( len0 == sim->mqttReceive.remLen ){
            // Принят контрольный пакет
            sim->mqttReceive.pktId = *msgptr << 8;
            msgptr++;
            sim->mqttReceive.pktId  |= *msgptr;
            mqttCtlProc( sim );
            goto msg_reset;
          }
          len0 = 0;
        }
        else {
          if( len0 >= 2 ){
            // Принята длина топика
            assert_param( msgptr != NULL );
            sim->mqttReceive.topicLen = *msgptr << 8;
            msgptr++;
            sim->mqttReceive.topicLen |= *msgptr;
            msgptr++;
            len0 -= 2;
            sim->mqttReceive.msgState = MSG_TOP_LEN;
          }
          else {
            len0 = 0;
          }
        }
        break;
      case MSG_TOP_LEN: {
        uint8_t size = sim->mqttReceive.topicLen;

        if( len >= size ){
          // Приняn топик
          // (15бит<IMEI> + '\0' - 1 + 1бит'/'
          uint8_t offset = sizeof(IMEI) - 1 + 1;
          eTopicId tp;
          uint8_t * tpc = msgptr + offset;
          uint16_t tpsize = size - offset;

          // Сохраняем только значимую часть топика, без "<IMEI>/"
          for( tp = 0; tp < ARRAY_SIZE(subList); tp++ ){
            if( *tpc == 'o' ){
              // Топик "o" - проверяем без продолжения
              tpsize = 1;
            }
            if( memcmp( tpc, (subList[tp].subtpc+3), tpsize ) == 0 ){
              memcpy(sim->mqttReceive.topic, tpc, tpsize );
              break;
            }
          }

          if( tp >= ARRAY_SIZE(subList)){
            sim->mqttReceive.topicId = TOPIC_NUM;
          }
          else {

            if( (sim->mqttReceive.topicId = subList[tp].tpid) == TOPIC_FW_BIN ){
              // Это обновление прошивки - запускаем таймер таймаута получения прошивки
              fwUpdTimFlag = SET;
//              timerStack( &fwUpdTimer, TOUT_1000 * 120, TIMER_MOD );  // 2 минуты
              fwUpdFlag = SET;
            }
          }
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
          sim->mqttReceive.pktId = *msgptr << 8;
          msgptr++;
          sim->mqttReceive.pktId |= *msgptr;
          msgptr++;
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
        trace_printf("paylen: %lu\n", sim->mqttReceive.payloadLen );
        if( sim->mqttReceive.payloadLen == 0 ){
          goto msg_reset;
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
            goto msg_reset;
          }
          else {
            ledOn( LED_G, 100 );
            // Получено сообщение целиком
            // TODO: Обработка полученых данных - сделать не в ПРЕРЫВАНИИ
            handle->rxProcFlag = SET;
            sim->mqttReceive.payOffset = msgptr - handle->rxFrame;
            if( sim->mqttReceive.qos == 1 ){
              MQTT_Puback( sim->mqttReceive.pktId );
            }
            else if( sim->mqttReceive.qos == 2 ) {
              trace_printf( "PUB rec: %d\n", sim->mqttReceive.pktId );
              MQTT_Pubrec( sim->mqttReceive.pktId );
            }
          }
          len0 = 0;
          sim->mqttReceive.msgState = MSG_NULL;
        }
        else if( len == 1024 ){
          if( sim->mqttReceive.topicId == TOPIC_FW_BIN ){
            // Прием прошивки - запись во флеш
            // TODO: Запись во Флеш сделать не в ПРЕРЫВАНИИ
            sim->mqttReceive.payOffset = msgptr - handle->rxFrame;
            handle->rxProcFlag = SET;
            len0 = 0;
          }
          else {
            goto msg_reset;
          }
        }
        else {
          assert_param( len <= 1024);
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
  return;

msg_reset:
  mqttMsgReset( handle, &SIM800 );
  return;
}


void mqttPubRecvProc( sUartRxHandle * handle ){
  if( handle->rxProcFlag == RESET ){
    return;
  }

  // Надо обработать полученное PUBLISH
  switch( SIM800.mqttReceive.topicId ){
    case TOPIC_CMD_I:
			break;
    case TOPIC_CFG_I:
      uspdCfgProc( handle, &SIM800 );
			break;
    case TOPIC_OUT:
			break;
    case TOPIC_FW_MAN:
      fwManProc(  handle, &(SIM800.mqttReceive) );
			break;
    case TOPIC_FW_BIN:
      // Приняли/принимаем Обновление прошивки
      fwUpProc( handle, &(SIM800.mqttReceive) );
			break;
    case TOPIC_RS_TX:
			break;
    case TOPIC_RS_RX:
			break;
    default:
      //Сюда не должны попасть
      mqttMsgReset( handle, &SIM800 );
      break;
  }

}


void mqttPubProc( uPubFlags * pubfl ){
  if( pubfl->cfgoPub ){
  // Требуются некоторые публикации
    pubfl->cfgoPub = cfgoPubFunc();         // Если успешно - флаг сбрасываем
#if DEBUG_GSM_TRACE
    trace_puts("CFGO Pub");
#endif
  }
  else if( pubfl->uspdAnnounce ){
    pubfl->uspdAnnounce = uspdAnnouncePub();      // Если успешно - флаг сбрасываем
#if DEBUG_GSM_TRACE
    trace_puts("Annce Pub");
#endif
//    if( (pubfl->uspdAnnounce = uspdAnnouncePub()) == RESET ){      // Если успешно - флаг сбрасываем
//      uspdCfg.updateFlag = SET;
//    }
  }
  else if( pubfl->archPub ){

#if DEBUG_GSM_TRACE
    trace_puts("Arch Pub");
#endif
    archPubFunc();
  }
}


void mqttPubInit( void ){
  uspd.cfgoPktId = 0;
  uspd.annPktId = 0;
  uspd.archPktId = 0;
  uspd.cmdPktId = 0;
  uspd.byePktId = 0;
  SIM800.mqttClient.pubFlags.announceEnd = RESET;
  SIM800.mqttClient.pubFlags.archPubEnd = RESET;
}


/**
 * initialization SIM800.
 * @param NONE
 * @return error status, 0 - OK
 */
void mqttInit(void) {
  SIM800.mqttServer.tcpconn = RESET;
  SIM800.mqttServer.mqttconn = RESET;
//    char str[32] = {0};

  timerSetup( &mqttPingTimer, mqttPingTout, (uintptr_t)NULL );
  timerSetup( &mqttSubTimer, mqttSubTout, (uintptr_t)NULL );
  timerSetup( &mqttPubTimer, mqttPubTout, (uintptr_t)NULL );
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
  uPubFlags * pubfl;

  // Удаляем таймер таймаута подписки
  if( mqttSubFlag == SUB_TIM_DEL ){
    timerDel( &mqttSubTimer );
    mqttSubFlag = SUB_NONE;
  }

  // Заводим таймер таймаута обновления прошивки
  if( fwUpdTimFlag ){
    timerMod( &fwUpdTimer, TOUT_1000 * 120 );  // 2 минуты
    fwUpdTimFlag = RESET;
  }

  if( SIM800.mqttServer.disconnFlag && (SIM800.mqttServer.disconnTout < mTick)){
    // Закрыто соединение TCP
#if DEBUG_GSM_TRACE
    trace_puts("TCP close");
#endif
    SIM800.mqttServer.disconnFlag = RESET;
    SIM800.mqttServer.tcpconn = RESET;
    SIM800.mqttServer.mqttconn = RESET;
    mqttConnectCb( SIM800.mqttServer.tcpconn );
    if( gsmRun != RESET ){
      // Это не штатное выключение системы
      evntFlags.mqttClose = SET;
    }
    // Получили и обработали строку - если и принимали что-то, но не до конца - все потеряли
    mqttMsgReset( simHnd.rxh, &SIM800 );
  }

  if(SIM800.mqttServer.mqttconn == RESET){
    return;
  }

  pubfl = &(SIM800.mqttClient.pubFlags);
  if( (SIM800.mqttClient.pubReady < 4)
      && (pubfl->u16pubFlags & PUB_FLAG_MASK) ){
    mqttPubProc( pubfl );
    timerMod( &mqttPingTimer, MQTT_PING_TOUT );
  }
  else if( mqttPingFlag ) {
    mqttPingFlag = RESET;
    MQTT_PingReq();
    timerMod( &mqttPingTimer, MQTT_PING_TOUT );
  }

  // Обработка принятых сообщений
  mqttPubRecvProc( simHnd.rxh );

// ----------------- ISENS PUBLIC -------------------------------------------
//      if( iSens[ISENS_1].isensFlag ){
//        uint32_t ut = getRtcTime();
//
//        sprintf( str, "{\"state\":[{time\":%ul,\"pls\":%ul}]}", (unsigned int)ut, (unsigned int)iSens[ISENS_1].isensCount );
//        MQTT_Pub( "imei/i/1", str, 0, 0 );
//        iSens[ISENS_1].isensFlag = RESET;
//      }
//      else {
//        // Считываем из ЛОГа
//        if( logRdBufFill == 0 ){
//          logQueryProcess();
//        }
//        if( logRdBufFill ){
//          for( uint8_t i = 0; i < logRdBufFill; i++ ){
//            char str[64];
//            sLogRec * logrec = &(logRdBuf[i]);
//
//            sprintf( str, "{\"arx\":[{time\":%ul,\"pls\":%ul}]}", (unsigned int)logrec->utime, (unsigned int)logrec->data );
//            MQTT_Pub( "imei/i/1", str, 0, 0 );
//            logRdBufFill = 0;
//          }
//        }
//        else {
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
//        }
//      }

}
