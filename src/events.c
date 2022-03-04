/*
 * events.c
 *
 *  Created on: 6 янв. 2022 г.
 *      Author: jet
 */
#include <stddef.h>
#include <stdio.h>

#include "main.h"
#include "buffer.log.h"
#include "topic_id.h"
#include "mqtt.h"
#include "events.h"

extern  logBuf_t logRdEvntBuffer;

uEventFlag evntFlags;

/*
void (* evntFunc[])(void) = {
// Пользовыательские события
  consumpEvFunc,
  ~sb2EvFunc,
  namurEvFunc,          // разрыв линии NAMUR
  ~sb1EvFunc,            // нажатие кнопки sb1
  leakageEvFunc,        // срабатывание датчика протечки
  ~sensFreqEvFunc,       // превышение частоты счета импульсов
// Технологические события
  ~iwdgEvFunc,           // срабатывание watchdog
  ~uspdOnEvFunc,         // включение устройства
  ~resetEvFunc,          // программная перезагрузка устройства
  ~fwUpdEvFunc,          // обновление прошивки
  ~cfgLoadEvFunc,        // загрузка конфигурации
  gsmFaultEvFunc,       // ошибка установки соединения GSM
  gprsFaultEvFunc,      // ошибка установки соединения GPRS
  tcpFualtEvFunc,       // ошибка установки соединения TCP
  ntpFaultEvFunc,       // ошибка синхронизации времени
  mqttFaultEvFunc,      // ошибка подключения к серверу
  mqttCloseEvFunc,      // MQTT обрыв соединения
};
*/

// Строковые шаблоны для сообщений о технологических событиях
const char * evntMsgTempl[] = {
    "wdg",              // срабатывание watchdog
    "pon",              // включение устройства
    "rebt",             // программная перезагрузка устройства
    "fwupd",            // обновление прошивки
    "cfupd",            // загрузка конфигурации
    "gsmerr",           // ошибка установки соединения GSM
    "gprserr",          // ошибка установки соединения GPRS
    "tcperr",           // ошибка установки соединения TCP
    "ntperr",           // ошибка синхронизации времени
    "mqtterr",          // ошибка подключения к серверу
    "brcon",            // MQTT обрыв соединения
};

/*
// --------------------- Пользовыательские события ----------------------------
// TODO: Аномальный расход
void consumpEvFunc(){
 if( logger( getRtcTime(), DEVID_CONSUMP, NULL, 1 ) == 1){
    // Записано в Архив успешно
    evntFlags.consump = RESET;
  }
  else {
    ErrHandler( NON_STOP );
  }
}


// TODO: открытие крышки прибора / отрыв от стены
void coverEvFunc( void ){
}
// TODO: разрыв линии NAMUR
void namurEvFunc( void ){
}
// TODO: нажатие кнопки sb1
void sb1EvFunc( void ){
}
// TODO: срабатывание датчика протечки
void leakageEvFunc( void ){
}
// TODO: превышение частоты счета импульсов
void sensFreqEvFunc( void ){
}

// ----------------------------------------------------------------------------

// --------------------- Технологические события ----------------------------
// TODO: срабатывание watchdog
void iwdgEvFunc( void ){
}
// TODO: включение устройства
void uspdOnEvFunc( void ){
}
// TODO: программная перезагрузка устройства
void resetEvFunc( void ){
}
// TODO: обновление прошивки
void cfgUpdEvFunc( void ){
}
// TODO: загрузка конфигурации
void cfgLoadEvFunc( void ){
}
// TODO: ошибка установки соединения GSM
void gsmFaultEvFunc( void ){
}
// TODO: ошибка установки соединения
void gprsFaultEvFunc( void ){
}
// TODO: GPRS ошибка установки соединения
void tcpFualtEvFunc( void ){
}
// TODO: TCP ошибка синхронизации времени
void ntpFaultEvFunc( void ){
}
// TODO: ошибка подключения к серверу
void mqttFaultEvFunc( void ){
}
// TODO: MQTT обрыв соединения
void mqttCloseEvFunc( void ){
}
// ----------------------------------------------------------------------------
*/

void evntProcess( void ){
  uint8_t evnum = DEVID_NUM - (DEVID_ISENS_4_STATE + 1);
  sLogRec logrec = {0};

  for( uint8_t i = 0; i < evnum; i++ ){
    if( evntFlags.u32evnt & (1<<i) ){
      // Есть событие

      if( logger( &logrec, getRtcTime(), i + (DEVID_ISENS_4_STATE + 1), NULL, 1 ) != 1){
        // Ошибка записи в Архив
        ErrHandler( NON_STOP );
      }
      else {
        evntFlags.u32evnt &= ~(1<<i);
        if( SIM800.mqttClient.evntPubFlag ){
          logRdBufFill += logBuf_Write( &logRdEvntBuffer, &logrec, 1 );
        }
      }
    }
  }

}



int8_t evntPubProc( sLogRec * rec ){
  char tpc[32];
  char pay[11 + 36];

  switch( rec->devid ){
    case DEVID_ISENS_1_STATE:
    case DEVID_ISENS_2_STATE:
    case DEVID_ISENS_3_STATE:
    case DEVID_ISENS_4_STATE:
    case DEVID_ISENS_5: {
      // Состояние датчика (i1 - i5) из Журнала событий
      uint8_t is = rec->devid - DEVID_ISENS_1_STATE;
      sprintf( tpc, tpcTempl[TOPIC_ISENS_STATE], SIM800.sim.imei, is + 1 );
      sprintf( pay, "{\"arch\":[\"time\":%lu,\"state\":%lu]}", rec->utime, rec->data[is] );
      break;
    }
// --------------------- Польховательские события ----------------------------
    case DEVID_ISENS_6:
      sprintf( tpc, tpcTempl[TOPIC_ALRM], SIM800.sim.imei );
      sprintf( pay, "{\"time\":%lu,\"alrm\":\"leak\", \"i\":6}", rec[0].utime );
      break;
    case DEVID_SB_1:
      sprintf( tpc, tpcTempl[TOPIC_ALRM], SIM800.sim.imei );
      sprintf( pay, "{\"time\":%lu,\"alrm\":\"but\"}", rec[0].utime );
      break;
    case DEVID_SB_2:
      sprintf( tpc, tpcTempl[TOPIC_ALRM], SIM800.sim.imei );
      sprintf( pay, "{\"time\":%lu,\"alrm\":\"dopen\"}", rec[0].utime );
      break;
    case DEVID_CONSUMP:
      sprintf( tpc, tpcTempl[TOPIC_ALRM], SIM800.sim.imei );
      sprintf( pay, "{\"time\":%lu,\"alrm\":\"abn_comp\"}", rec[0].utime );
      break;
    case DEVID_NAMUR_1:
    case DEVID_NAMUR_2:
    case DEVID_NAMUR_3:
    case DEVID_NAMUR_4: {
      uint8_t is = rec->devid - DEVID_NAMUR_1 + 1;
      sprintf( tpc, tpcTempl[TOPIC_ALRM], SIM800.sim.imei );
      // TODO: Реализовать КЗ и ОБРЫВ
      sprintf( pay, "{\"time\":%lu,\"alrm\":\"namur\", \"i\":[{\"kz\":%u]}}", rec[0].utime, is );
      break;
    }
    case DEVID_PULSE_1:
    case DEVID_PULSE_2:
    case DEVID_PULSE_3:
    case DEVID_PULSE_4: {
      uint8_t is = rec->devid - DEVID_PULSE_1 + 1;
      sprintf( tpc, tpcTempl[TOPIC_ALRM], SIM800.sim.imei );
      // TODO: Реализовать КЗ и ОБРЫВ
      sprintf( pay, "{\"time\":%lu,\"alrm\":\"ovsh\", \"i\":%u}", rec[0].utime, is );
      break;
    }
// --------------------- Технологические события ----------------------------
    case DEVID_IWDG:
    case DEVID_USPD_ON:
    case DEVID_USPD_RST:
    case DEVID_CFG_UPD:
    case DEVID_CFG_LOAD:
    case DEVID_GSM_FAULT:
    case DEVID_GPRS_FAUL:
    case DEVID_TCP_FAULT:
    case DEVID_NTP_FAULT:
    case DEVID_MQTT_FAULT:
    case DEVID_MQTT_CLOSE:
      sprintf( tpc, tpcTempl[TOPIC_LOG], SIM800.sim.imei );
      sprintf( pay, "{\"time\":%lu,\"log\":\"%s\"}", rec[0].utime, evntMsgTempl[rec->devid - DEVID_IWDG] );
      break;
    default:
      // Неправильный формат записи журнала
      return 0;
      break;
  }

  // Публикуем
  if( MQTT_Pub( tpc, pay, QOS1, SIM800.mqttReceive.pktIdo++ ) == 0 ){
    ErrHandler( NON_STOP );
    return 0;
  }

  return 1;
}
