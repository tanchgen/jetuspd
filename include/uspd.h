/*
 * uspd.h
 *
 *  Created on: 24 нояб. 2021 г.
 *      Author: jet
 */

#ifndef USPD_H_
#define USPD_H_

#include <sys/cdefs.h>

#include "isens.h"
#include "eeprom.h"
#include "mqtt.h"

#define USPD_CFG_ADDR       (EE_CFG_ADDR - FLASH_EEPROM_BASE)
#define USPD_SENS_ADDR      (EE_SENS_ADDR - FLASH_EEPROM_BASE)

typedef enum {
  RUN_MODE_NULL,
  RUN_MODE_FIRST,
  RUN_MODE_KEY,
  RUN_MODE_SENS_WRITE,
  RUN_MODE_SENS_SEND
} eRunMode;

typedef enum {
  PUB_CFGO,
  PUB_ANNOUNCE,
  PUB_ARCH,
  PUB_CMD,
  PUB_BYE
} ePubType;

typedef enum {
  SENS_TYPE_COUNT,
  SENS_TYPE_FLAG,
  SENS_TYPE_RES,
} eSensType;

typedef enum {
  OUT_UPDOWN,
  OUT_PWM,
} eOutState;

typedef enum {
  SIM_SEL_AUTO,
  SIM_SEL_1,
  SIM_SEL_2
} eSimSelect;

typedef enum {
  GPRS_CLASS_AUTO,
  GPRS_CLASS_12,
  GPRS_CLASS_10,
  GPRS_CLASS_8,
} eGprsClass;

typedef struct {
  uint16_t pin;                       // PIN-код. -1 - отключено
  FlagStatus pinAuto;                 // Генерация PIN-кода из MCU_ID
  uint32_t plmn;                      // Код PLMN сотового ператора
  char gprsUser[32];
  char gprsPass[32];
  char gprsApn[32];
  FlagStatus simActiv;                // Флаг периодической активации SIM
  uint16_t simActivTout;              // Период активации SIM в ДНЯХ
  uint8_t simActivMax;                // Макс. кол-во попыток активации SIM
// Текущие значения
  uint16_t simActivDay;               // Кодличество прошедших дней с последней активации
  uint8_t simActivCount;              // Кол-во попыток активации SIM
} sSimCfg;

typedef struct __aligned(4) uspdCfg{
 FlagStatus updateFlag;
 FlagStatus defCfgFlag;
 eSensType isensType[ISENS_NUM];      // Режим работы входов датчиков
 eOutState outState;                  // UP/DOWN вывода Выхода
 uint32_t arxTout;                    // Период записи данных в архив
 FlagStatus arxSend;                  // Флаг разрешения отправки архива на сервер
 char arxCalStr[22];                   // Календарь отправки архива
 FlagStatus autonamur;                // Автоматическое определение уровней срабатывания по сопротивлению
 eSimSelect simSel __aligned(4);      // Режим выбора SIM
 eGprsClass gprsClass __aligned(4);                // Класс GPRS
 uint16_t gprsConnTout;               // Макс. длительность соединения GPRS
 sSimCfg simcfg[2];
 char mqttHost[32];
 uint16_t mqttPort;
 char mqttUser[32];
 char mqttPass[32];
 FlagStatus termGate;                 // Прозрачный режим терминала
} sUspdCfg;

typedef struct {
  eRunMode runMode;
  uint16_t cfgoPktId;
  uint16_t annPktId;
  uint16_t archPktId;
  uint16_t cmdPktId;
  uint16_t byePktId;
  FlagStatus archWrFlag;              // Флаг запуска сохранения среза данных в Архив
  FlagStatus readArchSensQuery;       // Запрос на чтение журнала счетчиков
  FlagStatus readArchEvntQuery;       // Запрос на чтение журнала событий
  FlagStatus defCfgFlag;              // Флаг Установки конфига USPD по умолчанию
  sCalend arxCal;                   // Календарь отправки архива
  uint16_t flashEvQuant;
  uint16_t flashQuant;
  uint16_t pubEvQuant;
  uint16_t pubQuant;
} sUspd;
// ---------------------------------------------------------------------------------

extern sUspd uspd;
extern sUspdCfg uspdCfg;
extern FlagStatus cfgUpdateFinal;

void uspdCfgProc( sUartRxHandle * rxh, SIM800_t * sim );
FlagStatus cfgCalProc( sCalend * cal, char * newstr, char * oldstr );
// Формируем сообщение для топика "TOPIC_CFG_I"
char * cfgoMsgCreate( void );
void uspdCfgInit( FlagStatus * ee );
void uspdInit( void );

#endif /* USPD_H_ */
