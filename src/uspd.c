/*
 * uspd.c
 *
 *  Created on: 24 нояб. 2021 г.
 *      Author: jet
 */
#include <string.h>
#include <ctype.h>

#include "usart_arch.h"
#include "mqtt.h"
#include "uspd.h"

sUspdCfg uspdCfg = {
  // Режим работы входов датчиков
  .isensType = {
    SENS_TYPE_COUNT,
    SENS_TYPE_COUNT,
    SENS_TYPE_COUNT,
    SENS_TYPE_COUNT,
//    SENS_TYPE_RES,
//    SENS_TYPE_RES
  },
  .outState = OUT_UPDOWN,                     // UP/DOWN вывода Выхода
  .arxTout = 3600,                            // Период записи данных в архив
  .arxSend = SET,                             // Флаг разрешения отправки архива на сервер
  .arxCalend = { 0, 8, {1, 11, 21, 0, 0} },   // Календарь отправки архива
  .autonamur = RESET,                         // Автоматическое определение уровней срабатывания по сопротивлению
  .simSel = SIM_SEL_AUTO,                     // Режим выбора SIM
  .gprsClass = GPRS_CLASS_12,                 // Класс GPRS
  .gprsConnTout = 120,                        // Макс. длительность соединения GPRS
  .simcfg =
  {
   {
     .pin = 0,                            // PIN-код: Младшие 4 десятичные цифры UID
     .pinAuto = RESET,                    // Генерация PIN-кода из MCU_ID
     .plmn = 0,                           // Код PLMN сотового ператора
     .gprsUser = "",
     .gprsPass = "",
     .gprsApn = "internet",
     .simActiv = RESET,                   // Флаг периодической активации SIM
     .simActivTout = 45,                  // Период активации SIM в ДНЯХ
     .simActivMax = 1,                    // Макс. кол-во попыток активации SIM
     // Текущие значения
     .simActivDay = 0,                    // Количество прошедших дней с последней активации
     .simActivCount = 0,                  // Кол-во попыток активации SIM
   },
   {
     .pin = 0,                            // PIN-код: Младшие 4 десятичные цифры UID
     .pinAuto = RESET,                    // Генерация PIN-кода из MCU_ID
     .plmn = 0,                           // Код PLMN сотового ператора
     .gprsUser = "",
     .gprsPass = "",
     .gprsApn = "internet",
     .simActiv = RESET,                   // Флаг периодической активации SIM
     .simActivTout = 45,                  // Период активации SIM в ДНЯХ
     .simActivMax = 1,                    // Макс. кол-во попыток активации SIM
     // Текущие значения
     .simActivDay = 0,                    // Количество прошедших дней с последней активации
     .simActivCount = 0,                  // Кол-во попыток активации SIM
   }
  },

  .mqttHost = "test.mosquitto.org",
  .mqttPort = 1883,
  .mqttUser = "",
  .mqttPass = "",
  .termGate = RESET                           // Прозрачный режим терминала
};


FlagStatus cfgUpdateFinal;
uint16_t announcePktId;


void cfgUpdate( FlagStatus change );

// ======================= Формирование сообщения с конфигурацией =================================
const char * cfgiMsgTedmpl =
"{ "
  "\"i\" : [%d,%d,%d,%d,%d,%d],"            // Режим работы входов датчиков (isensType)
  "\"o\" : [%d],"                           // UP/DOWN вывода Выхода (outState)
  "\"arxtime\": %d,"                        // Период записи данных в архив (arxTout)
  "\"arxupl\": %s,"                         // Флаг разрешения отправки архива на сервер (arxSend: "true"/"false")
  "\"arxsend\" : \"%s * *\","   // Календарь отправки архива (arxCalend: { <min>, <hour>, {<d1>, <d2>, <d3>, <d4>, <d5>} }
  "\"autonamur\": %s,"                      // Автоматическое определение уровней срабатывания по сопротивлению (autonamur: "true"/"false")
  "\"sim\": %d,"                            // Режим выбора SIM (simSel)
  "\"gprs\": %d,"                           // Класс GPRS (gprsClass)
  "\"regtime\" : %d,"                       // Макс. длительность соединения GPRS (gprsConnTout)
  "\"pin\" : [%04d,%04d],"
  "\"syspin\" : [%s,%s],"             // ["true"/"false","true"/"false"]
  "\"op\" : [\"%05d\",\"%05d\"],"           // PLMN_1,PLMN_2
  "\"gprsnm\" : [\"%s\",\"%s\"],"
  "\"gprsps\" : [\"%s\",\"%s\"],"
  "\"gprsapn\" : [\"%s\",\"%s\"],"
  "\"simctl\" : %s,"                        // Флаг периодической активации SIM  ("true"/"false")
  "\"simctlday\" : %d,"                     // Период активации SIM в ДНЯХ (simActivTout)
  "\"simctlnum\" : %d,"                     // Макс. кол-во попыток активации SIM (simActivMax)
  "\"mqad\" : \"%s\","
  "\"mqpr\" : \"%d\","
  "\"mqnm\" : \"%s\","
  "\"mqps\" : \"%s\","
  "\"232\" : %s"                            // Прозрачный режим терминала (termGate)
" }";


#define BOOL_STR(x)    ((x)? "true" : "false")

void calStrCreate( char str[], sArxCal * cal ){
  char * beg = str;

  utoa( cal->min, str, 10 );
  while( *str != '\0' ){
    str++;
  }
  *str++ = ' ';
  utoa( cal->hour, str, 10 );
  while( *str != '\0' ){
    str++;
  }
  *str++ = ' ';
  for( uint8_t i = 0; i < 5; i++ ){
    if( cal->day[i] ){
      utoa( cal->day[i], str, 10 );
      while( *str != '\0' ){
        str++;
      }
      *str++ = ',';
    }
  }
  // Вместо последней запятой ставим терминатор
  str--;
  *str = '\0';

  assert_param( strlen(beg) < 21);
}


// Формируем сообщение для топика "TOPIC_CFG_O"
char * cfgoMsgCreate( void ){
  char * msg;
  char calStr[21];

  if( (msg = ta_alloc(1024)) == NULL ){
    Error_Handler( NON_STOP );
    return NULL;
  }

  calStrCreate( calStr, &(uspdCfg.arxCalend) );

  sprintf( msg, cfgiMsgTedmpl, \
          /* Режим работы входов датчиков (isensType) */
          uspdCfg.isensType[0], uspdCfg.isensType[0], uspdCfg.isensType[0],
          uspdCfg.isensType[0], uspdCfg.isensType[0], uspdCfg.isensType[0],
          /* UP/DOWN вывода Выхода (outState) */
          uspdCfg.outState,
          /* Период записи данных в архив (arxTout) */
          uspdCfg.arxTout,
          /* Флаг разрешения отправки архива на сервер (arxSend: "true"/"false") */
          BOOL_STR(uspdCfg.arxSend),
          /* Календарь отправки архива (arxCalend: { <min>, <hour>, {<d1>, <d2>, <d3>, <d4>, <d5>} } */
          calStr,
          /* Автоматическое определение уровней срабатывания по сопротивлению (autonamur: "true"/"false")*/
          BOOL_STR(uspdCfg.autonamur),
          /* Режим выбора SIM (simSel) */
          uspdCfg.simSel,
          /* Класс GPRS (gprsClass) */
          uspdCfg.gprsClass,
          /* Макс. длительность соединения GPRS (gprsConnTout) */
          uspdCfg.gprsConnTout,
          uspdCfg.simcfg[0].pin, uspdCfg.simcfg[1].pin,
          BOOL_STR(uspdCfg.simcfg[0].pinAuto), BOOL_STR(uspdCfg.simcfg[1].pinAuto),
          /* PLMN_1,PLMN_2 */
          uspdCfg.simcfg[0].plmn, uspdCfg.simcfg[1].plmn,
          uspdCfg.simcfg[0].gprsUser, uspdCfg.simcfg[1].gprsUser,
          uspdCfg.simcfg[0].gprsPass, uspdCfg.simcfg[1].gprsPass,
          uspdCfg.simcfg[0].gprsApn, uspdCfg.simcfg[1].gprsApn,
          /* Флаг периодической активации SIM  ("true"/"false") */
          BOOL_STR(uspdCfg.simcfg[0].simActiv),
          /* Период активации SIM в ДНЯХ (simActivTout) */
          uspdCfg.simcfg[0].simActivDay,
          /* Макс. кол-во попыток активации SIM (simActivMax) */
          uspdCfg.simcfg[0].simActivCount,
          uspdCfg.mqttHost,
          uspdCfg.mqttPort,
          uspdCfg.mqttUser,
          uspdCfg.mqttPass,
          /* Прозрачный режим терминала (termGate) */
          BOOL_STR(uspdCfg.termGate)
      );


  assert_param( strlen(msg) < 800 );
  return msg;
}


// ==================== Парсинг полученого сообщения с конфигурацией ==============================
static inline char * skipch( char * ptr, char ch ){
  while( *ptr != ch ){
    if(*ptr == '\0'){
      return NULL;
    }
    ptr++;
  }
  return ptr;
}

// Поиск имени и величины
// Возврашает указатель на продолжение JSON или NULL, если конец строки
char * volParse( char * str, char ** name, char ** vol ){
  char * ptr = str;
  char * ptr0;

  if( (ptr = skipch( ptr, '\"')) == NULL){
    goto fault_parse;
  }
  ptr0 = ++ptr;

  if( (ptr = skipch( ptr, '\"')) == NULL){
    goto fault_parse;
  }

  *ptr++ = '\0';
  *name = ptr0;

  if( (ptr = skipch( ptr, ':')) == NULL){
    goto fault_parse;
  }
  ptr++;
  while( isspace((int)*ptr) ){
    if(*ptr == '\0'){
      goto fault_parse;
    }
    ptr++;
  }

  if( *ptr == '[' ){
    ptr0 = ++ptr;
    if( (ptr = skipch( ptr, ']')) == NULL){
      goto fault_parse;
    }
  }
  else if( *ptr == '\"' ){
    ptr0 = ++ptr;
    if( (ptr = skipch( ptr, '\"')) == NULL){
      goto fault_parse;
    }
  }
  else {
    ptr0 = ptr;
    while( isalnum((int)*ptr) ){
      if(*ptr == '\0'){
        goto fault_parse;
      }
      ptr++;
    }
  }
  *ptr = '\0';
  ptr++;
  *vol = ptr0;

  return ptr;

fault_parse:
  name = NULL;
  vol = NULL;
  return NULL;
}

// ======================= Обработка отдельных полей конфигкрации ============================

FlagStatus cfgCalProc( sArxCal * cal, char * str ){
  FlagStatus change = RESET;
  sArxCal cal0;

  while( isspace((int)*str) || ispunct((int)*str) ){
    str++;
  }

  if( *str == '*' ){
    cal0.min = -1;
  }
  else {
    cal0.min = strtol( str, &str, 10 );
  }
  if( cal0.min != cal->min ){
    cal->min = cal0.min;
    change = SET;
  }
  while( isspace((int)*str) ){
    str++;
  }
  if( *str == '*' ){
    cal0.hour = -1;
  }
  else {
    cal0.hour = strtol( str, &str, 10 );
  }
  if( cal->hour != cal0.hour ){
    cal->hour = cal0.hour;
    change = SET;
  }
  while( isspace((int)*str) ){
    str++;
  }
  // Дни
  for( uint8_t i = 0; (isspace((int)*str) == 0) && (i < ARRAY_SIZE(cal->day)); i++ ){
    if( *str == '*' ){
      cal0.day[0] = -1;
      // Каждый день - дальнейшая обработка списка дней бесполезна
      i = ARRAY_SIZE(cal->day);
    }
    else {
      cal0.day[i] = strtol( str, &str, 10 );
    }

    if( cal0.day[i] != cal->day[i] ){
      cal->day[i] = cal0.day[i];
      change = SET;
    }
    while( ispunct((int)*str) ){
      str++;
    }
  }
  if( isspace((int)*str) == 0 ){
    // Ошибка формата - выходим
    Error_Handler( NON_STOP );
    return RESET;
  }

  return change;
}


// Обработка конфигурации SIM_PIN
FlagStatus cfgPinProc( sSimCfg sim[], char * str ){
  FlagStatus change = RESET;
  uint16_t p;

  while( isspace((int)*str) || ispunct((int)*str) ){
    str++;
  }

  p = atol( str );
  if( sim[0].pin != p ){
    sim[0].pin = p;
    change = SET;
  }
  while( isdigit((int)*str) ){
    str++;
  }
  while( ispunct((int)*str) ){
    str++;
  }

  p = atol( str );
  if( sim[1].pin != p ){
    sim[1].pin = p;
    change = SET;
  }

  return change;
}


// Обработка поля конфигурации с логическим значением
FlagStatus cfgBoolFieldProc( FlagStatus * field0, FlagStatus * field1, char * str ){
  FlagStatus change = RESET;
  uint8_t v;

  while( isspace((int)*str) || ispunct((int)*str) ){
    str++;
  }

  v = -1;
  if( (memcmp( str, "true", 4) == 0)
      || (memcmp( str, "TRUE", 4) == 0) ){
    v = 1;
  }
  else if( (memcmp( str, "false", 5) == 0)
      || (memcmp( str, "FALSE", 5) == 0) ){
    v = 0;
  }
  if( v <= 1 ){
    if( *field0 != v ){
      *field0 = v;
      change = SET;
    }
  }

  if( field1 == NULL ){
    return change;
  }

  while( isspace((int)*str) || ispunct((int)*str) ){
    str++;
  }

  v = -1;
  if( (memcmp( str, "true", 4) == 0)
      || (memcmp( str, "TRUE", 4) == 0) ){
    v = 1;
  }
  else if( (memcmp( str, "false", 5) == 0)
      || (memcmp( str, "FALSE", 5) == 0) ){
    v = 0;
  }
  if( v <= 1 ){
    if( *field1 != v ){
      *field1 = v;
      change = SET;
    }
  }

  return change;
}


// Обработка числового поля конфигурации
FlagStatus cfgNumFieldProc( int * field0, int * field1, char * str ){
  FlagStatus change = RESET;
  int v;

  while( isspace((int)*str) || ispunct((int)*str) ){
    str++;
  }

  v = atol( str );
  if( *field0 != v ){
    *field0 = v;
    change = SET;
  }

  if( field1 == NULL ){
    return change;
  }

  while( isdigit((int)*str) ){
    str++;
  }
  while( isspace((int)*str) || ispunct((int)*str) ){
    str++;
  }

  v = atol( str );
  if( *field1 != v ){
    *field1 = v;
    change = SET;
  }

  return change;
}


// Обработка конфигурации GPRS_NAME
FlagStatus cfgStrFieldProc( char * field0, char * field1, char * str ){
  FlagStatus change = RESET;
  uint8_t i;

  while( isspace((int)*str) || (*str == '"') ){
    str++;
  }

  for( i = 0; (i < 32) && (*str != '"') && (!isspace((int)*str)); i++ ){
    field0[i] = *str++;
  }
  field0[i] = '\0';

  if( field1 == NULL ){
    return change;
  }

  while( isspace((int)*str) || ispunct((int)*str) ){
    str++;
  }

  for( i = 0; (i < 32) && (*str != '"') && (!isspace((int)*str)); i++ ){
    field1[i] = *str++;
  }
  field1[i] = '\0';

  return change;
}


// ======================================================================
// Обработка принятой конфигурации
void uspdCfgProc( sUartRxHandle * rxh, SIM800_t * sim ){
  char * bch;                     // Начало обрабатываемого участка строки данных
//  char * ech;                     // Конец обрабатываемого участка строки данных
  FlagStatus change = RESET;

  bch = (char*)(rxh->rxFrame + sim->mqttReceive.payOffset);
  // Читаем версию обновления

  if( (bch = skipch(bch, '\"')) == NULL){
    mqttMsgReset( rxh, &SIM800 );
    return;
  }

  while( *bch != '\0' ){
    char * top;
    char * vol;

    bch = volParse( bch, &top, &vol );
    if( strcmp( top, "i") == 0 ){
      int v[6];
      sscanf( vol, "%d,%d,%d,%d,%d,%d", &(v[0]), &(v[1]), &(v[2]), &(v[3]), &(v[4]), &(v[5]));
      for( uint8_t i = 0; i < ISENS_NUM; i++ ){
        if( v[i] != uspdCfg.isensType[i] ){
          uspdCfg.isensType[i] = v[i];
          change = SET;
        }
      }
    }
    else if( strcmp( top, "o") == 0 ){
      uint8_t v = *vol - '0';

      if( uspdCfg.outState != v ){
        uspdCfg.outState = v;
        change = SET;
      }
    }
    else if( strcmp( top, "arxtime") == 0 ){
      change |= cfgNumFieldProc( (int*)&(uspdCfg.arxTout), NULL, vol );
    }
    else if( strcmp( top, "arxupl") == 0 ){
      change |= cfgBoolFieldProc( &(uspdCfg.arxSend), NULL, vol );
    }
    else if( strcmp( top, "arxsend") == 0 ){
      change |= cfgCalProc( &(uspdCfg.arxCalend), vol );
    }
    else if( strcmp( top, "autonamur") == 0 ){
      change |= cfgBoolFieldProc( &(uspdCfg.autonamur), NULL, vol );
    }
    else if( strcmp( top, "sim" ) == 0 ){
      change |= cfgNumFieldProc( (int*)&(uspdCfg.simSel), NULL, vol );
    }
    else if( strcmp( top, "gprs" ) == 0 ){
      change |= cfgNumFieldProc( (int*)&(uspdCfg.gprsClass), NULL, vol );
    }
    else if( strcmp( top, "regtime" ) == 0 ){
      change |= cfgNumFieldProc( (int*)&(uspdCfg.gprsConnTout), NULL, vol );
    }
    else if( strcmp( top, "pin" ) == 0 ){
      change |= cfgPinProc( uspdCfg.simcfg, vol );
    }
    else if( strcmp( top, "syspin" ) == 0 ){
      change |= cfgBoolFieldProc( &(uspdCfg.simcfg[0].pinAuto), &(uspdCfg.simcfg[1].pinAuto), vol );
    }
    else if( strcmp( top, "op" ) == 0 ){
      change |= cfgNumFieldProc( (int*)&(uspdCfg.simcfg[0].plmn), (int*)&(uspdCfg.simcfg[1].plmn), vol );
    }
    else if( strcmp( top, "gprsnm" ) == 0 ){
      change |= cfgStrFieldProc( uspdCfg.simcfg[0].gprsUser, uspdCfg.simcfg[1].gprsUser, vol );
    }
    else if( strcmp( top, "gprsps" ) == 0 ){
      change |= cfgStrFieldProc( uspdCfg.simcfg[0].gprsPass, uspdCfg.simcfg[1].gprsPass, vol );
    }
    else if( strcmp( top, "gprsapn" ) == 0 ){
      change |= cfgStrFieldProc( uspdCfg.simcfg[0].gprsApn, uspdCfg.simcfg[1].gprsApn, vol );
    }
    else if( strcmp( top, "simctl" ) == 0 ){
      change |= cfgBoolFieldProc( &(uspdCfg.simcfg[0].simActiv), &(uspdCfg.simcfg[1].simActiv), vol );
    }
    else if( strcmp( top, "simctlday" ) == 0 ){
      change |= cfgNumFieldProc( (int*)&(uspdCfg.simcfg[0].simActivTout), \
                                (int*)&(uspdCfg.simcfg[1].simActivTout), \
                                vol );
    }
    else if( strcmp( top, "simctlnum" ) == 0 ){
      change |= cfgNumFieldProc( (int*)&(uspdCfg.simcfg[0].simActivMax), \
                                (int*)&(uspdCfg.simcfg[1].simActivMax), \
                                vol );
    }
    else if( strcmp( top, "mqad" ) == 0 ){
      change |= cfgStrFieldProc( uspdCfg.mqttHost, NULL, vol );
    }
    else if( strcmp( top, "mqpr" ) == 0 ){
      change |= cfgNumFieldProc( (int*)&(uspdCfg.mqttPort), NULL, vol );
    }
    else if( strcmp( top, "mqnm" ) == 0 ){
      change |= cfgStrFieldProc( uspdCfg.mqttUser, NULL, vol );
    }
    else if( strcmp( top, "mqps" ) == 0 ){
      change |= cfgStrFieldProc( uspdCfg.mqttPass, NULL, vol );
    }
    else if( strcmp( top, "232" ) == 0 ){
      change |= cfgBoolFieldProc( &(uspdCfg.termGate), NULL, vol );
    }
    else {
    }

    if( (bch = skipch(bch, '\"')) == NULL){
      break;
    }
  }

  // Обновляем конфигурацию устройства
  cfgUpdate( change );

  mqttMsgReset( rxh, &SIM800 );

  return;
}


// Обновление конфигурации
void cfgUpdate( FlagStatus change ){
  if( change || (uspdCfg.updateFlag == RESET) ){
    // Записать в EEPROM
    uspdCfg.updateFlag = SET;
    if( stmEeWrite( USPD_CFG_ADDR, (uint32_t *)&(uspdCfg), sizeof(uspdCfg) ) != HAL_OK){
      // Ошибка при записи в EEPROM
      uspdCfg.updateFlag = RESET;
      stmEeWrite( USPD_CFG_ADDR, (uint32_t *)&(uspdCfg), sizeof(uspdCfg.updateFlag) );
      return;
    }
  }
  // Отмечаем
  SIM800.mqttClient.pubFlags.cfgoPub = SET;
  // Сбрасываем флаг до отправки подтверждения
  uspdCfg.updateFlag = RESET;
  timerMod( &mqttPubTimer, 0 );
}

void uspdInit( void ){
  SIM800.mqttServer.tcpconn = RESET;
  SIM800.mqttServer.mqttconn = RESET;
//    char str[32] = {0};

  // MQQT settings
  SIM800.sim.ready = SIM_NOT_READY;
  SIM800.sim.pin = (uspdCfg.simcfg[0].pinAuto)? UID_0 % 10000 : uspdCfg.simcfg[0].pin;
  SIM800.sim.apn = uspdCfg.simcfg[0].gprsApn;
  SIM800.sim.apn_user = uspdCfg.simcfg[0].gprsUser;
  SIM800.sim.apn_pass = uspdCfg.simcfg[0].gprsPass;
  SIM800.mqttServer.host = uspdCfg.mqttHost;
  SIM800.mqttServer.port = &(uspdCfg.mqttPort);
  SIM800.mqttReceive.pktIdo = 0x0001;
  SIM800.mqttReceive.mqttData = simHnd.rxh->rxFrame;
  SIM800.mqttClient.username = uspdCfg.mqttUser;
  SIM800.mqttClient.pass = uspdCfg.mqttPass;
  SIM800.mqttClient.clientID = "";
  SIM800.mqttClient.keepAliveInterval = 60;

  announcePktId = -1;
}

