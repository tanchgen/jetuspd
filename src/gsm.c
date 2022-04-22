/*
 * gsm.c
 *
 *  Created on: 3 окт. 2021 г.
 *      Author: jet
 */
#include <stdio.h>
#include <string.h>

#include "my_ntp.h"
#include "mqtt.h"
#include "logger.h"
#include "lowpwr.h"
#include "flash.h"
#include "usart_arch.h"
#include "gpio_arch.h"
#include "uspd.h"
#include "events.h"
#include "gsm.h"

extern SIM800_t SIM800;
//extern const uint32_t baudrate[BAUD_NUM];
//extern struct timer_list tArchPubTimer;

static uint32_t tmpTick;

static uint8_t gsmToutCount = 0;

eGsmState gsmState = GSM_OFF;

FlagStatus gsmRun = SET;
eGsmRunPhase gsmRunPhase = PHASE_OFF_OK;

eGsmState gsmStRestart = GSM_OFF;
eGsmReset gsmReset = SIM_RESET;

FlagStatus ntpFlag;

uint8_t reconnCount = 0;
uint16_t gprsConnTout[] = {
  2000,
  3000,
  3000,
  3000
};

// Время засыпания при ошибке установки связи SIM
static uint32_t simFaultSleep[] = {
  HOUR_1,
  HOUR_2,
  HOUR_1 * 4,
  HOUR_1 * 8,
  HOUR_24,
  HOUR_24,
  HOUR_24,
  HOUR_24,
  HOUR_24,
  HOUR_24
};
static uint8_t simSleepCount;

// Таймер таймаута процесса включения GSM_PROC
struct timer_list gsmOnToutTimer;

// Таймер таймаута процесса включения GSM_PROC
struct timer_list bigOnToutTimer;

// Таймер усыпления контроллера при включении GSM_PROC
struct timer_list tGsmOnSleepTimer;

// -------------- Function prototype ------------------------------------------
void simUartBaud( uint32_t baudrate );
void simUartHwFlow( void );

int mqttSubProcess(void);

int simWaitReady( void );
int gprsConnTest( void );
int simUartStart(void);
int gprsConnTest( void );
int gprsConnBreak( void );
int gprsConn( void );
int ntpInit(void);
int TCP_Connect(void);
int clkSet( void );
void ip4Parse( char * str );
void isensTimCorr( void );

int8_t flashArchFill( void );
// ----------------------------------------------------------------------------

// Выполняется при просыпании по будильнику
void rtcAlrmCb( void ){
  if( sleepFlag ){
    simSleepCount = 0;
  }
  ssleep = 0;
  trace_printf("t:%d\n", getRtcTime() );
}

static void gsmOnSleep( uintptr_t arg ){
// Пока не придумал - зачем
  (void)arg;
}

/**
 * Сброс таймера таймаута процесса включения/подключения GSM/GPRS/NTP/TCP/MQTT
 * @param non
 * @return non
 */
static void gsmToutTimDel( void ){
  // Сбрасываем счетчик срабатываний таймера таймаута
  gsmToutCount = 0;
  timerDel( &gsmOnToutTimer );
}


/**
 * Обработка "Большого" таймаута: Успеть подключится и опубликоваать анонс
 * @param arg состояние GSM-машины
 * @return non
 */
static void bigOnTout( uintptr_t arg ){
  (void)arg;

  trace_printf( "BIG_Tout\n" );
  // XXX: !!!
//  return;

  if( simSleepCount > uspdCfg.simcfg[0].simActivMax ){
    simSleepCount = 0;
    // Засыпаем до следующего включения по календарю
    gsmStRestart = GSM_OFF;
    toSleep( SET );
  }
  else {
    // Переносим публикацию сенсоров если это раньше следующей попытки
    if( timerPending( &tArchPubTimer ) ){
      tUxTime ut = getRtcTime();
      if( (tArchPubTimer.expires) < (ut + simFaultSleep[ simSleepCount++]) ){
        rtcTimMod( &tArchPubTimer, simFaultSleep[ simSleepCount++] + 1 );
      }
    }
    // Засыпаем на 1, 2, 4, 24 часа до следующей попытки
    gsmSleep( simFaultSleep[ simSleepCount++], SET );
  }
  gsmReset = MCU_SLEEP;
  gsmRun = RESET;
}


/**
 * Обработка таймаута процесса включения/подключения GSM/GPRS/NTP/TCP/MQTT
 * @param arg состояние GSM-машины
 * @return non
 */
static void gsmOnTout( uintptr_t arg ){
  eGsmState gsmst = (eGsmState)arg;

  trace_printf( "gsmTout %d\n", gsmst );

  // XXX: !!!
  return;


  switch( gsmst){
    case GSM_INIT:
      evntFlags.gsmFault = SET;
      goto restart;
      break;
    case GSM_START_INIT:
      evntFlags.gprsFault = SET;
      goto restart;
      break;
    case GSM_GPRS_CONN:
      evntFlags.ntpFault = SET;
      gsmToutCount = 0;
      break;
    case GSM_NTP_INIT:
      break;
    case GSM_MQTT_START:
      if( SIM800.mqttServer.tcpconn == 0 ){
        evntFlags.tcpFault = SET;
      }
      else if( SIM800.mqttServer.mqttconn == 0 ){
        evntFlags.mqttFault = SET;
      }
      goto restart;
      break;
    case GSM_MQTT_CONN:
      break;
//    case GSM_SERV_CONN:
//      break;
//    case GSM_CFG_ON:
//      break;
    case GSM_WORK:
      break;
    case GSM_OFF:
    case GSM_SIM_ON:
    default:
      break;

  }
  return;

restart:
  ledOff( LED_G, 0);
  // Красный светодиод - пока запускается
  ledToggleSet( LED_R, LED_BLINK_ON_TOUT, LED_BLINK_OFF_TOUT, 0, 0 );

  gsmStRestart = GSM_OFF;
  gsmRun = RESET;
  return;
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

  simHnd.rxh->reply = reply;
  simHnd.rxh->replyFlag = RESET;
//  trace_write( command, strlen(command) );
  if( uartSend(simHnd.txh, (uint8_t*)command, (uint16_t)strlen(command) ) == 0 ){
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

// Ввод/смена PIN-кода
int simPinEnter( char * pin, char * newpin ){
  char * str;

  if( pin == NULL ){
    return -1;
  }

  if( (str = malloc( 20 )) == NULL ){
    return -1;
  }

//  trace_printf( "a_buf_%x\n", str );

  memcpy( str, "AT+CPIN=", 8 );
  memcpy(str+8, pin, 4);

  if( newpin != NULL ){
    // Добавляем новый PIN-код
    str[12] = ',';
    memcpy( str+13, newpin, 4 );
    memcpy( str+17, "\r\n\0", 3 );
  }
  else {
    memcpy( str+12, "\r\n\0", 3 );
  }

  if( gsmSendCommand( str, "OK\r\n", CMD_DELAY_5, NULL ) == 0 ){
    return 0;
  }

  return -1;
}


// Процесс готовности SIM-карты
int simReadyProcess( void ){
  switch( SIM800.ready ){
    case SIM_NOT_READY:
    case SIM_PON:
      // Отправка команды CPIN
      simHnd.rxh->replyBuf = mqtt_buffer;
      *mqtt_buffer = '\0';
      if( gsmSendCommand("AT+CPIN?\r\n", "+CPIN:", CMD_DELAY_5, saveSimReply ) == 0 ){
        if( strstr( mqtt_buffer + 7, "READY" ) != NULL ){
          // Пин не нужен
          SIM800.ready = SIM_PIN_READY;
        }
        else if( strstr( mqtt_buffer + 5, "SIM_PIN" ) != NULL ){
          char str[5];
          simPinEnter( utoa( SIM800.sim.pin, str, 10 ), NULL );
        }
      }
      else {
        return -1;
      }
      break;
    case SIM_PIN_READY:
      simHnd.rxh->replyBuf = mqtt_buffer;
      *mqtt_buffer = '\0';
      if( gsmSendCommand("AT+CREG?\r\n", "+CREG:", CMD_DELAY_10, saveSimReply ) == 0 ){
        if( mqtt_buffer[9] == '1' ){
          SIM800.ready = SIM_GSM_READY;
        }
//        else if( mqtt_buffer[9] == '3' ){
//          gsmStRestart = GSM_OFF;
//          gsmRun = RESET;
//        }
        else {
          return -1;
        }
      }
      break;
    case SIM_GSM_READY:
    default:
      break;
  }

  return 0;
}


// Определяем урвень сигнала
int simImeiProcess( void ){
  static char * num = "0";
  simHnd.rxh->replyBuf = SIM800.sim.imei;
  *mqtt_buffer = '\0';
  if( num[0] <= '9' ){
    if( gsmSendCommand("AT+GSN\r\n", num, CMD_DELAY_5, saveImeiReply ) == 0){
      return 0;
    }
    else {
      return -1;
    }
  }
  else {
    ErrHandler( STOP );
  }

  return 0;
}


// Определяем урвень сигнала
int simCsqProcess( void ){
  simHnd.rxh->replyBuf = mqtt_buffer;
  *mqtt_buffer = '\0';
  if( gsmSendCommand("AT+CSQ\r\n", "+CSQ:", CMD_DELAY_5, saveSimReply ) == 0){
    SIM800.sim.csq = strtol( &(mqtt_buffer[6]), NULL, 10 );
    return 0;
  }
  else {
    return -1;
  }
}


int simWaitReady( void ){
  if( SIM800.ready == SIM_GSM_READY ){
    clearRxBuffer( (char *)(simHnd.rxh->rxFrame), &(simHnd.rxh->frame_offset) );
    return RESET;
  }

  return SET;
}



/**
 * initialization SNTP.
 * @param NONE
 * @return error status, 0 - OK
 */
int clkSet( void ) {
//  uint8_t errCount;

//  for( errCount = 0; errCount < 4; errCount++ ){
    simHnd.rxh->replyBuf = mqtt_buffer;
    *mqtt_buffer = '\0';
    if( gsmSendCommand("AT+CCLK?\r\n", "+CCLK: \"", CMD_DELAY_30, saveSimReply ) == 0){
      int8_t tz;

      // Получили дату-время
      sscanf(mqtt_buffer, "+CCLK: \"%d/%d/%d,%d:%d:%d%d\"", \
                        (int*)&rtc.year, (int*)&rtc.month, (int*)&rtc.date, \
                         (int*)&rtc.hour, (int*)&rtc.min, (int*)&rtc.sec, (int *)&tz );
//      tz /= 4;
//      // Переходим в Локальное время
//      uxTime = xTm2Utime( &rtc ) + tz * 3600;
      // Уже в Локальном времени
      uxTime = xTm2Utime( &rtc );
      setRtcTime( uxTime );
      // Переустанавливаем RTC-таймеры
      isensTimCorr();
      return RESET;
    }
//    else {
//      if( errCount >= 2) {
//        // Никак не получается включить GPRS
//        gsmSendCommand("AT+CCLK\r\n", "OK\r\n", CMD_DELAY_50, NULL );
//        wutSleep( TOUT_3000 * 1e3 );
//      }
//    }
//  }

  return SET;
}


// SIM800 reply callback: Ожидание начала работы SIM800 после включения
void ponSimReply( sUartRxHandle * handle ){
  SIM800.ready = SIM_PON;
  clearRxBuffer( (char *)(handle->rxFrame), &(handle->frame_offset) );
}


void simWaitPonInit( void ){
  simHnd.rxh->reply = "RDY\r\n";
  simHnd.rxh->replyCb = ponSimReply;
}


/**
 * initialization SIM800.
 * @param NONE
 * @return error status, 0 - OK
 */
int simUartStart(void) {
    int error = SET;
    uint8_t cnt = 0;

    while( gsmSendCommand("AT\r\n", "OK\r\n", CMD_DELAY_2, NULL ) != 0 ){
      wutSleep( TOUT_300 * 1e3);
    }

    if( gsmSendCommand("AT+IFC=2,2\r\n", "OK\r\n", CMD_DELAY_2, NULL ) == 0){
      simUartHwFlow();
    }
    if( gsmSendCommand("AT+IPR=115200\r\n", "OK\r\n", CMD_DELAY_2, NULL) == 0){
      simUartBaud(115200);
    }

    while( gsmSendCommand("AT\r\n", "OK\r\n", CMD_DELAY_10, NULL ) != 0 ){
      if( ++cnt > 5){
        ErrHandler( STOP );
      }
      else {
        wutSleep( TOUT_300 * 1e3);
      }
    }

    gsmSendCommand("ATE1\r\n", "OK\r\n", CMD_DELAY_2, NULL );

    return error;
}


int gprsConnTest( void ){
  int rc = -1;
  simHnd.rxh->replyBuf = mqtt_buffer;
  *mqtt_buffer = '\0';

  if( gsmSendCommand("AT+SAPBR=2,1\r\n", "+SAPBR:", CMD_DELAY_5, saveSimReply ) == 0){
    rc = mqtt_buffer[10];
  }

  return rc;
}


int gprsConnBreak( void ){
  if( SIM800.mqttServer.tcpconn ){
    return MQTT_Deinit();
  }
  else {
    return gsmSendCommand("ATE1\r\n", "OK\r\n", CMD_DELAY_2, NULL );
  }
}


int gprsConn( void ){
  char * str;

  gsmSendCommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n", "OK\r\n", CMD_DELAY_5, NULL );
  if((str = malloc( 256 )) == NULL ){
    ErrHandler( NON_STOP );
  }
  else {
    sprintf(str, "AT+SAPBR=3,1,\"APN\",\"%s\"\r\n", SIM800.sim.apn);
//    trace_printf( "a_buf_%x\n", str );
    gsmSendCommand(str, "OK\r\n", CMD_DELAY_5, NULL);
  }

  if( gsmSendCommand("AT+SAPBR=1,1\r\n", "OK\r\n", CMD_DELAY_50, NULL) == 0){
    // Есть соединение GPRS;
  }
  return 0;
}


int ntpInit(void) {
  if( ntpFlag == RESET ){

    if( gsmSendCommand("AT+CNTPCID=1\r\n", "OK\r\n", CMD_DELAY_5, NULL) ){
      return ntpFlag;
    }
    while( ntpFlag == RESET ){

      if( gsmSendCommand("AT+CNTP=\""NTP_SERVER"\",12\r\n", "OK\r\n", CMD_DELAY_5, NULL) == 0){;
        if( gsmSendCommand("AT+CNTP\r\n", "+CNTP: 1\r\n", CMD_DELAY_50 * 2, NULL) == 0 ){
          ntpFlag = SET;
        }
        else {
          ntpFlag = SET;
          ErrHandler( NON_STOP );
        }
      }
    }
  }

  return ntpFlag;
}

int gsmWorkProc( void ){
  uint8_t rc = 0;
  switch( uspd.runMode ){
    case RUN_MODE_FIRST:
      if( uspdCfg.updateFlag ){
        gsmStRestart = GSM_OFF;
      }
      else {
        // Просто ждем новую конфигурацию USPD...
        rc = 1;
      }
      break;
    case RUN_MODE_KEY:
      // TODO: Организовать ожидание, прием, выполнение команд и последующее выключение
      break;
    case RUN_MODE_SENS_SEND:
      // Проверка на "Опубликовали все!"
      if( (uspd.readArchEvntQuery == RESET)
          && SIM800.mqttClient.pubFlags.archPubEnd ){
        // Засыпаем до следующего включения по календарю
        gsmStRestart = GSM_OFF;
        trace_puts( "SENS send sleep ");
        assert_param( flashArchFill() == 0 );
        toSleep( SET );
      }
      else {
        rc = 1;
      }
      break;
    default:
      rc = 1;
      break;
  }

  return rc;
}

// ----------------  GSM PROCCESS FUNCTIONS -----------------------------------
// Включение питания SIM800
void gsmOffFunc( void ){
  if( gsmRun ){
    switch( gsmRunPhase ){
      case PHASE_NON:
#if DEBUG_GSM_TRACE
  trace_puts("SIM_On");
#endif

        if( timerPending( &sb1Timer ) ){
          return;
        }

        timerMod( &bigOnToutTimer, SEC_180 * 1e3 );
        simUartBaud(9600);
        uspdCfgInit( &uspd.defCfgFlag );
        // On SIM800 power if use
        gpioPinResetNow( &gpioPinSimPwr );
        gsmRunPhase = PHASE_ON;
        wutSleep( 500 * 1e3 );
        break;
      case PHASE_ON:
        gpioPinSetNow( &gpioPinPwrKey );
        wutSleep( 1200 * 1e3 );
        gsmRunPhase = PHASE_ON_OK;
        break;
      case PHASE_ON_OK:
        // Две вспышки красного цвета с интервалом в 3 сек
        gpioPinResetNow( &gpioPinPwrKey );
        ledToggleSet( LED_R, LED_BLINK_ON_TOUT, LED_TOGGLE_TOUT, 2, TOUT_3000);
        gsmState++;
        gsmRunPhase = PHASE_NON;
        wutSleep( 1500 * 1e3 );
        break;
      default:
        break;
    }
  }
  else {
    // Выключаем питание GSM
    switch( gsmRunPhase ){
      case PHASE_NON:
#if DEBUG_GSM_TRACE
  trace_puts("SIM_Off");
#endif

        gpioPinSetNow( &gpioPinPwrKey );
        wutSleep( 1700 * 1e3 );
        gsmRunPhase = PHASE_OFF;
        break;
      case PHASE_OFF:
        gpioPinResetNow( &gpioPinPwrKey );
        wutSleep( 2000 * 1e3 );
        gsmRunPhase = PHASE_OFF_OK;
        break;
      case PHASE_OFF_OK:
        switch( gsmReset ){
          case MCU_RESET:
            gpioPinSetNow( &gpioPinSimPwr );
            if( (logRdBufFill == 0) && (flashDev.state == FLASH_READY) ){
              // Все записи в журнал сделаны, операции с флеш закончены
              NVIC_SystemReset();
            }
            break;
          case MCU_SLEEP:
            gpioPinSetNow( &gpioPinSimPwr );
            if( flashDev.state == FLASH_READY ){
              // Все записи в журнал сделаны, операции с флеш закончены
              timerDel( &gsmOnToutTimer );
              timerDel( &bigOnToutTimer );
              gsmReset = SIM_RESET;
              toSleep( RESET );
              uspd.runMode = RUN_MODE_NULL;
            }
            break;
          case SIM_RESET:
            // Проснулись после сна - RESET по питанию SIM800
            gsmRun = SET;
            gsmRunPhase = PHASE_NON;
            break;
          default:
            break;
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
        // Настраиваем ожидание посылки от SIM по UART
        simWaitPonInit();
        tmpTick = mTick + 10000;
        gsmRunPhase = PHASE_ON;
        break;
      case PHASE_ON:
        if( SIM800.ready == SIM_PON ){
          gsmRunPhase = PHASE_ON_OK;
        }
        else if( tmpTick < mTick ){
          gsmRunPhase = PHASE_ON_OK;
        }
        break;
      case PHASE_ON_OK:
        gsmRunPhase = PHASE_NON;
        ntpFlag = RESET;
        gsmState++;
        break;
      default:
        break;
    }
  }
  else {
    // Выключаем питание GSM
    trace_puts("sim->off");
    gsmState--;
    gsmRunPhase = PHASE_NON;
  }
}


// Состояние "GSM PWR ON": Установка сохраненной конфигурации
void gsmInitFunc( void ){
  if( gsmRun ){
    switch( gsmRunPhase ){
      case PHASE_NON:
        timerModArg( &gsmOnToutTimer, SEC_30*1e3, gsmState );
        simUartStart();
        timerModArg( &gsmOnToutTimer, SEC_30*1e3, gsmState );
        gsmRunPhase = PHASE_ON;
        tmpTick = 0;
//        gsmSleep(SEC_30);
        break;
      case PHASE_ON:
        if( SIM800.ready != SIM_GSM_READY ){
          tmpTick = ( simReadyProcess() != 0 )? 500 : 1000;
        }
        else {
          if( simImeiProcess() != 0 ){
            tmpTick = 30;
          }
          else {
            gsmRunPhase = PHASE_ON_OK;
            tmpTick = 0;
#if DEBUG_GSM_TRACE
            trace_puts("Sim_OK");
#endif
          }
        }
        if( tmpTick ){
          wutSleep( tmpTick * 1e3 );
        }
        break;
      case PHASE_ON_OK:
        if( simCsqProcess() != 0 ){
          wutSleep( 100 * 1e3 );
        }
        else {
#if DEBUG_GSM_TRACE
          trace_printf("CSQ: %d\n", SIM800.sim.csq);
#endif
          gsmToutTimDel();
          tmpTick = 0;
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
    if( gsmStRestart == GSM_INIT ){
      gsmRun = SET;
      gsmRunPhase = PHASE_NON;
    }
    else {
      trace_puts("init->sim");
      gsmState--;
    }
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
//            ErrHandler( NON_STOP );
            wutSleep( TOUT_50 * 1e3 );
//            rtcTimModArg( &tGsmOnToutTimer, SEC_30, gsmState );
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
#if DEBUG_GSM_TRACE
        trace_puts("GPRS conn");
#endif
        gsmToutTimDel();
        // TODO: Получение IP
        ip4Parse( mqtt_buffer );

        assert_param( SIM800.mqttServer.tcpconn == RESET );

        // Две вспышки оранжевого цвета с интервалом в 3 сек
        ledToggleSet( LED_G, LED_BLINK_ON_TOUT, LED_SLOW_TOGGLE_TOUT, 2, TOUT_3000);
        ledToggleSet( LED_R, LED_BLINK_ON_TOUT, LED_SLOW_TOGGLE_TOUT, 2, TOUT_3000);
        gsmRunPhase = PHASE_NON;
        gsmState++;
        break;
      default:
        break;
    }
  }
  else {
    // Выключаем питание GSM
    if( gsmStRestart == GSM_START_INIT ){
      // Пробуем переподключить
#if DEBUG_GSM_TRACE
      trace_puts("GSM restart");
#endif
      if( gsmRunPhase == PHASE_NON) {
        tmpTick = 0;
        if( gprsConnBreak() == 0) {
          gsmRun = SET;
          gsmRunPhase = PHASE_NON;
        }
        else {
          gsmRunPhase = PHASE_OFF;
          wutSleep( TOUT_1000 * 1e3 );
        }
      }
      else {
        gsmRunPhase = PHASE_NON;
        gsmState--;
      }
    }
    else {
      // Продолжаем выключать
      gprsConnBreak();
      trace_puts("start->init");
      gsmState--;
    }


  }
}


// Состояние "GSM PWR ON": Установка сохраненной конфигурации
void gsmGprsConnFunc( void ){
  if( gsmRun ){
    switch( gsmRunPhase ){
      case PHASE_NON:
        timerModArg( &gsmOnToutTimer, SEC_30*1e3, gsmState );
        ntpInit();
        gsmRunPhase = PHASE_ON;
        break;
      case PHASE_ON:
        if( clkSet() == RESET ){
          // Время установлено - включаем интефейс Терминала и отправляем время
#if DEBUG_GSM_TRACE
          trace_puts("Time set");
#endif
#if TERM_UART_ENABLE
          gpioPinSetNow( &gpioPinTermOn );
          termSendTime();
#endif //TERM_UART_ENABLE
          gsmToutTimDel();

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
    trace_puts("gprs->start");
    gsmState--;
  }
}


// Состояние "GSM PWR ON": Установка сохраненной конфигурации
void gsmNtpInitFunc( void ){
  if( gsmRun ){
    if( mqttStart() == RESET ){
      gsmRunPhase = PHASE_NON;
      gsmState++;
      // Таймаут для TCP-connect to MQTT-broker
      timerModArg( &gsmOnToutTimer, SEC_20*1e3, gsmState );
      tmpTick = 0;
    }
  }
  else {
    if( tmpTick == 0 ){
      if( SIM800.mqttServer.tcpconn == 0 ){
        gsmRunPhase = PHASE_OFF_OK;
      }
      else if ( SIM800.mqttServer.mqttconn == 0 ){
        gsmRunPhase = PHASE_OFF;
        tmpTick = mTick + 200;
      }
      // Продолжаем выключать
      if( logRdBufFill != 0 ){
        return;
      }
    }
    if( gsmRunPhase == PHASE_NON ){
      char tpc[32];
      char pay[64] = "{\"imei\":";
      uint32_t ut = getRtcTime();

      // Передача RealTime
      memset( tpc, 0, 32);
      memset( pay, 0, 64);
      sprintf( tpc, tpcTempl[TOPIC_INFO], SIM800.sim.imei );
      sprintf( pay, "{time\":%u,\"state\":\"bay\"}", (unsigned int)ut );
      if( MQTT_Pub( tpc, pay, QOS1, SIM800.mqttReceive.pktIdo++ ) != 0 ){
        uspd.byePktId = SIM800.mqttReceive.pktIdo;
        SIM800.mqttReceive.pktIdo++;
        tmpTick = 0;
        gsmRunPhase = PHASE_OFF;
      }
      else {
        return;
      }
    }
    else if( gsmRunPhase == PHASE_OFF ){
      if( tmpTick == 0 ){
        if( uspd.byePktId == (uint16_t)(~0) ){
          MQTT_Disconnect();
        }
        else {
          SIM800.mqttClient.pubFlags.uspdAnnounce = RESET;
        }
        tmpTick = mTick + 1200;
      }
      else if( tmpTick < mTick ){
        gsmSendCommand("+++", "OK\r\n", CMD_DELAY_10 * 2, NULL );
        gsmRunPhase = PHASE_OFF_OK;
        if( SIM800.mqttServer.tcpconn ){
          SIM800.mqttServer.tcpconn = 0;
        }
      }
    }
    else if( gsmRunPhase == PHASE_OFF_OK ){
      if( MQTT_Deinit() == 0){
        trace_puts("ntp->gprs");
        gsmState--;
        gsmRunPhase = PHASE_NON;
        tmpTick = 0;
      }
    }
  }
}


// Состояние "GSM MQTT_START": Установка сохраненной конфигурации
void gsmMqttStartFunc( void ){
  if( gsmRun ){
    switch( gsmRunPhase ){
      case PHASE_NON:
        if( SIM800.mqttServer.tcpconn == 0 ){
          TCP_Connect();
        }
        else if( SIM800.mqttServer.mqttconn == 0 ){
          if( tmpTick == 0 ){
            timerModArg( &gsmOnToutTimer, SEC_60*1e3, gsmState );
          }
          MQTT_Connect();
          tmpTick = mTick + 5000;
          gsmRunPhase = PHASE_ON;
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
//          wutSleep( 1e6 );
          gsmRunPhase = PHASE_NON;
        }
        break;
      case PHASE_ON_OK:
        gsmToutTimDel();
        mqttPubInit();
        gsmState++;
        gsmRunPhase = PHASE_NON;
        gsmStRestart = GSM_MQTT_START;
        break;
      default:
        break;
    }
  }
  else {
    // Выключаем питание GSM
    if( gsmStRestart == GSM_MQTT_START ){
      gsmRun = SET;
      gsmRunPhase = PHASE_NON;
    }
    else {
      trace_puts("mqtt_st->ntp");
      gsmState--;
      tmpTick = 0;
    }
  }
}


// Состояние "MQTT_CONN": Подписка SUBSCIPTION
void gsmMqttConnFunc( void ){
  if( gsmRun ){
    switch( gsmRunPhase ){
      case PHASE_NON:
        if( mqttSubProcess() ){
          break;
        }
        trace_puts( "mqtt sub" );
        SIM800.mqttClient.pubFlags.uspdAnnounce = SET;
        SIM800.mqttClient.pubReady = 0;
        gsmRunPhase = PHASE_ON;
        break;
      case PHASE_ON:
//        mqttSubFlag = SUB_NONE;
//				mqttSubFlag = RESET;
        gsmState++;
        gsmRunPhase = PHASE_NON;
        break;
      default:
        break;
    }
  }
  else {
    // Выключаем питание GSM
    trace_puts("conn->mqtt_st");
    gsmState--;
  }
}


// Состояние "GSM PWR ON": Установка сохраненной конфигурации
//void gsmServConnFunc( void ){
//  if( gsmRun ){
//    if( uspdCfg.updateFlag ){
//      gsmState++;
//      gsmRunPhase = PHASE_NON;
//    }
//  }
//  else {
//    // Выключаем питание GSM
//    gsmState--;
//  }
//}


// Состояние "GSM PWR ON": Установка сохраненной конфигурации
//void gsmCfgOnFunc( void ){
//  if( gsmRun ){
//    gsmState++;
//    gsmRunPhase = PHASE_NON;
//  }
//  else {
//    // Выключаем питание GSM
//    gsmState--;
//  }
//}


// Состояние "GSM PWR ON": Установка сохраненной конфигурации
void gsmWorkFunc( void ){
  if( gsmRun ){
    if( SIM800.mqttServer.tcpconn == 0 ){
      mqttConnectCb( RESET );
    }
    else {
      switch( gsmRunPhase ){
        case PHASE_NON:
          if( gsmWorkProc() == 0 ){
            gsmRunPhase = PHASE_ON;
          }
          break;
        case PHASE_ON:
          // Все сделано - засыпаем до следующего раза
          sensPubAlrmSet( &(uspd.arxCal) );
          gsmReset = MCU_SLEEP;
          gsmRun = RESET;
          break;
        default:
          break;
      }
    }
  }
  else {
    // Выключаем питание GSM
    trace_puts("work->conn");
    gsmState--;
    gsmRunPhase = PHASE_NON;
  }
}



// ----------------------------------------------------------------------------
// Машина состояния break;
void gsmProcess( void ){
  if( uspd.runMode == RUN_MODE_SENS_WRITE ){
    return;
  }

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
//    case GSM_SERV_CONN:
//      gsmServConnFunc();
//      break;
//    case GSM_CFG_ON:
//      gsmCfgOnFunc();
//      break;
    case GSM_WORK:
      gsmWorkFunc();
      break;
    default:
      break;
  }
}


/**
 * Инициализация GSM-процесса.
 * @param NONE
 * @return error status, 0 - OK
 */
void gsmInit(void) {
  // 10 - максимальное количество попыток переподключения SIM при ошибке подключения
  MAYBE_BUILD_BUG_ON( ARRAY_SIZE(simFaultSleep) != 10 );

  timerSetup( &gsmOnToutTimer, gsmOnTout, (uintptr_t)NULL );
  timerSetup( &bigOnToutTimer, bigOnTout, (uintptr_t)NULL );
  rtcTimSetup( &tGsmOnSleepTimer, gsmOnSleep, (uintptr_t)NULL );
}
