/*
 * gsm.h
 *
 *  Created on: 3 окт. 2021 г.
 *      Author: jet
 */

#ifndef GSM_H_
#define GSM_H_

#include "stm32l1xx.h"
//#include "uspd.h"
#include "uart.h"
#include "times.h"
#include "lowpwr.h"

// === CONFIG ===
#define UART_SIM800     &simUart
#define CMD_DELAY_2     200
#define CMD_DELAY_5     500
#define CMD_DELAY_10    1000
#define CMD_DELAY_30    3000
#define CMD_DELAY_50    5000
// ==============

#define IMEI       "000000000000000"

typedef enum {
  GSM_OFF,
  GSM_SIM_ON,
  GSM_INIT,
  GSM_START_INIT,
  GSM_GPRS_CONN,
  GSM_NTP_INIT,
  GSM_MQTT_START,
  GSM_MQTT_CONN,
  GSM_SERV_CONN,
  GSM_CFG_ON,
  GSM_WORK,
} eGsmState;

typedef enum {
  SIM_NOT_READY,
  SIM_PON,
  SIM_PIN_READY,
  SIM_GSM_READY
} eSimReady;

typedef enum {
  PHASE_NON_0,
  PHASE_NON,
  PHASE_ON,
  PHASE_ON_OK,
  PHASE_OFF,
  PHASE_OFF_OK,
} eGsmRunPhase;

typedef enum {
  MCU_RESET,
  MCU_SLEEP,
  SIM_RESET
} eGsmReset;

typedef struct {
    uint16_t pin;
    char *apn;
    char *apn_user;
    char *apn_pass;
    uint8_t csq;
    char imei[16];
    union{
      struct {
        uint8_t ip4_0;
        uint8_t ip4_1;
        uint8_t ip4_2;
        uint8_t ip4_3;
      };
      uint32_t u32ip4;
    };
} sim_t;

extern FlagStatus gsmRun;
extern eGsmState gsmState;
extern eGsmState gsmStRestart;
extern eGsmReset gsmReset;

extern struct timer_list mqttPingTimer;
// Таймер усыпления контроллера при включении GSM_PROC
extern struct timer_list tGsmOnSleepTimer;

int gsmSendCommand(char *command, char *reply, uint16_t delay, void (*simreplycb)( sUartRxHandle *) );

void gsmProcess( void );

static inline void gsmSleep( uint32_t sec, FlagStatus pre ){
  rtcTimMod( &tGsmOnSleepTimer, sec );
  toSleep( pre );
}

#endif /* GSM_H_ */
