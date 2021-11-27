/*
 * gsm.h
 *
 *  Created on: 3 окт. 2021 г.
 *      Author: jet
 */

#ifndef GSM_H_
#define GSM_H_

#include "stm32l1xx.h"

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
  SIM_PIN_READY,
  SIM_GSM_READY
} eSimReady;

typedef enum {
  PHASE_NON,
  PHASE_ON,
  PHASE_ON_OK,
  PHASE_OFF,
  PHASE_OFF_OK,
} eGsmRunPhase;

typedef struct {
    sim_t sim;
    mqttServer_t mqttServer;
    mqttClient_t mqttClient;
    mqttReceive_t mqttReceive;
} SIM800_t;

extern FlagStatus gsmRun;
extern eGsmState gsmState;
extern FlagStatus gsmFinal;

void gsmProcess( void );

#endif /* GSM_H_ */
