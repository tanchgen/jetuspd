/*
 * gsm.h
 *
 *  Created on: 3 окт. 2021 г.
 *      Author: jet
 */

#ifndef GSM_H_
#define GSM_H_

#include "stm32l1xx.h"

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
  PHASE_NON,
  PHASE_ON,
  PHASE_ON_OK,
  PHASE_OFF,
  PHASE_OFF_OK,
} eGsmRunPhase;

extern FlagStatus gsmRun;
extern eGsmState gsmState;
extern FlagStatus gsmFinal;

void gsmProcess( void );

#endif /* GSM_H_ */
