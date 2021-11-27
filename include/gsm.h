/*
 * gsm.h
 *
 *  Created on: 3 окт. 2021 г.
 *      Author: jet
 */

#ifndef GSM_H_
#define GSM_H_

#include "stm32l1xx.h"
#include "pos.h"
#include "uart.h"

typedef enum {
  GSM_OFF,
  GSM_SIM_ON,
  GSM_IFACE_INIT,
  GSM_GPRS_CONN,
  GSM_NTP_INIT,
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


int _disconnect(uint8_t rfOff);

int gsmCmdMode( void );
int gsmSendCommand(char *command, char *reply, uint16_t delay, void (*simreplycb)( sUartRxHandle *) );
void gsmProcess( void );

#endif /* GSM_H_ */
