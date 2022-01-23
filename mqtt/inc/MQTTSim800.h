/*
 * MQTTSim800.h
 *
 *
 */

#ifndef MQTT_SIM_800_H_
#define MQTT_SIM_800_H_


#include "main.h"
#include "uart.h"
#include "MQTTPacket.h"
//#include "mqtt.h"

void Sim800_RxCallBack(void);
void clearRxBuffer( char * buf, uint32_t * size );
void clearMqttBuffer(void);

int MQTT_Deinit(void);

void saveSimReply( sUartRxHandle * handle );
void saveImeiReply( sUartRxHandle * handle );
void connectSimReply( sUartRxHandle * handle );

int mqttStart(void);
void MQTT_Connect(void);
void MQTT_Disconnect(void);

uint16_t MQTT_Pub(const char *topic, char *payload, enum QoS, uint16_t pktid);

uint16_t MQTT_Puback(  unsigned short packetid );
uint16_t MQTT_Pubrec(  unsigned short packetid );
uint16_t MQTT_Pubrel(  unsigned short packetid );
uint16_t MQTT_Pubcomp(  unsigned short packetid );

void MQTT_PingReq(void);

void MQTT_Sub( char const *topic, uint8_t qos );

void MQTT_Receive(unsigned char *buf);

void mqttProcess( void );

#endif // MQTT_SIM_800_H_
