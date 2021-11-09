/*
 * mqtt.h
 *
 *  Created on: 23. 10. 2013
 *      Author: hp
 */

#ifndef MQTT_H_
#define MQTT_H_

#include <stdint.h>
#include "topic_id.h"
#include "MQTTSim800.h"

//#include "lwip/opt.h"
//#include "lwip/stats.h"
//#include "lwip/sys.h"
//#include "lwip/pbuf.h"
//#include "lwip/udp.h"
//#include "lwip/tcp.h"
//#include "lwip/dns.h"
//#include "lwip/dhcp.h"
//#include "lwip/init.h"
//#include "lwip/netif.h"
//#include "netif/etharp.h"
//#include "netif/loopif.h"


#define MQTTCONNECT 1<<4
#define MQTTPUBLISH 3<<4
#define MQTTSUBSCRIBE 8<<4

#define MAX_PACKET_SIZE 128

#define KEEPALIVE 30000

typedef struct Mqtt sMqtt;

//typedef void (*msgReceived)(sMqtt *this, uint8_t *topic, uint8_t topicLen, uint8_t *data, uint32_t dataLen);

enum QoS {
	QOS0,
	QOS1,
	QOS2
};

struct Mqtt
{
//	struct ip_addr server;
	uint16_t port;
	uint32_t connTout;
	uint8_t connCount;
	uint8_t connected;
	uint8_t autoConnect;
	struct tcp_pcb *pcb;
	uint32_t nextActivity;
//	msgReceived msgReceivedCallback;
	char deviceId[9];
	char username[80];
	char password[42];
	uint8_t * pubTopic;
	uint8_t * subsTopic;
	uint16_t mqttPackId;
	uint8_t subs;							//Флаг созданной подписки.
	uint8_t pubFree;							//Флаг отправленной пудликации.
	uint8_t pollAbortCounter;
};


extern sMqtt mqtt;


typedef struct MqttFixedHeader
{
	uint8_t header;
	uint8_t remainingLength;
} MqttFixedHeader;

#define MQTT_MSGT_PINGREQ	(12 << 4)
#define MQTT_MSGT_PINGRESP	(13 << 4)
#define MQTT_MSGT_PUBLISH	(3 << 4)
#define MQTT_MSGT_CONACK	(2 << 4)
#define MQTT_MSGT_SUBACK	(9 << 4)
#define MQTT_MSGT_PUBACK	(4 << 4)

#define MQTT_PINGREQ_HEADER (MQTT_MSGT_PINGREQ)


//void mqttInit(sMqtt *mqtt, struct ip_addr serverIp, int port, msgReceived fn, char *devId);
//uint8_t mqttConnect(sMqtt *this);
//uint8_t mqttPublish(sMqtt *this,char* pub_topic, char* msg);
//uint8_t mqttDisconnect(sMqtt *this);
//uint8_t mqttSubscribe(sMqtt *this, char* topic);
//uint8_t mqttLive(sMqtt *this);
//void mqttDisconnectForced(sMqtt *this);
//uint8_t mqttTcpConnect(sMqtt *this);
//uint8_t mqttBrokConnect(sMqtt *this);

static inline void mqttBufClean( sUartRxHandle *handle, SIM800_t * sim ){
  // Очистим буфер
  sim->mqttReceive.mqttData = handle->rxFrame;
  handle->frame_offset = 0;
  handle->rxProcFlag = RESET;
}

//void mqttMsgReset( sUartRxHandle *handle, SIM800_t * sim );
static inline void mqttMsgReset( sUartRxHandle *handle, SIM800_t * sim ){
//  mqttBufClean( handle, sim );
  sim->mqttReceive.mqttData = handle->rxFrame;
  handle->frame_offset = 0;
  handle->rxProcFlag = RESET;
  sim->mqttReceive.msgState = MSG_NULL;
}

#endif /* MQTT_H_ */
