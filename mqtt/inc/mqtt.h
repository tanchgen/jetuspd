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
#include "gsm.h"

#include "topic_id.h"

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

typedef enum {
  MSG_NULL,
  MSG_TYPE,
  MSG_REMAINING_LEN,
  MSG_TOP_LEN,
  MSG_PKT_ID,
  MSG_TOPIC,
  MSG_PAY_LEN,
  MSG_PAYLOAD,
} eMsgState;

#define PUB_FLAG_MASK     0xF

typedef union {
  struct {
    uint16_t uspdAnnounce: 1;
    uint16_t cfgoPub: 1;
    uint16_t archPub: 1;
    uint16_t cmdPub: 1;
    uint16_t archPubEnd: 1;
    uint16_t announceEnd: 1;
  };
  uint16_t u16pubFlags;
} uPubFlags;

typedef struct {
    char *host;
    uint16_t * port;
    FlagStatus tcpconn;
    uint8_t mqttconn;
    uint32_t disconnFlag;
    uint32_t disconnTout;
} mqttServer_t;

typedef struct {
    char *username;
    char *pass;
    char *clientID;
    unsigned short keepAliveInterval;
    uint8_t subCount;
    uPubFlags pubFlags;
    uint8_t pubReady;
    //    uint32_t toutTick;
} mqttClient_t;

typedef struct {
    unsigned char dup;
    int qos;
    unsigned char retained;
    unsigned short pktId;
    unsigned short pktIdo;
    uint8_t * mqttData;
    uint32_t remLenMp;
    uint32_t remLen;
    unsigned char payload[64];
    uint32_t payloadLen;
    uint32_t payOffset;
    unsigned char topic[64];
    int topicLen;
    eTopicId topicId;
    eMsgType msgType;
    eMsgState msgState;
} mqttReceive_t;

typedef struct {
  uint8_t simActive;        // Номер активной SIM
  sim_t sim;
  mqttServer_t mqttServer;
  mqttClient_t mqttClient;
  mqttReceive_t mqttReceive;
  eSimReady ready;
} SIM800_t;

#if 0
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

extern sMqtt mqtt;

//void mqttInit(sMqtt *mqtt, struct ip_addr serverIp, int port, msgReceived fn, char *devId);
//uint8_t mqttConnect(sMqtt *this);
//uint8_t mqttPublish(sMqtt *this,char* pub_topic, char* msg);
//uint8_t mqttDisconnect(sMqtt *this);
//uint8_t mqttSubscribe(sMqtt *this, char* topic);
//uint8_t mqttLive(sMqtt *this);
//void mqttDisconnectForced(sMqtt *this);
//uint8_t mqttTcpConnect(sMqtt *this);
//uint8_t mqttBrokConnect(sMqtt *this);

#endif // 0
extern SIM800_t SIM800;

//extern uint8_t mqtt_receive;
extern char mqtt_buffer[1460];
extern uint16_t mqtt_index;

extern FlagStatus mqttSubFlag;
extern FlagStatus mqttPingFlag;

void mqttPubInit( void );
void mqttInit( void );
void mqttConnectCb( FlagStatus conn );

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
