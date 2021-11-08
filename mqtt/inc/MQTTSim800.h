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
#include "topic_id.h"

// === CONFIG ===
#define UART_SIM800     &simUart
#define CMD_DELAY_2     200
#define CMD_DELAY_5     500
#define CMD_DELAY_10    1000
#define CMD_DELAY_30    3000
#define CMD_DELAY_50    5000
// ==============

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

typedef struct {
    char *apn;
    char *apn_user;
    char *apn_pass;
} sim_t;

typedef struct {
    char *host;
    uint16_t port;
    FlagStatus tcpconn;
    uint8_t mqttconn;
} mqttServer_t;

typedef struct {
    char *username;
    char *pass;
    char *clientID;
    unsigned short keepAliveInterval;
//    uint32_t toutTick;
} mqttClient_t;

typedef struct {
    unsigned char dup;
    int qos;
    unsigned char retained;
    unsigned short pktId;
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
    sim_t sim;
    mqttServer_t mqttServer;
    mqttClient_t mqttClient;
    mqttReceive_t mqttReceive;
    FlagStatus ready;
} SIM800_t;


extern SIM800_t SIM800;

//extern uint8_t mqtt_receive;
extern char mqtt_buffer[1460];
extern uint16_t mqtt_index;

extern FlagStatus mqttSubFlag;
extern FlagStatus mqttPubFlag;

void Sim800_RxCallBack(void);
void clearRxBuffer( char * buf, uint32_t * size );
void clearMqttBuffer(void);
int SIM800_SendCommand(char *command, char *reply, uint16_t delay, void (*simreplycb)( sUartRxHandle *) );

void mqttInit( void );
int MQTT_Deinit(void);

void saveSimReply( sUartRxHandle * handle );
void connectSimReply( sUartRxHandle * handle );

int mqttStart(void);
void MQTT_Connect(void);

uint16_t MQTT_Pub(char *topic, char *payload);
void MQTT_PubUint8(char *topic, uint8_t data);
void MQTT_PubUint16(char *topic, uint16_t data);
void MQTT_PubUint32(char *topic, uint32_t data);
void MQTT_PubFloat(char *topic, float payload);
void MQTT_PubDouble(char *topic, double data);

uint16_t MQTT_Puback(  unsigned short packetid );
uint16_t MQTT_Pubrec(  unsigned short packetid );
uint16_t MQTT_Pubrel(  unsigned short packetid );
uint16_t MQTT_Pubcomp(  unsigned short packetid );

void MQTT_PingReq(void);

void MQTT_Sub(char *topic);

void MQTT_Receive(unsigned char *buf);

void mqttProcess( void );

#endif // MQTT_SIM_800_H_
