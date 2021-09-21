/*
 * MQTTSim800.h
 *
 *
 */


#include <main.h>

// === CONFIG ===
#define UART_SIM800     &simUart
#define FREERTOS    0
#define CMD_DELAY_2     200
#define CMD_DELAY_5     500
#define CMD_DELAY_10    1000
#define CMD_DELAY_30    3000
#define CMD_DELAY_50    5000
// ==============

typedef struct {
    char *apn;
    char *apn_user;
    char *apn_pass;
} sim_t;

typedef struct {
    char *host;
    uint16_t port;
    uint8_t connect;
} mqttServer_t;

typedef struct {
    char *username;
    char *pass;
    char *clientID;
    unsigned short keepAliveInterval;
//    uint32_t toutTick;
} mqttClient_t;

typedef struct {
    uint8_t newEvent;
    unsigned char dup;
    int qos;
    unsigned char retained;
    unsigned short msgId;
    unsigned char payload[64];
    int payloadLen;
    unsigned char topic[64];
    int topicLen;
} mqttReceive_t;

typedef struct {
    sim_t sim;
    mqttServer_t mqttServer;
    mqttClient_t mqttClient;
    mqttReceive_t mqttReceive;
} SIM800_t;


extern uint8_t rx_data;
extern char mqtt_buffer[1460];
extern SIM800_t SIM800;

void Sim800_RxCallBack(void);

void clearRxBuffer(void);

void clearMqttBuffer(void);

int SIM800_SendCommand(char *command, char *reply, uint16_t delay);

void mqttInit( void );
int MQTT_Deinit(void);
int mqttStart(void);

void MQTT_Connect(void);

void MQTT_Pub(char *topic, char *payload);

void MQTT_PubUint8(char *topic, uint8_t data);

void MQTT_PubUint16(char *topic, uint16_t data);

void MQTT_PubUint32(char *topic, uint32_t data);

void MQTT_PubFloat(char *topic, float payload);

void MQTT_PubDouble(char *topic, double data);

void MQTT_PingReq(void);

void MQTT_Sub(char *topic);

void MQTT_Receive(unsigned char *buf);

void mqttProcess( void );
