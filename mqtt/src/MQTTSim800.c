/*
 * MQTTSim800.c
 *
 */
#include "MQTTSim800.h"

#include <gsm.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "usart_arch.h"
#include "mqtt.h"


uint8_t tx_buffer[256] = {0};

//uint8_t rx_data = 0;
//uint8_t rx_buffer[1460] = {0};
uint16_t rx_index = 0;

//uint8_t mqtt_receive = 0;

#define MQTT_BUF_SIZE   1460
char mqtt_buffer[MQTT_BUF_SIZE] = {0};
uint16_t mqtt_index = 0;

FlagStatus mqttSubFlag = RESET;
FlagStatus mqttPubFlag = RESET;

struct timer_list mqttPubTimer;

// ------------------- Function prototype ---------------------------
void mqttConnectCb( FlagStatus conn );
void mqttMsgProc( sUartRxHandle * handle, SIM800_t * sim );
// ------------------------------------------------------------------


/**
 * Clear SIM800 UART RX buffer.
 * @param NONE
 * @return NONE
 */
void clearRxBuffer( char * buf, uint32_t * size ){
  memset( buf, 0, *size );
  *size = 0;
}

/**
 * Clear MQTT buffer.
 * @param NONE
 * @return NONE
 */
void clearMqttBuffer(void) {
  mqtt_index = 0;
  memset(mqtt_buffer, 0, sizeof(mqtt_buffer));
}

/**
 * Send AT command to SIM800 over UART.
 * @param command the command to be used the send AT command
 * @param reply to be used to set the correct answer to the command
 * @param delay to be used to the set pause to the reply
 * @return error, 0 is OK
 */
int SIM800_SendCommand(char *command, char *reply, uint16_t delay){
  uint32_t tmptick;
  tmptick = mTick + delay;
  uint8_t rc = 1;

  *mqtt_buffer = '\0';
  simHnd.txh->data = (uint8_t*)command;
  simHnd.rxh->reply = reply;
  simHnd.rxh->replyFlag = RESET;
  if( uartTransmit(simHnd.txh, (uint16_t)strlen(command), 1000) == 0 ){
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
  // Подготовим буфер для приема
  clearRxBuffer( (char *)(simHnd.rxh->rxFrame), &(simHnd.rxh->frame_offset) );
  return rc;
}


/**
 * Deinitialization SIM800.
 * @param NONE
 * @return error status, 0 - OK
 */
int MQTT_Deinit(void) {
    int error = 0;

    const char * cmd = "+++\r\n";

    mDelay(1000);
    simHnd.txh->data = (uint8_t*)cmd;
    if( uartTransmit(simHnd.txh, 5, 1000) != 0 ){
      trace_puts( "uart err" );
    }

    mDelay(1000);

    SIM800_SendCommand("ATE1\r\n", "OK\r\n", CMD_DELAY_2);

    error += SIM800_SendCommand("AT+CGATT=0\r\n", "OK\r\n", CMD_DELAY_5);
    error += SIM800_SendCommand("AT+CIPSHUT\r\n", "SHUT OK\r\n", CMD_DELAY_5);
    SIM800.mqttServer.tcpconn = 0;
    SIM800.mqttServer.mqttconn = 0;
    return error;
}


/**
 * Connect to TCP (MQTT) server.
 * @param NONE
 * @return NONE
 */
int TCP_Connect(void) {
    SIM800.mqttServer.mqttconn = 0;
    char str[128] = {0};
    sprintf(str, "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", SIM800.mqttServer.host, SIM800.mqttServer.port);
    return SIM800_SendCommand(str, "CONNECT\r\n", CMD_DELAY_10*30 );
}

/**
 * Connect to MQTT server in Internet over TCP.
 * @param NONE
 * @return NONE
 */
void MQTT_Connect(void) {
  unsigned char buf[128] = {0};

  MQTTPacket_connectData datas = MQTTPacket_connectData_initializer;
  datas.username.cstring = SIM800.mqttClient.username;
  datas.password.cstring = SIM800.mqttClient.pass;
  datas.clientID.cstring = SIM800.mqttClient.clientID;
  datas.keepAliveInterval = SIM800.mqttClient.keepAliveInterval;
  datas.cleansession = 1;
  int mqtt_len = MQTTSerialize_connect(buf, sizeof(buf), &datas);
  simHnd.txh->data = buf;
  uartTransmit( simHnd.txh, mqtt_len, TOUT_100 );
}

/**
 * Public on the MQTT broker of the message in a topic
 * @param topic to be used to the set topic
 * @param payload to be used to the set message for topic
 * @return NONE
 */
uint16_t MQTT_Pub(char *topic, char *payload){
  unsigned char buf[256] = {0};

  MQTTString topicString = MQTTString_initializer;
  topicString.cstring = topic;

  int mqtt_len = MQTTSerialize_publish(buf, sizeof(buf), 0, 0, 0, 0,
                                       topicString, (unsigned char *)payload, (int)strlen(payload));
  simHnd.txh->data = buf;
  return uartTransmit( simHnd.txh, mqtt_len, TOUT_100 );
}

/**
 * Public on the MQTT broker of the message in a topic
 * @param topic (uint8_t)  to be used to the set topic
 * @param payload to be used to the set message for topic
 * @return NONE
 */
void MQTT_PubUint8(char *topic, uint8_t payload)
{
    char str[32] = {0};
    sprintf(str, "%u", payload);
    MQTT_Pub(topic, str);
}

/**
 * Public on the MQTT broker of the message in a topic
 * @param topic (uint16_t)  to be used to the set topic
 * @param payload to be used to the set message for topic
 * @return NONE
 */
void MQTT_PubUint16(char *topic, uint16_t payload)
{
    char str[32] = {0};
    sprintf(str, "%u", payload);
    MQTT_Pub(topic, str);
}

/**
 * Public on the MQTT broker of the message in a topic
 * @param topic (uint32_t)  to be used to the set topic
 * @param payload to be used to the set message for topic
 * @return NONE
 */
void MQTT_PubUint32(char *topic, uint32_t payload)
{
    char str[32] = {0};
    sprintf(str, "%lu", payload);
    MQTT_Pub(topic, str);
}

/**
 * Public on the MQTT broker of the message in a topic
 * @param topic (float)  to be used to the set topic
 * @param payload to be used to the set message for topic
 * @return NONE
 */
void MQTT_PubFloat(char *topic, float payload)
{
    char str[32] = {0};
    sprintf(str, "%f", payload);
    MQTT_Pub(topic, str);
}

/**
 * Public on the MQTT broker of the message in a topic
 * @param topic (double)  to be used to the set topic
 * @param payload to be used to the set message for topic
 * @return NONE
 */
void MQTT_PubDouble(char *topic, double payload)
{
    char str[32] = {0};
    sprintf(str, "%f", payload);
    MQTT_Pub(topic, str);
}

/**
 * A PUBACK Packet is the response to a PUBLISH Packet with QoS level 1.
 * @param topic to be used to the set topic
 * @param payload to be used to the set message for topic
 * @return NONE
 */
uint16_t MQTT_Puback(  unsigned short packetid ){
  unsigned char buf[4] = {0};

  int mqtt_len = MQTTSerialize_puback(buf, sizeof(buf), packetid );
  simHnd.txh->data = buf;
  return uartTransmit( simHnd.txh, mqtt_len, TOUT_100 );
}


/**
 * PUBREC – Publish received (QoS 2 publish received, part 1)
 * @param topic to be used to the set topic
 * @param payload to be used to the set message for topic
 * @return NONE
 */
uint16_t MQTT_Pubrec(  unsigned short packetid ){
  unsigned char buf[4] = {0};

  int mqtt_len = MQTTSerialize_pubrec(buf, sizeof(buf), packetid );
  simHnd.txh->data = buf;
  return uartTransmit( simHnd.txh, mqtt_len, TOUT_100 );
}


/**
 * PUBREL – Publish release (QoS 2 publish received, part 2)
 * @param topic to be used to the set topic
 * @param payload to be used to the set message for topic
 * @return NONE
 */
uint16_t MQTT_Pubrel(  unsigned short packetid ){
  unsigned char buf[4] = {0};

  int mqtt_len = MQTTSerialize_pubrel(buf, sizeof(buf), 0, packetid );
  simHnd.txh->data = buf;
  return uartTransmit( simHnd.txh, mqtt_len, TOUT_100 );
}


/**
 * The PUBCOMP Packet is the response to a PUBREL Packet.
 * It is the fourth and final packet of the QoS 2 protocol exchange.
 * @param topic to be used to the set topic
 * @param payload to be used to the set message for topic
 * @return NONE
 */
uint16_t MQTT_Pubcomp(  unsigned short packetid ){
  unsigned char buf[4] = {0};

  int mqtt_len = MQTTSerialize_pubcomp(buf, sizeof(buf), packetid );
  simHnd.txh->data = buf;
  return uartTransmit( simHnd.txh, mqtt_len, TOUT_100 );
}


/**
 * Send a PINGREQ to the MQTT broker (active session)
 * @param NONE
 * @return NONE
 */
void MQTT_PingReq(void)
{
    unsigned char buf[16] = {0};

    int mqtt_len = MQTTSerialize_pingreq(buf, sizeof(buf));
    simHnd.txh->data = buf;
    uartTransmit( simHnd.txh, mqtt_len, TOUT_100 );
}


/**
 * Subscribe on the MQTT broker of the message in a topic
 * @param topic to be used to the set topic
 * @return NONE
 */
void MQTT_Sub(char *topic)
{
    unsigned char buf[256] = {0};

    MQTTString topicString = MQTTString_initializer;
    topicString.cstring = topic;

    int mqtt_len = MQTTSerialize_subscribe(buf, sizeof(buf), 0, 1, 1,
                                           &topicString, 0);
    simHnd.txh->data = buf;
    uartTransmit( simHnd.txh, mqtt_len, TOUT_100 );
    mDelay(100);
}


void mqttConnectCb( FlagStatus conn ){
  if( conn ){
    mqttSubFlag = SET;
    ledOff( LED_R, 0 );
  }
  else {
    mqttPubFlag = RESET;
    mqttSubFlag = RESET;
    // Две вспышки оранжевого цвета с интервалом в 3 сек
    ledToggleSet( LED_R, LED_BLINK_ON_TOUT, LED_SLOW_TOGGLE_TOUT, TOUT_3000, 2);
    ledToggleSet( LED_G, LED_BLINK_ON_TOUT, LED_SLOW_TOGGLE_TOUT, TOUT_3000, 2);
  }
}


// Обработка принятых по UART данных посимвольно
void simUartRxProc( sUartRxHandle * handle, uint8_t byte ){

  if (SIM800.mqttServer.tcpconn == 0) {
    if( (handle->rxFrame[handle->frame_offset-1] == '\r') && (byte == '\n') ){
      if( handle->frame_offset == 1 ) {
        // Пустая строка
        handle->frame_offset = 0;
        return;
      }
    }
    else {
//    memcpy(mqtt_buffer, handle->rxFrame, handle->frame_offset );
      handle->rxFrame[handle->frame_offset + 1] = '\0';
//        clearRxBuffer( (char *)handle->rxFrame, &handle->frame_offset );
      if( handle->reply != NULL ){
        if( strstr( (char*)handle->rxFrame, handle->reply ) ) {
          handle->replyFlag = SET;
          if( strcmp( handle->reply, "CONNECT\r\n") == 0 ) {
            // Есть соединение с MQTT-сервером
            SIM800.mqttServer.tcpconn = 1;
          }
        }
      }
      // Получили и обработали строку
      handle->frame_offset = 0;
      return;
    }
  }
  else {
    // Есть TCP-соединение
    if( (handle->rxFrame[handle->frame_offset-2] == '\r') && (byte == '\n') ) {
      if (strstr((char *)handle->rxFrame, "CLOSED\r\n")
          || strstr((char *)handle->rxFrame, "ERROR\r\n")
          || strstr((char *)handle->rxFrame, "DEACT\r\n"))
      {
        // Нет соединения с MQTT-сервером
        SIM800.mqttServer.tcpconn = 0;
        SIM800.mqttServer.mqttconn = 0;
        mqttConnectCb( SIM800.mqttServer.tcpconn );
        // Получили и обработали строку - если и принимали что-то, но не до конца - все потеряли
        mqttMsgReset( handle, &SIM800 );
      }
      return;
    }

    // Пытаемся принять MQTT-сообщение
    mqttMsgProc( handle, &SIM800 );
  }

  return;
}
