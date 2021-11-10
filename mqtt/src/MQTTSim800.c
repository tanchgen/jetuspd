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


//uint8_t rx_data = 0;
//uint8_t rx_buffer[1460] = {0};
uint16_t rx_index = 0;

//uint8_t mqtt_receive = 0;

#define MQTT_BUF_SIZE   1460
char mqtt_buffer[MQTT_BUF_SIZE] = {0};
uint16_t mqtt_index = 0;

FlagStatus mqttSubFlag = RESET;
FlagStatus mqttPubFlag = RESET;

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


// SIM800 reply callback: Сохранение отклика в буфере mqtt_buf
void saveSimReply( sUartRxHandle * handle ){
  if( handle->replyBuf != NULL ){
    char * start = strstr( (char*)handle->rxFrame, handle->reply );
    assert_param( start != NULL );
    strcpy( handle->replyBuf, start );
  }
  clearRxBuffer( (char *)(handle->rxFrame), &(handle->frame_offset) );
}


// SIM800 reply callback: Сохранение отклика в буфере mqtt_buf
void connectSimReply( sUartRxHandle * handle ){
  if( strcmp( handle->reply, "CONNECT\r\n") == 0 ) {
    // Есть соединение с MQTT-сервером
    SIM800.mqttServer.tcpconn = 1;
  }
  clearRxBuffer( (char *)(handle->rxFrame), &(handle->frame_offset) );
}


/**
 * Send AT command to SIM800 over UART.
 * @param command the command to be used the send AT command
 * @param reply to be used to set the correct answer to the command
 * @param delay to be used to the set pause to the reply
 * @return error, 0 is OK
 */
int SIM800_SendCommand(char *command, char *reply, uint16_t delay, void (*simreplycb)( sUartRxHandle *) ){
  uint32_t tmptick;
  tmptick = mTick + delay;
  uint8_t rc = 1;

  simHnd.rxh->replyCb = simreplycb;

  simHnd.txh->data = (uint8_t*)command;
  simHnd.rxh->reply = reply;
  simHnd.rxh->replyFlag = RESET;
  trace_write( command, strlen(command) );
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
  return rc;
}


/**
 * Deinitialization SIM800.
 * @param NONE
 * @return error status, 0 - OK
 */
int MQTT_Deinit(void) {
  int error = 0;

  mDelay(1000);
  simHnd.txh->data = (uint8_t *)"+++\r\n";
  if( uartTransmit(simHnd.txh, 5, 1000) == 0 ){
    trace_puts( "uart err" );
  }

  mDelay(1000);

  error += SIM800_SendCommand("ATE1\r\n", "OK\r\n", CMD_DELAY_2, NULL );
  error += SIM800_SendCommand("AT+CGATT=0\r\n", "OK\r\n", CMD_DELAY_5, NULL );
  error += SIM800_SendCommand("AT+CIPSHUT\r\n", "SHUT OK\r\n", CMD_DELAY_5, NULL );
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
  int rc = -1;

  SIM800.mqttServer.mqttconn = 0;
  char * str;

  if((str = ta_alloc( 256 )) == NULL ){
    Error_Handler( NON_STOP );
  }
  else {
    sprintf(str, "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", SIM800.mqttServer.host, SIM800.mqttServer.port);
    rc = SIM800_SendCommand(str, "CONNECT\r\n", CMD_DELAY_10*30, connectSimReply );
  }
  return rc;
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

  if( (simHnd.txh->data = ta_alloc( mqtt_len)) == NULL ){
    Error_Handler( NON_STOP );
  }
  else {
    memcpy( simHnd.txh->data, buf, mqtt_len);
    trace_puts( "mqtt conn ");
    uartTransmit( simHnd.txh, mqtt_len, TOUT_100 );
  }
}

/**
 * Public on the MQTT broker of the message in a topic
 * @param topic to be used to the set topic
 * @param payload to be used to the set message for topic
 * @return NONE
 */
uint16_t MQTT_Pub(char *topic, char *payload){
  uint16_t rc = 0;
  uint16_t len = strlen(topic) + strlen(payload) + 9;
  MQTTString topicString = MQTTString_initializer;
  topicString.cstring = topic;

  if( (simHnd.txh->data = ta_alloc(len) ) == NULL ){
    Error_Handler( NON_STOP );
  }
  else {
    int mqtt_len = MQTTSerialize_publish(simHnd.txh->data, len, 0, 0, 0, 0,
                                         topicString, (unsigned char *)payload, (int)strlen(payload));
    rc = uartTransmit( simHnd.txh, mqtt_len, TOUT_100 );
  }

  return rc;
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
  uint16_t rc;

  if( (simHnd.txh->data = ta_alloc(4) ) == NULL ){
    Error_Handler( NON_STOP );
    rc = 0;
  }
  else {
    int mqtt_len = MQTTSerialize_pubrec(simHnd.txh->data, 4, packetid);
    rc = uartTransmit( simHnd.txh, mqtt_len, TOUT_100 );
  }

  return rc;
}


/**
 * PUBREL – Publish release (QoS 2 publish received, part 2)
 * @param topic to be used to the set topic
 * @param payload to be used to the set message for topic
 * @return NONE
 */
uint16_t MQTT_Pubrel(  unsigned short packetid ){
  uint16_t rc;

  if( (simHnd.txh->data = ta_alloc(4) ) == NULL ){
    Error_Handler( NON_STOP );
    rc = 0;
  }
  else {
    int mqtt_len = MQTTSerialize_pubrel(simHnd.txh->data, 4, 0, packetid);
    rc = uartTransmit( simHnd.txh, mqtt_len, TOUT_100 );
  }

  return rc;
}


/**
 * The PUBCOMP Packet is the response to a PUBREL Packet.
 * It is the fourth and final packet of the QoS 2 protocol exchange.
 * @param topic to be used to the set topic
 * @param payload to be used to the set message for topic
 * @return NONE
 */
uint16_t MQTT_Pubcomp(  unsigned short packetid ){
  uint16_t rc;

  if( (simHnd.txh->data = ta_alloc(4) ) == NULL ){
    Error_Handler( NON_STOP );
    rc = 0;
  }
  else {
    int mqtt_len = MQTTSerialize_pubcomp(simHnd.txh->data, 4, packetid);
    rc = uartTransmit( simHnd.txh, mqtt_len, TOUT_100 );
  }

  return rc;
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
void MQTT_Sub(char *topic, uint8_t qos){


    MQTTString topicString = MQTTString_initializer;
    topicString.cstring = topic;

    if( (simHnd.txh->data = ta_alloc( 256 )) == NULL ){
      Error_Handler( NON_STOP );
    }
    else {
      int mqtt_len = MQTTSerialize_subscribe( simHnd.txh->data, 256, 0, 1, 1,
                                             &topicString, (int *)&qos);
      uartTransmit( simHnd.txh, mqtt_len, TOUT_100 );
    }
}


void mqttConnectCb( FlagStatus conn ){
  if( conn ){
    mqttSubFlag = SET;
    mqttPubFlag = SET;
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
    if( (byte == '\n') && (handle->rxFrame[handle->frame_offset-2] == '\r') ){
      if( handle->frame_offset == 2 ) {
        // Пустая строка
        handle->frame_offset = 0;
        return;
      }
      else {
  //    memcpy(mqtt_buffer, handle->rxFrame, handle->frame_offset );
        handle->rxFrame[handle->frame_offset] = '\0';
  //        clearRxBuffer( (char *)handle->rxFrame, &handle->frame_offset );
        if( handle->reply != NULL ){
          if( strstr( (char*)handle->rxFrame, handle->reply ) ) {
            handle->replyFlag = SET;
            if( handle->replyCb!= NULL ){
              (handle->replyCb)( handle );
            }
            else {
              // Callback == NULL
              if( strstr(mqtt_buffer, "SMS Ready\r\n" ) != NULL ) {
                SIM800.ready = SET;
              }
              clearRxBuffer( (char *)(simHnd.rxh->rxFrame), &(simHnd.rxh->frame_offset) );
            }

          }
        }
        // Получили и обработали строку
        handle->frame_offset = 0;
        return;
      }
    }
  }
  else {
    // Есть TCP-соединение
    if( (byte == '\n') && (handle->rxFrame[handle->frame_offset-2] == '\r') ) {
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
