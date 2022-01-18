/*
 * MQTTSim800.c
 *
 */
#include "MQTTSim800.h"

#include <gsm.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "main.h"
#include "usart_arch.h"
#include "mqtt.h"

extern FlagStatus fwUpdFlag;

//uint8_t rx_data = 0;
//uint8_t rx_buffer[1460] = {0};
uint16_t rx_index = 0;

//uint8_t mqtt_receive = 0;

#define MQTT_BUF_SIZE   1460
char mqtt_buffer[MQTT_BUF_SIZE] = {0};
uint16_t mqtt_index = 0;

FlagStatus mqttSubFlag = RESET;
FlagStatus mqttPingFlag = RESET;

struct timer_list mqttPingTimer;
struct timer_list mqttSubTimer;

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


// SIM800 reply callback: Сохранение IMEI в буфере mqtt_buf
void saveImeiReply( sUartRxHandle * handle ){
  if( handle->replyBuf != NULL ){
    if( isdigit(handle->rxFrame[0]) && isdigit(handle->rxFrame[14]) ){
      // Похоже, получен IMEI
      memcpy( SIM800.sim.imei, (char*)handle->rxFrame, 15 );
      SIM800.sim.imei[15] = '\0';
    }
  }
  clearRxBuffer( (char *)(handle->rxFrame), &(handle->frame_offset) );
}


// SIM800 reply callback: Сохранение отклика в буфере mqtt_buf
void connectSimReply( sUartRxHandle * handle ){
  if( strcmp( handle->reply, "CONNECT\r\n") == 0 ) {
    // Есть соединение с MQTT-сервером
    SIM800.mqttServer.tcpconn = SET;
    trace_puts("tcp conn");
  }
  clearRxBuffer( (char *)(handle->rxFrame), &(handle->frame_offset) );
}

/**
 * Connect to MQTT server in Internet over TCP.
 * @param NONE
 * @return NONE
 */
void MQTT_Disconnect(void) {
  uint8_t * buf;
  if( (buf = malloc( 4 )) == NULL ){
    ErrHandler( NON_STOP );
  }
  else {
    int mqtt_len = MQTTSerialize_disconnect(buf, 4);
//    trace_printf( "a_buf_%x\n", buf );
    uartSend( simHnd.txh, buf, mqtt_len );
  }
}

/**
 * Deinitialization SIM800.
 * @param NONE
 * @return error status, 0 - OK
 */
int MQTT_Deinit(void) {
  int error = 0;

//  MQTT_Disconnect();
//  mDelay(100);

  gsmSendCommand("AT+CIPCLOSE=1\r\n", "OK\r\n", CMD_DELAY_5, NULL );
  error += gsmSendCommand("AT+CGATT=0\r\n", "OK\r\n", CMD_DELAY_10, NULL );
  error += gsmSendCommand("AT+CIPSHUT\r\n", "SHUT OK\r\n", CMD_DELAY_10, NULL );
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

  if((str = malloc( 256 )) == NULL ){
    ErrHandler( NON_STOP );
  }
  else {
    sprintf(str, "AT+CIPSTART=\"TCP\",\"%s\",%u\r\n", SIM800.mqttServer.host, *SIM800.mqttServer.port);
//    trace_printf( "a_buf_%x\n", str );
    rc = gsmSendCommand(str, "CONNECT\r\n", CMD_DELAY_10*30, connectSimReply );
  }
  return rc;
}

/**
 * Connect to MQTT server in Internet over TCP.
 * @param NONE
 * @return NONE
 */
void MQTT_Connect(void) {
  int mqtt_len;
  unsigned char buf[128] = {0};
  uint8_t * bf;

  MQTTPacket_connectData datas = MQTTPacket_connectData_initializer;
  datas.username.cstring = SIM800.mqttClient.username;
  datas.password.cstring = SIM800.mqttClient.pass;
  datas.clientID.cstring = SIM800.mqttClient.clientID;
  datas.keepAliveInterval = SIM800.mqttClient.keepAliveInterval;
  datas.cleansession = 1;
  mqtt_len = MQTTSerialize_connect(buf, sizeof(buf), &datas);

  if( (bf = malloc( mqtt_len)) == NULL ){
    ErrHandler( NON_STOP );
  }
  else {
    memcpy( bf, buf, mqtt_len);
//    trace_printf( "a_buf_%x\n", bf );
    uartSend( simHnd.txh, bf, mqtt_len );
  }
}

/**
 * Public on the MQTT broker of the message in a topic
 * @param topic to be used to the set topic
 * @param payload to be used to the set message for topic
 * @return NONE
 */
uint16_t MQTT_Pub(const char *topic, char *payload, enum QoS qos, uint16_t pktid){
  uint16_t rc = 0;
  uint16_t len = strlen(topic) + strlen(payload) + 9;
  MQTTString topicString = MQTTString_initializer;
  topicString.cstring = topic;
  uint8_t * buf;

  if( (buf = malloc(len) ) == NULL ){
    ErrHandler( NON_STOP );
  }
  else {
    int mqtt_len = MQTTSerialize_publish(buf, len, 0, qos, 0, pktid,
                                         topicString, (unsigned char *)payload, (int)strlen(payload));
//    trace_printf( "a_buf_%x\n", buf );
    if( (rc = uartSend( simHnd.txh, buf, mqtt_len )) ){
      if( qos ){
        SIM800.mqttClient.pubReady++;
      }
    }
  }

  return rc;
}


/**
 * A PUBACK Packet is the response to a PUBLISH Packet with QoS level 1.
 * @param topic to be used to the set topic
 * @param payload to be used to the set message for topic
 * @return NONE
 */
uint16_t MQTT_Puback(  unsigned short packetid ){
  uint16_t rc = 0;
  uint8_t * buf;

  if( (buf = malloc( 4 ) ) == NULL ){
    ErrHandler( NON_STOP );
  }
  else {
//    trace_printf( "a_buf_%x\n", buf );
    int mqtt_len = MQTTSerialize_puback(buf, 4, packetid );
    rc = uartSend( simHnd.txh, buf, mqtt_len );
  }

  return rc;
}


/**
 * PUBREC – Publish received (QoS 2 publish received, part 1)
 * @param topic to be used to the set topic
 * @param payload to be used to the set message for topic
 * @return NONE
 */
uint16_t MQTT_Pubrec(  unsigned short packetid ){
  uint16_t rc = 0;
  uint8_t * buf;

  if( (buf = malloc( 4 ) ) == NULL ){
    ErrHandler( NON_STOP );
  }
  else {
    int mqtt_len = MQTTSerialize_pubrec(buf, 4, packetid );
//    trace_printf( "a_buf_%x\n", buf );
    rc = uartSend( simHnd.txh, buf, mqtt_len );
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
  uint16_t rc = 0;
  uint8_t * buf;

  if( (buf = malloc( 4 ) ) == NULL ){
    ErrHandler( NON_STOP );
  }
  else {
    int mqtt_len = MQTTSerialize_pubrel(buf, 4, 0, packetid );
//    trace_printf( "a_buf_%x\n", buf );
    rc = uartSend( simHnd.txh, buf, mqtt_len );
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
  uint16_t rc = 0;
  uint8_t * buf;

  if( (buf = malloc( 4 ) ) == NULL ){
    ErrHandler( NON_STOP );
  }
  else {
    int mqtt_len = MQTTSerialize_pubcomp(buf, 4, packetid );
//    trace_printf( "a_buf_%x\n", buf );
    rc = uartSend( simHnd.txh, buf, mqtt_len );
  }

  return rc;
}


/**
 * Send a PINGREQ to the MQTT broker (active session)
 * @param NONE
 * @return NONE
 */
void MQTT_PingReq(void){
  uint8_t * buf;

  if( (buf = malloc( 16 ) ) == NULL ){
    ErrHandler( NON_STOP );
  }
  else {
    int mqtt_len = MQTTSerialize_pingreq(buf, 16 );
//    trace_printf( "a_buf_%x\n", buf );
    uartSend( simHnd.txh, buf, mqtt_len );
  }
}

/**
 * Subscribe on the MQTT broker of the message in a topic
 * @param topic to be used to the set topic
 * @return NONE
 */
void MQTT_Sub( char const *topic, uint8_t qos){
  uint8_t * buf;
  MQTTString topicString = MQTTString_initializer;
  topicString.cstring = topic;

  if( (buf = malloc( 256 )) == NULL ){
    ErrHandler( NON_STOP );
  }
  else {
    int mqtt_len = MQTTSerialize_subscribe( buf, 256, 0, 1, 1,
                                           &topicString, (int *)&qos);
//    trace_printf( "a_buf_%x\n", buf );
    uartSend( simHnd.txh, buf, mqtt_len );
  }
}


void mqttConnectCb( FlagStatus conn ){
  if( conn ){
    // Сначала подпишемся, потом там объявим о себе
    timerStack( &mqttSubTimer, 0, TIMER_MOD );
    ledOff( LED_R, 0 );
  }
  else {
    mqttPingFlag = RESET;
    mqttSubFlag = RESET;
    timerStack( &mqttSubTimer, 0, TIMER_DEL );
    // Две вспышки оранжевого цвета с интервалом в 3 сек
    ledToggleSet( LED_R, LED_BLINK_ON_TOUT, LED_SLOW_TOGGLE_TOUT, TOUT_3000, 2);
    ledToggleSet( LED_G, LED_BLINK_ON_TOUT, LED_SLOW_TOGGLE_TOUT, TOUT_3000, 2);
    if( gsmState >= GSM_MQTT_START ){
      gsmRun = RESET;
      gsmStRestart = GSM_MQTT_START;
    }
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
                SIM800.sim.ready = SIM_GSM_READY;
              }
              else if( strstr(mqtt_buffer, "PIN Ready\r\n" ) != NULL ) {
                SIM800.sim.ready = SIM_PIN_READY;
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
    if( fwUpdFlag == RESET ){
      if( (byte == '\n') && (handle->rxFrame[handle->frame_offset-2] == '\r') ) {
        if (strstr((char *)(handle->rxFrame+(handle->frame_offset-8)), "CLOSED\r\n")
            || strstr((char *)(handle->rxFrame+(handle->frame_offset-7)), "ERROR\r\n")
            || strstr((char *)(handle->rxFrame+(handle->frame_offset-7)), "DEACT\r\n"))
        {
          // Нет соединения с MQTT-сервером
          SIM800.mqttServer.disconnFlag = SET;
          SIM800.mqttServer.disconnTout = mTick + 30;
        }
        return;
      }
      else {
        SIM800.mqttServer.disconnFlag = RESET;
      }
    }

    // Пытаемся принять MQTT-сообщение
    mqttMsgProc( handle, &SIM800 );
  }

  return;
}
