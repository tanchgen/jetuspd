/*
 * MQTTSim800.c
 *
 */
#include "MQTTSim800.h"

#include <gsm.h>
#include <my_mqtt.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "usart_arch.h"


//uint8_t rx_data = 0;
//uint8_t rx_buffer[1460] = {0};
uint16_t rx_index = 0;

//uint8_t mqtt_receive = 0;

char mqtt_buffer[MQTT_BUF_SIZE] = {0};
uint16_t mqtt_index = 0;

FlagStatus mqttSubFlag = RESET;
FlagStatus mqttPubFlag = RESET;

struct timer_list mqttPubTimer;

// ------------------- Function prototype ---------------------------
//void mqttConnectCb( mqtt_client_t *client, void *arg, mqtt_connection_status_t conn );
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

  if( (simHnd.txh->data = my_alloc(len) ) == NULL ){
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

  if( (simHnd.txh->data = my_alloc(4) ) == NULL ){
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

  if( (simHnd.txh->data = my_alloc(4) ) == NULL ){
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

  if( (simHnd.txh->data = my_alloc(4) ) == NULL ){
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

    if( (simHnd.txh->data = my_alloc( 256 )) == NULL ){
      Error_Handler( NON_STOP );
    }
    else {
      int mqtt_len = MQTTSerialize_subscribe( simHnd.txh->data, 256, 0, 1, 1,
                                             &topicString, (int *)&qos);
      uartTransmit( simHnd.txh, mqtt_len, TOUT_100 );
    }
}


// Обработка принятых по UART данных посимвольно
void simUartRxProc( sUartRxHandle * handle, uint8_t byte ){

  if (SIM800.mqttServer.gprsconn == RESET ) {
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
    pppos_input( ppp, &(handle->rxFrame[handle->frame_offset]), 1 );
    handle->frame_offset = 0;
  }
  return;
}
