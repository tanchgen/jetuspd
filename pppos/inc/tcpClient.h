/*
 * tcp_client.h
 *
 *  Created on: 13 февр. 2020 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#ifndef TCP_CLIENT_H_
#define TCP_CLIENT_H_

#define DEST_IP_ADDR0   ((uint8_t)192)
#define DEST_IP_ADDR1   ((uint8_t)168)
#define DEST_IP_ADDR2   ((uint8_t)21)
#define DEST_IP_ADDR3   ((uint8_t)29)

#define DEST_PORT   34634

/* TCP server protocol states */
typedef enum {
  CLIENT_NONE = 0,
  CLIENT_CONNECT,
  CLIENT_CONNECTED,
  CLIENT_RECEIVED,
  CLIENT_CLOSING,
} eClientState ;



void tcpClientConnect(void);

err_t tcpClient_dataSend( uint8_t * data, uint32_t len );

#endif /* TCP_CLIENT_H_ */
