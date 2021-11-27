/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 */
#ifndef __TCP_SERVER_H__
#define __TCP_SERVER_H__

#include "tm_xfer.h"


/** Маска пакетов TCP основной прошивки. */
#define FIRMWARE_TCP_FRAME_TYPE_MASK  (0x8000)

/** Магическое слово переопределения стандартного MAC-адреса. */
#define TCP_SOFTWARE_MAC_MAGIC1         (0xabad)
#define TCP_SOFTWARE_MAC_MAGIC2         (0xbabe)

/** Типы пакетов TCP. */
enum arch_tcp_frame_types {
  /** Пакет с командой сброса микроконтроллера. */
  FirmwareMagicTcpFrameType     = (FIRMWARE_TCP_FRAME_TYPE_MASK | 0),
  /** Пакет с командой чтения MAC-адреса микроконтроллера. */
  FirmwareReadMacTcpFrameType     = (FIRMWARE_TCP_FRAME_TYPE_MASK | 1),
  /** Пакет с командой записи MAC-адреса микроконтроллера. */
  FirmwareWriteMacTcpFrameType      = (FIRMWARE_TCP_FRAME_TYPE_MASK | 2),
};

/* TCP server protocol states */
enum tcpServerStates {
  ES_NONE = 0,
  ES_ACCEPTED,
  ES_RECEIVED,
  ES_CLOSING
};

/* structure for maintaing connection infos to be passed as argument
   to LwIP callbacks*/
typedef struct {
  enum tcpServerStates state;             /* current connection state */
  struct tcp_pcb *pcb;   /* pointer on the current tcp_pcb */
  struct pbuf *rxp;      /* pointer on the received */
  struct pbuf *txp;      /* pointer on the transmitting pbuf */
  uint16_t txId;          /** ИД исходящего пакета */
  uint8_t  n_err;      /** Счетчик ошибок. */
  uint16_t  rxId;      /** Идентификатор последнего обработанного пакета из TCP. */
} sTcpClient;

/** Структура дескриптора tcp сервера */
typedef struct {
  struct list_head xferNode;    // Нод очереди XFER
  struct list_head * pTxQueue;  // Указатель на Очередь пакетов на передачу
  struct list_head  txQueue;    // Очередь пакетов на передачу.
  /** Буфер для размещения структуры дескриптора текущего обрабатываемого пакета USART. */
  sXferFrameTx  *txframe;
  /** Текущее смещение данных относительно начала пакета. */
  size_t  txleft;
  uint8_t tcpConnQuant;   // Счетчик подключенных клиентов
  uint8_t tcpConnLeft;    // Которому количеству клиентов осталось отправить пакет
} sTcpServer;

typedef struct {
  struct list_head ctxnode;
  struct pbuf *p;
} sTcpFrameCtx;


/** Структура пакета TCP по приему. */
typedef struct __packed {
  /** Тип пакета TCP. */
  uint16_t  type;

  /** Идентификатор пакета TCP. */
  uint16_t  n_s;

  /** Размер данных. */
  uint16_t  data_size;

  /** Буфер опциональных данных. */
  uint8_t  rx_data[0];
} sTcpFrameRx;

extern sTcpServer tcpServer;

#endif /* __TCP_SERVER */
