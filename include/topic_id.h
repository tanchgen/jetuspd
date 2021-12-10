/*
 * topic_id.h
 *
 *  Created on: 24 окт. 2021 г.
 *      Author: jet
 */

#ifndef TOPIC_ID_H_
#define TOPIC_ID_H_

typedef enum topicId {
  TOPIC_DEV_IMEI,
  TOPIC_DEV_UID,
  TOPIC_INFO,
  TOPIC_TEMP,
  TOPIC_VOLT,
  TOPIC_CMD_I,
  TOPIC_CMD_O,
  TOPIC_CFG_I,
  TOPIC_CFG_O,
  TOPIC_ALRM,
  TOPIC_LOG,
  TOPIC_ISENS,
  TOPIC_ISENS_ARX,
  TOPIC_ISENS_STATE,
  TOPIC_ISENS_ADC,
  TOPIC_ISENS_TEMP,
  TOPIC_OUT,
  TOPIC_OUT_STATE,
  TOPIC_FW,
  TOPIC_FW_MAN,
  TOPIC_FW_BIN,
  TOPIC_RS,
  TOPIC_RS_TX,
  TOPIC_RS_RX,
  TOPIC_GSM,
  TOPIC_NUM
} eTopicId;

extern const char * tpcTempl[TOPIC_NUM];

#endif /* TOPIC_ID_H_ */
