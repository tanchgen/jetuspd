/*******************************************************************************
 * Copyright (c) 2014 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Ian Craggs - initial API and implementation and/or initial documentation
 *    Xiang Rong - 442039 Add makefile to Embedded C client
 *******************************************************************************/

#ifndef MQTTPUBLISH_H_
#define MQTTPUBLISH_H_


#define MQTT_SUB_TOUT     (uint32_t)(TOUT_1000 * 20)
#define MQTT_PUB_TOUT     (uint32_t)(TOUT_1000 * 30)
#define MQTT_PING_TOUT     (uint32_t)(TOUT_1000 * 45)

int MQTTSerialize_publish(unsigned char* buf, int buflen, unsigned char dup, int qos, unsigned char retained, unsigned short packetid,
		MQTTString topicName, unsigned char* payload, int payloadlen);

int MQTTDeserialize_publish(unsigned char* dup, int* qos, unsigned char* retained, unsigned short* packetid, MQTTString* topicName,
		unsigned char** payload, int* payloadlen, unsigned char* buf, int len);

int MQTTSerialize_puback(unsigned char* buf, int buflen, unsigned short packetid);
int MQTTSerialize_pubrec(unsigned char* buf, int buflen, unsigned short packetid);
int MQTTSerialize_pubrel(unsigned char* buf, int buflen, unsigned char dup, unsigned short packetid);
int MQTTSerialize_pubcomp(unsigned char* buf, int buflen, unsigned short packetid);

#endif /* MQTTPUBLISH_H_ */
