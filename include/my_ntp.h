/*
 * my_ntp.h
 *
 *  Created on: 10 сент. 2021 г.
 *      Author: jet
 */

#ifndef MY_NTP_H_
#define MY_NTP_H_

#define SNTP_SET_SYSTEM_TIME(sec)     setRtcTime( sec );
#include "lwip/apps/sntp.h"
#include "times.h"

#define NTP_SERVER    "3.pool.ntp.org"

#endif /* MY_NTP_H_ */
