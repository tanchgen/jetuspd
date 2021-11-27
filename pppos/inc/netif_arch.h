#ifndef _NETIF_ARCH_H
#define _NETIF_ARCH_H

#ifdef STM32F4XX
#include "stm32f4xx.h"
#include "stm32f4x7_eth.h"
#else
#include "stm32f2xx.h"
#include "stm32f2x7_eth.h"
#include "stm32f2x7_eth_conf.h"
#endif // STM32F4XX
#include "netconf.h"

#define ETH_LINK_EXTI_LINE    EXTI_Line14
#define ETH_LINK_STATUS_IRQ_PRIORITY   4
/* Статус сетевого соединения */
extern __IO uint8_t arch_net_link_status;

void netifClock( void );
void netifEnable( void );
void netifInit( void );

#endif /* _ETH_ARCH_H */

