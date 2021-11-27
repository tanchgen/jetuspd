/**
  ******************************************************************************
  * @file    netconf.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    31-July-2013 
  * @brief   This file contains all the functions prototypes for the netconf.c 
  *          file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NETCONF_H
#define __NETCONF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
/* Exported types ------------------------------------------------------------*/

 //#define USE_LCD        /* enable LCD  */
 #define USE_DHCP     1       /* enable DHCP, if disabled static address is used */

 /*Static IP ADDRESS: IP_ADDR0.IP_ADDR1.IP_ADDR2.IP_ADDR3 */
 #define IP_ADDR0   (uint8_t) 192
 #define IP_ADDR1   (uint8_t) 168
 #define IP_ADDR2   (uint8_t) 0
 #define IP_ADDR3   (uint8_t) 10

 /*NETMASK*/
 #define NETMASK_ADDR0   (uint8_t) 255
 #define NETMASK_ADDR1   (uint8_t) 255
 #define NETMASK_ADDR2   (uint8_t) 255
 #define NETMASK_ADDR3   (uint8_t) 0

 /*Gateway Address*/
 #define GW_ADDR0   (uint8_t) 192
 #define GW_ADDR1   (uint8_t) 168
 #define GW_ADDR2   (uint8_t) 0
 #define GW_ADDR3   (uint8_t) 1

 #define TCP_LOCAL_PORT     43827
 #define TCP_REMOTE_PORT    34633

#define FLASH_MAC_MAGIC_OFFSET          (4 * 16 * 1024)
#define FLASH_MAC_ADDRESS_OFFSET        (4 * 16 * 1024 + 8)
#define FLASH_MAC_SECTOR_NUM            (FLASH_Sector_4)

/* Exported constants --------------------------------------------------------*/
#define DHCP_START                 1
#define DHCP_WAIT_ADDRESS          2
#define DHCP_ADDRESS_ASSIGNED      3
#define DHCP_TIMEOUT               4
#define DHCP_LINK_DOWN             5
/* Exported macro ------------------------------------------------------------*/

/** Состояния подсистемы DHCP. */
enum arch_dhcp_states {
  DHCP_OffState,

  DHCP_StartState ,
  DHCP_WaitAddressState,
  DHCP_AddressAssignedState,

  DHCP_TimeoutState,
  DHCP_LinkDownState,
};

/** Текущее состояние подсистемы DHCP */
extern enum arch_dhcp_states DHCP_state;

FlagStatus ipsetFlag;

/* Exported functions ------------------------------------------------------- */
void Eth_Link_ITHandler(uint16_t PHYAddress);

#ifdef __cplusplus
}
#endif

#endif /* __NETCONF_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
