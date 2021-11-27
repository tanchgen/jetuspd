#ifndef __STM32F2xx_HAL_CONF_H
#define __STM32F2xx_HAL_CONF_H

/* ################## Ethernet peripheral configuration ##################### */

/* Section 1 : Ethernet peripheral configuration */

/* MAC ADDRESS: MAC_ADDR0:MAC_ADDR1:MAC_ADDR2:MAC_ADDR3:MAC_ADDR4:MAC_ADDR5 */
#define MAC_ADDR0                         2U
#define MAC_ADDR1                         0U
#define MAC_ADDR2                         0U
#define MAC_ADDR3                         0U
#define MAC_ADDR4                         0U
#define MAC_ADDR5                         0U

/* Definition of the Ethernet driver buffers size and count */   
#define ETH_RX_BUF_SIZE                   ETH_MAX_PACKET_SIZE /* buffer size for receive               */
#define ETH_TX_BUF_SIZE                   ETH_MAX_PACKET_SIZE /* buffer size for transmit              */
#define ETH_RXBUFNB                       4U       /* 4 Rx buffers of size ETH_RX_BUF_SIZE  */
#define ETH_TXBUFNB                       4U       /* 4 Tx buffers of size ETH_TX_BUF_SIZE  */

/* Section 2: PHY configuration section */

/* Uncomment the line below if you want to use user defined Delay function
   (for precise timing), otherwise default _eth_delay_ function defined within
   the Ethernet driver is used (less precise timing) */
#define USE_Delay

#ifdef USE_Delay
  #include "times.h"
  #define _eth_delay_    mDelay   /* User can provide more timing precise _eth_delay_ function
                                   in this example Systick is configured with an interrupt every 10 ms*/
#else
  #define _eth_delay_    ETH_Delay /* Default _eth_delay_ function with less precise timing */
#endif

/* PHY configuration section **************************************************/
#ifdef USE_Delay
/* PHY Reset delay */
#define PHY_RESET_DELAY    ((uint32_t)2500) // ((uint32_t)0x00000FF)
/* PHY Configuration delay */
#define PHY_CONFIG_DELAY   ((uint32_t)2500) //((uint32_t)0x00000FFF)
/* Delay when writing to Ethernet registers*/
#define ETH_REG_WRITE_DELAY ((uint32_t)0x00000001)
#else
/* PHY Reset delay */
#define PHY_RESET_DELAY    ((uint32_t)0x000FFFFF)
/* PHY Configuration delay */
#define PHY_CONFIG_DELAY   ((uint32_t)0x00FFFFFF)
/* Delay when writing to Ethernet registers*/
#define ETH_REG_WRITE_DELAY ((uint32_t)0x0000FFFF)
#endif

/*******************  PHY Extended Registers section : ************************/

/* DP83848 PHY Address*/ 
#define DP83848_PHY_ADDRESS             0x01U

//#define PHY_READ_TO                     0x0000FFFFU
//#define PHY_WRITE_TO                    0x0000FFFFU

/* Section 3: Common PHY Registers */

//#define PHY_BCR                         ((uint16_t)0x0000)  /*!< Transceiver Basic Control Register   */
//#define PHY_BSR                         ((uint16_t)0x0001)  /*!< Transceiver Basic Status Register    */
 
#define PHY_RESET                       ((uint16_t)0x8000)  /*!< PHY Reset */
#define PHY_LOOPBACK                    ((uint16_t)0x4000)  /*!< Select loop-back mode */
#define PHY_FULLDUPLEX_100M             ((uint16_t)0x2100)  /*!< Set the full-duplex mode at 100 Mb/s */
#define PHY_HALFDUPLEX_100M             ((uint16_t)0x2000)  /*!< Set the half-duplex mode at 100 Mb/s */
#define PHY_FULLDUPLEX_10M              ((uint16_t)0x0100)  /*!< Set the full-duplex mode at 10 Mb/s  */
#define PHY_HALFDUPLEX_10M              ((uint16_t)0x0000)  /*!< Set the half-duplex mode at 10 Mb/s  */
#define PHY_AUTONEGOTIATION             ((uint16_t)0x1000)  /*!< Enable auto-negotiation function     */
#define PHY_RESTART_AUTONEGOTIATION     ((uint16_t)0x0200)  /*!< Restart auto-negotiation function    */
#define PHY_POWERDOWN                   ((uint16_t)0x0800)  /*!< Select the power down mode           */
#define PHY_ISOLATE                     ((uint16_t)0x0400)  /*!< Isolate PHY from MII                 */

#define PHY_AUTONEGO_COMPLETE           ((uint16_t)0x0020)  /*!< Auto-Negotiation process completed   */
#define PHY_LINKED_STATUS               ((uint16_t)0x0004)  /*!< Valid link established               */
#define PHY_JABBER_DETECTION            ((uint16_t)0x0002)  /*!< Jabber condition detected            */
  
/* Section 4: Extended PHY Registers */

#define PHY_SR                          ((uint16_t)0x0010)  /*!< PHY status register Offset                      */
#define PHY_MICR                        ((uint16_t)0x0011)  /*!< MII Interrupt Control Register                  */
#define PHY_MISR                        ((uint16_t)0x0012)  /*!< MII Interrupt Status and Misc. Control Register */
 
// Для DP83848
#define PHY_LINK_INT_STATUS             ((uint16_t)0x2000)  /*!< PHY Link_INT mask                                   */
#define PHY_LINK_STATUS                 ((uint16_t)0x0001)  /*!< PHY Link mask                                   */
#define PHY_SPEED_STATUS                ((uint16_t)0x0002)  /*!< PHY Speed mask                                  */
#define PHY_DUPLEX_STATUS               ((uint16_t)0x0004)  /*!< PHY Duplex mask                                 */

#define PHY_MICR_INT_EN                 ((uint16_t)0x0002)  /*!< PHY Enable interrupts                           */
#define PHY_MICR_INT_OE                 ((uint16_t)0x0001)  /*!< PHY Enable output interrupt events              */

#define PHY_MISR_LINK_INT_EN            ((uint16_t)0x0020)  /*!< Enable Interrupt on change of link status       */
#define PHY_LINK_INTERRUPT              ((uint16_t)0x2000)  /*!< PHY link status interrupt mask                  */


#endif /* __STM32F2xx_HAL_CONF_H */
 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
