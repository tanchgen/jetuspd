/*
 * pos.c
 *
 *  Created on: 14 нояб. 2021 г.
 *      Author: jet
 */

/*
As usual, raw API for lwIP means the lightweight API which *MUST* only be used
for NO_SYS=1 systems or called inside lwIP core thread for NO_SYS=0 systems.
*/

#include "pos.h"
#include "led.h"

#include "usart_arch.h"
#include "netconf.h"
#include "lwip/init.h"
#include "MQTTSim800.h"
#include "gsm.h"

/* The PPP control block */
ppp_pcb *ppp;

/* The PPP IP interface */
struct netif sim_netif;

static uint32_t ppp_output_cb(ppp_pcb *pcb, u8_t *data, u32_t len, void *ctx);
static void ppp_status_cb(ppp_pcb *pcb, int err_code, void *ctx);

/**
 * Deinitialization SIM_PPP.
 * @param NONE
 * @return error status, 0 - OK
 */
int pppOff(void) {
  int error = 0;

  _disconnect( SET );

  error += gsmSendCommand("ATE1\r\n", "OK\r\n", CMD_DELAY_2, NULL );
  return error;
}



/**
  * @brief  Инициализация интерфейса подсистемы сетевого интерфейса.
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void pppInit( void ){
      ip_addr_t ipaddr;
      ip_addr_t netmask;
      ip_addr_t gw;

      /* Настраиваем сетевой интерфейс. */

      /* Initialize the LwIP stack */
      lwip_init();

#if USE_DHCP
      ip_addr_set_zero_ip4(&ipaddr);
      ip_addr_set_zero_ip4(&netmask);
      ip_addr_set_zero_ip4(&gw);
#else
      IP_ADDR4(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
      IP_ADDR4(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
      IP_ADDR4(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
      ipsetFlag = SET;
#endif /* USE_DHCP */

      // ** After first successful initialization create PPP control block
      ppp = pppos_create(&sim_netif, ppp_output_cb, ppp_status_cb, NULL);

      /*
       * Initiate PPP client connection
       * ==============================
       */

      /* Set this interface as default route */
      ppp_set_default(ppp);


      /*
       * Basic PPP client configuration. Can only be set if PPP session is in the
       * dead state (i.e. disconnected). We don't need to provide thread-safe
       * equivalents through PPPAPI because those helpers are only changing
       * structure members while session is inactive for lwIP core. Configuration
       * only need to be done once.
       */

      /* Ask the peer for up to 2 DNS server addresses. */
      ppp_set_usepeerdns(ppp, 1);

//      /* Auth configuration, this is pretty self-explanatory */
//      ppp_set_auth(ppp, PPPAUTHTYPE_ANY, "login", "password");

}

/*
 * PPP status callback
 * ===================
 *
 * PPP status callback is called on PPP status change (up, down, …) from lwIP
 * core thread
 */

/* PPP status callback example */
static void ppp_status_cb(ppp_pcb *pcb, int err_code, void *ctx) {
  struct netif *pppif = ppp_netif(pcb);
  LWIP_UNUSED_ARG(ctx);

  switch(err_code) {
    case PPPERR_NONE: {
#if LWIP_DNS
      const ip_addr_t *ns;
#endif /* LWIP_DNS */
      printf("status_cb: Connected\n");
#if PPP_IPV4_SUPPORT
      printf("   our_ipaddr  = %s\n", ipaddr_ntoa(&pppif->ip_addr));
      printf("   his_ipaddr  = %s\n", ipaddr_ntoa(&pppif->gw));
      printf("   netmask     = %s\n", ipaddr_ntoa(&pppif->netmask));
#if LWIP_DNS
      ns = dns_getserver(0);
      printf("   dns1        = %s\n", ipaddr_ntoa(ns));
      ns = dns_getserver(1);
      printf("   dns2        = %s\n", ipaddr_ntoa(ns));
#endif /* LWIP_DNS */
#endif /* PPP_IPV4_SUPPORT */
#if PPP_IPV6_SUPPORT
      printf("   our6_ipaddr = %s\n", ip6addr_ntoa(netif_ip6_addr(pppif, 0)));
#endif /* PPP_IPV6_SUPPORT */
      SIM800.mqttServer.pppconn = SET;
      break;
    }
    case PPPERR_PARAM: {
      printf("status_cb: Invalid parameter\n");
      break;
    }
    case PPPERR_OPEN: {
      printf("status_cb: Unable to open PPP session\n");
      break;
    }
    case PPPERR_DEVICE: {
      printf("status_cb: Invalid I/O device for PPP\n");
      break;
    }
    case PPPERR_ALLOC: {
      printf("status_cb: Unable to allocate resources\n");
      break;
    }
    case PPPERR_USER: {
      printf("status_cb: User interrupt\n");
      SIM800.mqttServer.pppconn = RESET;
      break;
    }
    case PPPERR_CONNECT: {
      printf("status_cb: Connection lost\n");
      SIM800.mqttServer.pppconn = RESET;
      break;
    }
    case PPPERR_AUTHFAIL: {
      printf("status_cb: Failed authentication challenge\n");
      break;
    }
    case PPPERR_PROTOCOL: {
      printf("status_cb: Failed to meet protocol\n");
      break;
    }
    case PPPERR_PEERDEAD: {
      printf("status_cb: Connection timeout\n");
      break;
    }
    case PPPERR_IDLETIMEOUT: {
      printf("status_cb: Idle Timeout\n");
      break;
    }
    case PPPERR_CONNECTTIME: {
      printf("status_cb: Max connect time reached\n");
      break;
    }
    case PPPERR_LOOPBACK: {
      printf("status_cb: Loopback detected\n");
      break;
    }
    default: {
      printf("status_cb: Unknown error code %d\n", err_code);
      break;
    }
  }

/*
 * This should be in the switch case, this is put outside of the switch
 * case for example readability.
 */

  if (err_code == PPPERR_NONE) {
    return;
  }

  /* ppp_close() was previously called, don't reconnect */
  if (err_code == PPPERR_USER) {
    /* ppp_free(); -- can be called here */
    return;
  }

  /*
   * Try to reconnect in 30 seconds, if you need a modem chatscript you have
   * to do a much better signaling here ;-)
   */
  ppp_connect(pcb, 30);
  /* OR ppp_listen(pcb); */
}


/*
 * Creating a new PPPoS session
 * ============================
 *
 * In lwIP, PPPoS is not PPPoSONET, in lwIP PPPoS is PPPoSerial.
 */

/*
 * PPPoS serial output callback
 *
 * ppp_pcb, PPP control block
 * data, buffer to write to serial port
 * len, length of the data buffer
 * ctx, optional user-provided callback context pointer
 *
 * Return value: len if write succeed
 */
static uint32_t ppp_output_cb(ppp_pcb *pcb, u8_t *data, u32_t len, void *ctx) {
  (void)pcb;
  (void)ctx;

  return simUartSend( data, len );
}


/*
 * Closing PPP connection
 * ======================
 */

/*
 * Initiate the end of the PPP session, without carrier lost signal
 * (nocarrier=0), meaning a clean shutdown of PPP protocols.
 * You can call this function at anytime.
 */
//u8_t nocarrier = 0;
//ppp_close(ppp, nocarrier);
/*
 * Then you must wait your status_cb() to be called, it may takes from a few
 * seconds to several tens of seconds depending on the current PPP state.
 */

/*
 * Freeing a PPP connection
 * ========================
 */

/*
 * Free the PPP control block, can only be called if PPP session is in the
 * dead state (i.e. disconnected). You need to call ppp_close() before.
 */
//ppp_free(ppp);



/*
5 Notify phase callback (PPP_NOTIFY_PHASE)
==========================================

Notify phase callback, enabled using the PPP_NOTIFY_PHASE config option, let
you configure a callback that is called on each PPP internal state change.
This is different from the status callback which only warns you about
up(running) and down(dead) events.

Notify phase callback can be used, for example, to set a LED pattern depending
on the current phase of the PPP session. Here is a callback example which
tries to mimic what we usually see on xDSL modems while they are negotiating
the link, which should be self-explanatory:
*/

void ppp_notify_phase_cb(ppp_pcb *pcb, u8_t phase, void *ctx) {
  (void)pcb;
  (void)ctx;
  switch (phase) {

  /* Session is down (either permanently or briefly) */
  case PPP_PHASE_DEAD:
    ledOff(LED_G, 0);
    break;

  /* We are between two sessions */
  case PPP_PHASE_HOLDOFF:
    ledToggleSet(LED_G, LED_SLOW_TOGGLE_TOUT, LED_SLOW_TOGGLE_TOUT, 0, 0 );
    break;

  /* Session just started */
  case PPP_PHASE_INITIALIZE:
    ledToggleSet(LED_G, LED_TOGGLE_TOUT, LED_TOGGLE_TOUT, 0, 0 );
    break;

  /* Session is running */
  case PPP_PHASE_RUNNING:
    ledToggleSet(LED_G, LED_BLINK_ON_TOUT, LED_BLINK_OFF_TOUT, 0, 0 );
    break;

  default:
    break;
  }
}


/**
* @brief  Returns the current time in milliseconds
*         when LWIP_TIMERS == 1 and NO_SYS == 1
* @param  None
* @retval Time
*/
u32_t sys_now(void){
  return mTick;
}

/**
* @brief  Returns the current time in milliseconds
*         when LWIP_TIMERS == 1 and NO_SYS == 1
* @param  None
* @retval Time
*/
u32_t sys_jiffies(void){
  return HAL_GetTick();
}
