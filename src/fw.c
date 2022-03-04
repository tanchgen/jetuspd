/*
 * fw.h
 *
 *  Created on: 24 окт. 2021 г.
 *      Author: jet
 */
#include <string.h>

//#include "stm32l1xx_hal_flash_ex.h"
#include "uart.h"
#include "mqtt.h"
#include "eeprom.h"
#include "events.h"
#include "logger.h"
#include "gpio_arch.h"
#include "usart_arch.h"
#include "gsm.h"
#include "fw.h"

/* Here for now until needed in other places in lwIP */
#ifndef isprint
#define in_range(c, lo, up)  ((uint8_t)c >= lo && (uint8_t)c <= up)
#define isprint(c)           in_range(c, 0x20, 0x7f)
#define isdigit(c)           in_range(c, '0', '9')
#define isxdigit(c)          (isdigit(c) || in_range(c, 'a', 'f') || in_range(c, 'A', 'F'))
#define islower(c)           in_range(c, 'a', 'z')
#define isspace(c)           (c == ' ' || c == '\f' || c == '\n' || c == '\r' || c == '\t' || c == '\v')
#endif

/** IPv4 only: set the IP address given as an u32_t */
#define ip4_addr_set_u32(dest_ipaddr, src_u32) ((dest_ipaddr)->addr = (src_u32))
/** IPv4 only: get the IP address as an u32_t */
#define ip4_addr_get_u32(src_ipaddr) ((src_ipaddr)->addr)

sFwHandle fwHandle;
//uint32_t * pfa;
//uint32_t fa;
struct timer_list fwUpdTimer;
FlagStatus fwUpdFlag = RESET;

//------------------- Function prototype -------------------------
uint32_t ipaddr_addr(const char *cp);
uint32_t hexToL( uint8_t *pStr, uint16_t len );
void FLASH_PageErase(uint32_t PageAddress);
int ipaddr_aton(const char *cp, ip_addr_t *addr);
uint32_t htonl(uint32_t n) ;
// ---------------------------------------------------------------


static void fwUpdTout( uintptr_t arg ){
  (void)arg;

  // Выключение CRC
  RCC->AHBENR &= ~RCC_AHBENR_CRCEN;

  // Очистим буфер
  mqttMsgReset( simHnd.rxh, &SIM800 );
  fwUpdFlag = RESET;

#if DEBUG_GSM_TRACE
  trace_puts("FW Upd TOUT");
#endif

  // Перезагрузим SIM: может сервер отсоединен, а мы и не знаем, потому что не отслеживали
  gsmStRestart = GSM_OFF;
  gsmReset = SIM_RESET;
  gsmRun = RESET;
}

void fwInit( void ){
  fwHandle.fwActive = SCB->VTOR > FW_1_START_ADDR;

  timerSetup( &fwUpdTimer, fwUpdTout, (uintptr_t)NULL );
}


// Обработка принятого манифеста прошивки
void fwManProc( sUartRxHandle * rxh, mqttReceive_t * mqttrx ){
  sFwUp * pfw = &fwHandle.fwUp;
  char * bch;                     // Начало обрабатываемого участка строки данных
  char * ech;                     // Конец обрабатываемого участка строки данных

  bch = (char*)(rxh->rxFrame + mqttrx->payOffset);
  // Читаем версию обновления
  if( strstr( bch, "{\"fwupd\":\"") == NULL ){
    goto bad_man;
  }
  else {
    unsigned int v, sv, r;
    // Переносим начало строки
    bch += 10;
    ech = strstr( bch, "\"" );
    // Ограничиваем строку нулем
    *ech = '\0';
    sscanf( bch, "%u.%u.%u", &v, &sv, &r );
    pfw->fwVer = (v << 16) | (sv << 8) | r;
    if( pfw->fwVer == 0 ){
      goto bad_man;
    }
    // Переходим на следующий кусок
    bch = ech + 2;
  }

  // Читаем длину обновления
  if( strstr( bch, "\"lght\":") == 0 ){
    goto bad_man;
  }
  else {
    // Переносим начало строки
    bch += 7;
    if( (pfw->fwLen = atoi( bch )) == 0 ){
      goto bad_man;
    }
  }

  // Читаем CRC обновления
  if( (bch = strstr( bch, "\"crc\":\"")) == 0 ){
    goto bad_man;
  }
  else {
    // Переносим начало строки
    bch += 7;
    if( (pfw->crc = hexToL( (uint8_t *)bch, 8 )) == (uint32_t)-1 ){
      goto bad_man;
    }
  }
  // Начиннаем принимать с начала
  pfw->fwOffset = 0;
  pfw->fwUpOk = RESET;
  rxh->rxProcFlag = RESET;
  // Очистим буфер
  mqttMsgReset( rxh, &SIM800 );

#if DEBUG_GSM_TRACE
  trace_puts("FW man OK");
#endif

  return;

bad_man:
#if DEBUG_GSM_TRACE
  trace_puts("FW man BAD");
#endif
  pfw->crc = ~0;
  pfw->fwLen = 0;
  // Очистим буфер
  mqttMsgReset( rxh, &SIM800 );

  return;
}


// Процедура обновления прошивки
void fwUpProc( sUartRxHandle * rxh, mqttReceive_t * mqttrx ){
  sFwUp * fwup = &fwHandle.fwUp;

  if( fwup->fwLen == 0 ){
    mqttMsgReset( rxh, &SIM800 );
  }

  switch( fwHandle.fwFlashState ){
    case FWFLASH_READY:
      if( fwup->fwOffset == 0 ){
        sFwHandle * eeFwh = (sFwHandle *)FW_HANDLE_ADDR_0;
        sFw tmpfw = {0};
//        uint32_t fwaddr;

        // Это первый фрагмент прошивки
        fwup->fwStartAddr = (fwHandle.fwActive)? FW_1_START_ADDR : FW_2_START_ADDR;
        fwup->fwEndAddr = fwup->fwStartAddr + ((fwHandle.fwActive)? FW_1_SIZE : FW_2_SIZE);
        // Включение CRC
        RCC->AHBENR |= RCC_AHBENR_CRCEN;
        // Сброс CRC
        CRC->CR = CRC_CR_RESET;

        // Сотрем данные неактивной прошивк
        stmEeWrite( (uint32_t)&(eeFwh->fw[!fwHandle.fwActive]), (uint32_t*)&tmpfw, sizeof(sFw) );
        // TODO: Запустим стирание флеш для новой прошивки
        /* Unlock the Flash to enable the flash control register access *************/
        if( (FLASH->PECR & FLASH_PECR_PRGLOCK) != RESET ){
          HAL_FLASH_Unlock();
        }
        fwHandle.fwFlashState = FWFLASH_ERASE;
      }
      else if( mqttrx->payOffset == 0 ){
        // Нужно записать очередной фрагмент
        //fwup->fwOpAddr = fwup->fwStartAddr;
        fwHandle.fwFlashState = FWFLASH_WRITE_START;
      }
      break;
    case FWFLASH_READ:
      break;
    case FWFLASH_ERASE:
      if( (FLASH->SR & FLASH_SR_BSY) == RESET ){
        uint32_t addr = fwup->fwStartAddr + fwup->fwOffset;
        uint32_t fwend = (fwup->fwStartAddr + fwup->fwLen);
        if( addr < fwend ){
          // Еще не всю область прошивки стерли
          assert_param( FLASH_PAGE_SIZE <= (fwup->fwEndAddr - addr));
          FLASH_PageErase( addr );
          fwup->fwOffset += FLASH_PAGE_SIZE;
        }
        else {
          // Очистили всю нужную область
          FLASH->PECR &= ~(FLASH_PECR_PROG | FLASH_PECR_ERASE);
          fwup->fwOffset = 0;
          fwHandle.fwFlashState = FWFLASH_WRITE_START;
        }
      }
      break;
    case FWFLASH_WRITE_START:
      if( (FLASH->SR & FLASH_FLAG_BSY) == RESET ){
        uint32_t addr = fwup->fwStartAddr + fwup->fwOffset;
        if( mqttrx->payOffset < rxh->frame_offset){
          if( (rxh->frame_offset - mqttrx->payOffset) < 4 ){
            // Дополним до 4-х байт '0'
            rxh->rxFrame[rxh->frame_offset] = 0;
            rxh->rxFrame[rxh->frame_offset+1] = 0;
            rxh->rxFrame[rxh->frame_offset+2] = 0;
          }
          // Еще не все записали
          //Считаем CRC
          volatile uint32_t u32data = *(uint32_t *)&(rxh->rxFrame[mqttrx->payOffset]);
          // Записываем во Флеш
          *(uint32_t *)addr = u32data;
          u32data = *(uint32_t *)addr;
          CRC->DR = u32data;
          mqttrx->payOffset += 4;
          fwup->fwOffset += 4;
        }
        else {
          if( fwup->fwOffset >= fwup->fwLen ){
            fwHandle.fwFlashState = FWFLASH_WRITE_END;
          }
          else {
            // Очистим буфер
            mqttBufClean( rxh, &SIM800 );
            mqttrx->payloadLen = fwup->fwLen - fwup->fwOffset;
            trace_printf( "fwOff: %lu\n", fwup->fwOffset );
          }
        }
      }
      break;
    case FWFLASH_WRITE_END:
      // TODO: Проверка на окончание записи
      assert_param( fwup->fwOffset >= fwup->fwLen );
      // Вся прошивка записана
      // Залочить Флеш
      FLASH->PECR |= FLASH_PECR_PRGLOCK;
      // Проверка CRC
      if( CRC->DR == fwup->crc ){
        sFwHandle * eeFwh = (sFwHandle *)FW_HANDLE_ADDR_0;
        sFw tmpfw;
        eFwNum fwact = !(fwHandle.fwActive);

#if DEBUG_GSM_TRACE
        trace_puts("FW Upd OK");
#endif
        tmpfw.crc = fwup->crc;
        tmpfw.fwLen = fwup->fwLen;
        tmpfw.fwVer = fwup->fwVer;
        tmpfw.good = SET;
        // Запишем данные новой прошивки на место неактивной
        stmEeWrite( (uint32_t)&(eeFwh->fw[fwact]), (uint32_t*)&tmpfw, sizeof(sFw) );
        evntFlags.fwUpd = SET;
        fwup->fwUpOk = SET;
        // Перезагрузка после отправки пакета PUBCOMP
      }
      else {
        trace_printf("0x%08x\n", CRC->DR );
        fwup->crc = 0;
        fwup->fwLen = 0;
        fwup->fwVer = 0;
      }
      // Выключение CRC
      RCC->AHBENR &= ~RCC_AHBENR_CRCEN;

      fwHandle.fwFlashState = FWFLASH_READY;
      // Останавливаем таймер таймаута получения прошивки
      timerDel( &fwUpdTimer );
      fwUpdFlag = RESET;
      // Очистим буфер
      mqttMsgReset( rxh, &SIM800 );
      break;
    case FWFLASH_SKIP:
      // Пропускаем фрагмент
      // Очистим буфер
      mqttMsgReset( rxh, &SIM800 );
      break;
    default:
      //Сюда не должны попадать
      break;
  }

}


/**
 * Ascii internet address interpretation routine.
 * The value returned is in network order.
 *
 * @param cp IP address in ascii represenation (e.g. "127.0.0.1")
 * @return ip address in network order
 */
uint32_t ipaddr_addr(const char *cp) {
  ip_addr_t val;

  if (ipaddr_aton(cp, &val)) {
    return ip4_addr_get_u32(&val);
  }
  return (0);
}

/**
 * Check whether "cp" is a valid ascii representation
 * of an Internet address and convert to a binary address.
 * Returns 1 if the address is valid, 0 if not.
 * This replaces inet_addr, the return value from which
 * cannot distinguish between failure and a local broadcast address.
 *
 * @param cp IP address in ascii represenation (e.g. "127.0.0.1")
 * @param addr pointer to which to save the ip address in network order
 * @return 1 if cp could be converted to addr, 0 on failure
 */
int ipaddr_aton(const char *cp, ip_addr_t *addr) {
  uint32_t val;
  uint8_t base;
  char c;
  uint32_t parts[4];
  uint32_t *pp = parts;

  c = *cp;
  for (;;) {
    /*
     * Collect number up to ``.''.
     * Values are specified as for C:
     * 0x=hex, 0=octal, 1-9=decimal.
     */
    if (!isdigit(c))
      return (0);
    val = 0;
    base = 10;
    if (c == '0') {
      c = *++cp;
      if (c == 'x' || c == 'X') {
        base = 16;
        c = *++cp;
      } else
        base = 8;
    }
    for (;;) {
      if (isdigit(c)) {
        val = (val * base) + (int)(c - '0');
        c = *++cp;
      } else if (base == 16 && isxdigit(c)) {
        val = (val << 4) | (int)(c + 10 - (islower(c) ? 'a' : 'A'));
        c = *++cp;
      } else
        break;
    }
    if (c == '.') {
      /*
       * Internet format:
       *  a.b.c.d
       *  a.b.c   (with c treated as 16 bits)
       *  a.b (with b treated as 24 bits)
       */
      if (pp >= parts + 3) {
        return (0);
      }
      *pp++ = val;
      c = *++cp;
    } else
      break;
  }
  /*
   * Check for trailing characters.
   */
  if (c != '\0' && !isspace(c)) {
    return (0);
  }
  /*
   * Concoct the address according to
   * the number of parts specified.
   */
  switch (pp - parts + 1) {

  case 0:
    return (0);       /* initial nondigit */

  case 1:             /* a -- 32 bits */
    break;

  case 2:             /* a.b -- 8.24 bits */
    if (val > 0xffffffUL) {
      return (0);
    }
    val |= parts[0] << 24;
    break;

  case 3:             /* a.b.c -- 8.8.16 bits */
    if (val > 0xffff) {
      return (0);
    }
    val |= (parts[0] << 24) | (parts[1] << 16);
    break;

  case 4:             /* a.b.c.d -- 8.8.8.8 bits */
    if (val > 0xff) {
      return (0);
    }
    val |= (parts[0] << 24) | (parts[1] << 16) | (parts[2] << 8);
    break;
  default:
    assert_param(0);
    break;
  }
  if (addr) {
    ip4_addr_set_u32(addr, htonl(val));
  }
  return (1);
}


uint32_t hexToL( uint8_t *pStr, uint16_t len ){
  uint32_t id = 0;
  int32_t ret = len;
  while( ret-- ){
    id <<= 4;
    uint8_t ch = *(pStr++);
    if( (ch >='0') && (ch <='9') ) {
      id += ch - '0';
    }
    else if( (ch >= 'a') && (ch <= 'f') ){
      id += ch - 'a'+10;
    }
    else if( (ch >= 'A') && (ch <= 'F') ){
      id += ch - 'A'+10;
    }
    else {
      return (uint32_t)-1;
    }
  }
  return id;
}


uint32_t htonl(uint32_t n) {
  return __REV( n );
}
