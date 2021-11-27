/*
 * fw.h
 *
 *  Created on: 24 окт. 2021 г.
 *      Author: jet
 */
#include <my_mqtt.h>
#include <string.h>

//#include "stm32l1xx_hal_flash_ex.h"
#include "uart.h"
#include "eeprom.h"
#include "gpio_arch.h"
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

//------------------- Function prototype -------------------------
uint32_t hexToL( uint8_t *pStr, uint16_t len );
void FLASH_PageErase(uint32_t PageAddress);
// ---------------------------------------------------------------


void fwInit( void ){
  // Настройка CRC
  RCC->AHBENR |= RCC_AHBENR_CRCEN;

  // TODO: Настройка Флеш
  fwHandle.fwActive = SCB->VTOR > FW_2_START_ADDR;
}


// Обработка принятого манифеста прошивки
void fwManProc( sUartRxHandle * rxh, mqttClient_t * mqttcl ){
  sFwUp * pfw = &fwHandle.fwUp;
  char * bch;                     // Начало обрабатываемого участка строки данных
  char * ech;                     // Конец обрабатываемого участка строки данных

  // Проверка: принят payload полностью
  assert_param( mqttcl->last );

  bch = (char*)(mqttcl->payload);
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

  return;

bad_man:
  pfw->crc = ~0;
  pfw->fwLen = 0;
  // Очистим буфер
  mqttMsgReset( rxh, &SIM800 );

  return;
}


// Процедура обновления прошивки
void fwUpProc( sUartRxHandle * rxh, mqttClient_t * mqttcl ){
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
        // Сброс CRC
        CRC->CR = CRC_CR_RESET;

        // Сотрем данные неактивной прошивке
        stmEeWrite( (uint32_t)&(eeFwh->fw[!fwHandle.fwActive]), (uint32_t*)&tmpfw, sizeof(sFw) );
        // TODO: Запустим стирание флеш для новой прошивки
        /* Unlock the Flash to enable the flash control register access *************/
        if( (FLASH->PECR & FLASH_PECR_PRGLOCK) != RESET ){
          HAL_FLASH_Unlock();
        }
        fwHandle.fwFlashState = FWFLASH_ERASE;
      }
      else if( mqttcl->payOffset == 0 ){
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
        uint16_t len;
        uint32_t addr = fwup->fwStartAddr + fwup->fwOffset;
        uint32_t u32data = *(uint32_t *)&(mqttcl->payload[mqttcl->payOffset]);

        if( mqttcl->last == RESET ){
          // Выравниваем запись по целому слову
          len = (mqttcl->payLen / 4) * 4 ;
        }
        else {
          len = mqttcl->payLen;
        }

        if( mqttcl->payOffset < len ){
          if( (mqttcl->payLen - mqttcl->payOffset) < 4 ){
            // Дополним до 4-х байт '0'
            u32data &= (1 << ((mqttcl->payLen - mqttcl->payOffset) * 8)) - 1;
          }
          // Еще не все записали
          //Считаем CRC
          // Записываем во Флеш
          *(uint32_t *)addr = u32data;
          u32data = *(uint32_t *)addr;
          CRC->DR = u32data;
          mqttcl->payOffset += 4;
          fwup->fwOffset += 4;
        }
        else {
          if( mqttcl->last ){
            fwHandle.fwFlashState = FWFLASH_WRITE_END;
          }
          else {
            // Скопируем незаписанный остаток данных в начало буфера
            mqttcl->payLen -= len;
            memcpy( mqttcl->payload, &(mqttcl->payload[len]), mqttcl->payLen );
          }
        }
      }
      break;
    case FWFLASH_WRITE_END:
      // Вся прошивка записана
      // Залочить Флеш
      FLASH->PECR |= FLASH_PECR_PRGLOCK;
      // Проверка на окончание записи и Проверка CRC
      if( (fwup->fwOffset >= fwup->fwLen)
          && (CRC->DR == fwup->crc) ){
        sFwHandle * eeFwh = (sFwHandle *)FW_HANDLE_ADDR_0;
        sFw tmpfw;
        eFwNum fwact = !(fwHandle.fwActive);

        // TODO: Сохранение данных прошивки в EEPROM
        tmpfw.crc = fwup->crc;
        tmpfw.fwLen = fwup->fwLen;
        tmpfw.fwVer = fwup->fwVer;
        tmpfw.good = SET;
        // Запишем данные новой прошивки на место неактивной
        stmEeWrite( (uint32_t)&(eeFwh->fw[fwact]), (uint32_t*)&tmpfw, sizeof(sFw) );
        fwup->fwUpOk = SET;
        // Перезагрузка после отправки пакета PUBCOMP
      }
      else {
        fwup->crc = 0;
        fwup->fwLen = 0;
        fwup->fwVer = 0;
      }
      fwHandle.fwFlashState = FWFLASH_READY;
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


