/*
 * fw.h
 *
 *  Created on: 24 окт. 2021 г.
 *      Author: jet
 */

#ifndef FW_H_
#define FW_H_

#include "sys/cdefs.h"
#include "stm32l1xx.h"

//#define FLASH_SIZE              (FLASH_END - FLASH_BASE + 1)
#define BOOTLOADER_SIZE         ((uint32_t)0x2000)
#define FW_1_START_ADDR         (FLASH_BASE + BOOTLOADER_SIZE)
#define FW_1_SIZE               ((FLASH_SIZE - BOOTLOADER_SIZE) / 2)
#define FW_1_END                (FW_1_START_ADDR + FW_1_SIZE - 1)
#define FW_1_VER_ADDR           (FW_1_START_ADDR + FW_1_SIZE - 4)
#define FW_2_START_ADDR         (FW_1_END + 1)
#define FW_2_SIZE               FW_1_SIZE
#define FW_2_END                (FW_2_START_ADDR + FW_2_SIZE - 1)
#define FW_2_VER_ADDR           (FW_2_START_ADDR + FW_2_SIZE - 4)

#define EE_FW_HANDLE_ADDR_0          (FLASH_EEPROM_BASE + 0x80)

#define FW_HANDLE_ADDR_0       (0x80UL)

#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE         (256U)
#endif // FLASH_PAGE_SIZE

/* This is the aligned version of ip_addr_t,
   used as local variable, on the stack, etc. */
struct ip_addr {
  uint32_t addr;
};

typedef struct ip_addr ip_addr_t;

typedef enum{
  FW_1 = 0,       // Структура прошивки №1
  FW_2 = 1,       // Структура прошивки №2
  FW_NUM
} eFwNum;

typedef enum {
  FWFLASH_READY,
  FWFLASH_READ,
  FWFLASH_ERASE,
  FWFLASH_WRITE_START,
  FWFLASH_WRITE_END,
  FWFLASH_SKIP,
} eFwFlashState;


typedef struct __aligned(4) {
  uint32_t fwVer;
  uint32_t fwLen;
  uint32_t crc;
  FlagStatus good;
} sFw;

typedef struct {
  uint32_t fwStartAddr;
  uint32_t fwEndAddr;
  uint32_t fwOffset;
  uint32_t fwLen;
  uint32_t fwVer;
  uint32_t crc;
  FlagStatus fwUpOk;
} sFwUp;

typedef struct {
  sFw fw[FW_NUM];
  sFwUp fwUp;
  eFwNum fwActive;
  eFwFlashState fwFlashState;
} sFwHandle;

#endif /* FW_H_ */
