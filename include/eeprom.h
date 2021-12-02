/*
 * eeprom.h
 *
 *  Created on: 19 сент. 2021 г.
 *      Author: jet
 */

#ifndef EEPROM_H_
#define EEPROM_H_

#include "stm32l1xx.h"

#define EEPROM_PAGESIZE_M95040          16      /* EEPROM M95040-R used */

/* EEPROM BSP return values */
#define EEPROM_OK                       0
#define EEPROM_FAIL                     1
#define EEPROM_TIMEOUT                  2

/* EEPROM BSP devices definition list supported */
#define BSP_EEPROM_M24LR64              1       /* RF I2C EEPROM M24LR64 */
#define BSP_EEPROM_M95040               3       /* SPI EEPROM M95040-R */

/* Maximum number of trials for EEPROM_I2C_WaitEepromStandbyState() function */
#define EEPROM_MAX_TRIALS               300

extern uint32_t EEPROMPageSize;

// Расспределение памяти
// Адрес данных Буфера архива датчиков
#define EE_SENS_ARCH_BUF_ADDR        (FLASH_EEPROM_BASE + 0x000)
// Адрес данных Буфера архива событий
#define EE_EVNT_ARCH_BUF_ADDR        (FLASH_EEPROM_BASE + 0x040)
// Адрес данных прошивки во Флеш
#define EE_FW_HANDLE_ADDR_0          (FLASH_EEPROM_BASE + 0x080)
// Адрес зранения конфигурации УСПД
#define EE_CFG_ADDR                  (FLASH_EEPROM_BASE + 0x100)


void stmEeInit( void );
HAL_StatusTypeDef   stmEeWrite( uint32_t addr, uint32_t * data, uint32_t datalen);
HAL_StatusTypeDef   stmEeWrite( uint32_t addr, uint32_t * data, uint32_t datalen);

#endif /* EEPROM_H_ */
