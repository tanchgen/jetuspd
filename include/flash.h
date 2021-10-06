/*
 * flash.h
 *
 *  Created on: 23 сент. 2021 г.
 *      Author: jet
 */

#ifndef FLASH_H_
#define FLASH_H_

/* LINK FLASH SPI */
#define FLASH_CMD_WREN         0x06  /*!< Write enable instruction */
#define FLASH_CMD_WRDI         0x04  /*!< Write disable instruction */
#define FLASH_CMD_RDSR         0x05  /*!< Read Status Register instruction  */
#define FLASH_CMD_WRSR         0x01  /*!< Write Status Register instruction */
#define FLASH_CMD_WRITE        0x02  /*!< Write to Memory instruction */
#define FLASH_CMD_READ         0x03  /*!< Read from Memory instruction */
#define FLASH_CMD_SE           0x20  /*!< Sector Erase */
#define FLASH_CMD_RDSCUR       0x2B  /*!< Read Security Register */
#define FLASH_CMD_RDID         0x9F  /*!< Read ID*/


#define FLASH_WIP_FLAG         0x01  /*!< Write In Progress (WIP) flag */
#define FLASH_WEL_FLAG         0x02  /*!< Write Enable Latch (WEL) flag */
#define FLASH_EFAIL_FLAG       0x20  /*!< Program failed flag */
#define FLASH_PFAIL_FLAG       0x40  /*!< Erase failed flag */

typedef enum {
  LOG_BUF_NULL,
  LOG_BUF_SENS,
  LOG_BUF_EVNT,
} eLogBufType;

typedef enum {
  FLASH_READY,
  FLASH_READ_START,
  FLASH_WRITE_START,
  FLASH_WRITE_EN,
  FLASH_WRITE_END,
  FLASH_WRITE_OK,
  FLASH_BUSY
} eFlashState;

/** Структура дескриптора FLASH. */
typedef struct {
  sSpiHandle flashSpi;
  eFlashState state;
  uint16_t writeCount;                /** Количество незаписанных байт */
  FlagStatus sectorClear;
  FlagStatus flashEmpty;                 /** Требуется обновление конфигурации */
  FlagStatus readSensQuery;              // Запрос на чтение журнала счетчиков
  FlagStatus readEvntQuery;               // Запрос на чтение журнала событий
  eLogBufType lbType;               // Тип буфера (SENS/EVNT) оп. чтения/записи
  sLogRec * rec;                // Указатель на запись(си)  оп. чтения/записи
  uint16_t  quant;              // Количество обрабатываемых записей
} sFlashDev;


#define FLASH_PAGESIZE_M95040          16      /* EEPROM M95040-R used */

/* EEPROM BSP return values */
#define FLASH_OK                       0
#define FLASH_FAIL                     1
#define FLASH_TIMEOUT                  2

#define SENS_HEADER_ADDR         0
#define EVNT_HEADER_ADDR         64

#define SENS_FLASH_SIZE         ((uint32_t)512 * 1024)
#define SENS_LOG_START_ADDR     0
#define SENS_LOG_SIZE           (((SENS_FLASH_SIZE - SENS_LOG_START_ADDR) + (sizeof(logBuf_t) - 1)) / sizeof(logBuf_t))

#define EVNT_LOG_START_ADDR     (SENS_LOG_SIZE * sizeof(logBuf_t))
#define EVNT_FLASH_SIZE         (((uint32_t)1024 * 1024) - SENS_LOG_SIZE)
#define EVNT_LOG_SIZE           (((EVNT_FLASH_SIZE - EVNT_LOG_START_ADDR) + (sizeof(logBuf_t) - 1)) / sizeof(logBuf_t))


extern sFlashDev flashDev;

extern logBuf_t flashSensBuffer;
extern logBuf_t flashEvntBuffer;

void flashInit( void );

uint8_t flashBuf_Init(logBuf_t* Buffer, sLogRec * startAddr, uint16_t Size );
HAL_StatusTypeDef  flashBufSave( logBuf_t* Buffer );

#endif /* FLASH_H_ */
