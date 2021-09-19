/*
 * logger.h
 *
 *  Created on: 13 апр. 2021 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include <stdbool.h>
#include <sys/cdefs.h>

#include "stm32l1xx.h"

#include "spi.h"

#define SMALLEST_BUFFER_SIZE  20
#define SMALL_BUFFER_SIZE     (SMALLEST_BUFFER_SIZE * 2)
#define BIG_BUFFER_SIZE       (SMALL_BUFFER_SIZE * 2)


#define FLASH_SPI              SPI1
#define FLASH_SPI_IRQn         SPI1_IRQn
#define FLASH_DMA_RX_Stream   DMA2_Stream0
#define FLASH_DMA_RX_TCIF     DMA_IT_TCIF0
#define FLASH_DMA_RX_IRQn     DMA2_Stream0_IRQn
#define FLASH_DMA_TX_Stream   DMA2_Stream3
#define FLASH_DMA_TX_TCIF     DMA_IT_TCIF3
#define FLASH_DMA_TX_IRQn     DMA2_Stream3_IRQn

/* MX25R3235F SPI FLASH defines */
#define FLASH_WREN  0x06  /*!< Write Enable */
#define FLASH_WRDI  0x04  /*!< Write Disable */
#define FLASH_RDSR  0x05  /*!< Read Status Register */
#define FLASH_WRSR  0x01  /*!< Write Status Register */
#define FLASH_READ  0x03  /*!< Read from Memory Array */
#define FLASH_WRITE 0x02  /*!< Write to Memory Array */
#define FLASH_RDID  0x83  /*!< Read to ID */

#define FLASH_WIP_FLAG        0x01  /*!< Write In Progress (WIP) flag */

#define FLASH_PAGE_SIZE     32    /*!< FLASH Buffer size. Setup to your needs */

#define FLASH_SIZE               ((uint32_t)128 * 1024)
#define FLASH_LOG_START_ADDR     ((uint32_t)128)
#define FLASH_LOG_SIZE           (((FLASH_SIZE - FLASH_LOG_START_ADDR) + (sizeof(logBuf_t) - 1)) / sizeof(logBuf_t))
#define FLASH_HEADER_ADDR        0x0
#define FLASH_HEADER_ADDR        0x0

/**
 * @brief FLASH Operations statuses
 */
typedef enum {
    FLASH_STATUS_PENDING,
    FLASH_STATUS_COMPLETE,
    FLASH_STATUS_ERROR
} eFlashOperations;

typedef enum {
  LOG_ERR_PRM_OFF_LOW,
  LOG_ERR_PRM_OFF_HI,
  LOG_ERR_PRM_ON_LOW,
  LOG_ERR_PRM_ON_HI,
  LOG_ERR_FLAG,
} eLogErrId;

// Идентификатор логируемого устройства
typedef enum {
  DEVID_NULL,
  DEVID_STATUS1,
  DEVID_STATUS2,
  DEVID_STATUS3,
  DEVID_PS,
  DEVID_PSU,
  DEVID_TMP513,
  DEVID_LTM,
  DEVID_FPGA,
  DEVID_PLL,
  DEVID_FAN_STATUS,
  DEVID_FAN_PWM,
  DEVID_FAN,
  DEVID_ADC,
  DEVID_NUM
} eDevId;


// Структура записи FLASH
typedef struct __packed {
  uint32_t utime;           // Unix-time
  uint32_t isincCount;      // Счетчик импульсов
} sLogRec;

//#include "buffer.log.h"

typedef union {
  struct {
    uint8_t header[4];
    uint8_t data[sizeof(sLogRec) * BIG_BUFFER_SIZE];
  };
  uint8_t u8buf;
} uFlashXferBuffer;

/** Структура дескриптора FLASH. */
typedef struct {
  union {
    struct {
      uint16_t  dataNok: 1;                   /* Данные не готовы */
      uint16_t  statusIfaceErr : 1;     /** Бит состояния: Ошибка коммуникации с PLL */
    };
    uint16_t status;
  };
  sSpiHandle flashSpi;
  uint16_t writeCount;                /** Количество незаписанных байт */
  FlagStatus flashEmpty;                 /** Требуется обновление конфигурации */
} sFlashDev;

extern sFlashDev flash;
extern sFlashDev logDev;

eFlashOperations FLASH_SPI_WriteBuffer(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite);
eFlashOperations FLASH_WritePage(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite);
eFlashOperations FLASH_SPI_ReadBuffer(uint8_t* pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead);
uint8_t FLASH_SPI_WaitStandbyState( sSpiHandle * hspi );

/* Low layer functions */
uint8_t FLASH_SendByte(uint8_t byte);
void flashWriteEnable( sSpiHandle * hspi );
void flashWriteDisable( sSpiHandle * hspi );
void sFLASH_WriteStatusRegister(uint8_t regval);
uint8_t sFLASH_ReadStatusRegister(void);

void FLASH_SPI_SendInstruction(sSpiHandle * hspi, uint8_t *instruction, uint8_t size);
//void  FLASH_SPI_ReadStatusByte(SPI_HandleTypeDef SPIe, uint8_t *statusByte );

int flashHwTest( void );


#endif /* LOGGER_H_ */
