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

#define FLASH_WIP_FLAG      0x01  /*!< Write In Progress (WIP) flag */

//#define FLASH_PAGE_SIZE     256    /*!< FLASH Buffer size. Setup to your needs */
#define FLASH_REC_SIZE      32

//#define FLASH_SIZE               ((uint32_t)128 * 1024)
//#define FLASH_LOG_START_ADDR     ((uint32_t)128)
//#define FLASH_LOG_SIZE           (((FLASH_SIZE - FLASH_LOG_START_ADDR) + (sizeof(logBuf_t) - 1)) / sizeof(logBuf_t))
//#define FLASH_HEADER_ADDR        0x0
//#define FLASH_HEADER_ADDR        0x0

#define EE_SIZE               ((uint32_t)4 * 1024)
#define EE_LOG_START_ADDR     ((uint32_t)64)
#define EE_LOG_SIZE           (((EE_SIZE - EE_LOG_START_ADDR) + (sizeof(logBuf_t) - 1)) / sizeof(logBuf_t))
#define EE_HEADER_ADDR        0x0
#define EE_HEADER_ADDR        0x0

#define LOG_WRITE_TOUT        (uint32_t)10    //  Интервал записи параметров в ЛОГ в сек

/**
 * @brief FLASH Operations statuses
 */
typedef enum {
    FLASH_STATUS_PENDING,
    FLASH_STATUS_COMPLETE,
    FLASH_STATUS_ERROR
} eFlashOperations;

typedef enum {
  DEVID_ISENS_1,
  DEVID_ISENS_2,
  DEVID_ISENS_3,
  DEVID_ISENS_4,
  DEVID_ISENS_5,
  DEVID_ISENS_6,    // срабатывание датчика протечки
  DEVID_SB_1,       // нажатие кнопки sb1
  DEVID_SB_2,       // открытие крышки прибора / отрыв от стены
  DEVID_VBAT,       // Аномальный расход
  DEVID_NAMUR_1,    // разрыв линии NAMUR SENS_1
  DEVID_NAMUR_2,    // разрыв линии NAMUR SENS_1
  DEVID_NAMUR_3,    // разрыв линии NAMUR SENS_1
  DEVID_NAMUR_4,    // разрыв линии NAMUR SENS_1
  DEVID_PULSE_1,     // превышение частоты счета импульсов SENS_1
  DEVID_PULSE_2,     // превышение частоты счета импульсов SENS_1
  DEVID_PULSE_3,     // превышение частоты счета импульсов SENS_1
  DEVID_PULSE_4,     // превышение частоты счета импульсов SENS_1
  DEVID_LOG_SIZE,   // Размер журнала
  DEVID_NUM
} eDevId;

// Структура записи FLASH
typedef struct __packed __aligned(4){
  uint32_t utime;           // Unix-time
  uint32_t data[4];         // Данные датчиков ISENS_1-ISENS_4
  union {
    eDevId   devid;
    uint32_t u32devid;
  };
  uint32_t dummy[2];           // Расширяем до 32 байт
} sLogRec;

extern sLogRec logRdBuf[SMALLEST_BUFFER_SIZE];

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

void flashHwTest( void );

void logInit( void );
void logEnable( void );

uint8_t logger( uint32_t utime, eDevId devid, uint32_t data[], uint8_t size );

#endif /* LOGGER_H_ */
