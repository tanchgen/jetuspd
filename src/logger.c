/*
 * logger.c
 *
 *  Created on: 13 апр. 2021 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#include "main.h"
#include "stm32l1xx_ll_spi.h"
#include "gpio_arch.h"
#include "isens.h"
#include "buffer.log.h"
#include "logger.h"
#include "flash.h"
#include "mqtt.h"

typedef struct {
  logBuf_t logBufHandle;
//  uint32_t mcuStartCount;
//  uint32_t errCount;
//  FlagStatus catErr;
} sFlashHeader;


uint32_t mcuStartCount;     // Общий счетчик запусков MCU
uint32_t errCount;     // Общий счетчик ошибок

// --------------- БУФЕРЫ ПЕРЕДАЧИ --------------------------------
// БУФЕР очереди на запись в FLASH
sLogRec logWrBuf[SMALLEST_BUFFER_SIZE];
logBuf_t logWrBuffer;

// БУФЕР считанных записей ISENS из FLASH
sLogRec logRdSensBuf[8];
logBuf_t logRdSensBuffer;

// БУФЕР считанных записей ISENS из FLASH
sLogRec logRdEvntBuf[8];
logBuf_t logRdEvntBuffer;

logBuf_t stmEeBuffer;

// ----------------------- Чтения логов ------------------------
//static enum eLogQuery{
//  LOG_READ_NULL,
//  LOG_READ_QUERY,
//  LOG_READ_ACCEPT,
//  LOG_READ_END
//} logQuery = LOG_READ_NULL;                // Запрос логов

uint16_t logRdBufFill = 0;                  // Уровень заполненности буфера чтения логов

// Таймер чтения логов для отправки на сервер
//struct timer_list logReadTimer;
// Таймер записи состояния системы для записи в Лог
struct timer_list logWriteTimer;


// ----------------------- Private function ---------------------------
void stmEeInit( void );
HAL_StatusTypeDef   stmEeRead( uint32_t addr, uint32_t * data, uint32_t datalen);

//uint16_t stmEeBuf_Read( logBuf_t* Buffer, sLogRec * pkt, uint16_t count);
//uint16_t stmEeBuf_Write( logBuf_t* Buffer, sLogRec * pkt, uint16_t count);
//HAL_StatusTypeDef  stmEeBufSave( logBuf_t* Buffer );

//---------------------------------------------------------------------

/**
  * @brief Hardware test MRAM
  * @param None
  * @retval None
  */
void loggerHwTest( void ) {
//  GPIO_InitTypeDef GPIO_InitStruct;

//  logInit();
//  logEnable();

//  // Особая конфигурация вывода NSS - без подтяжки
//  GPIO_InitStruct.GPIO_Pin = eeprom.eeSpi.gpioPinMiso;
//  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//
//  GPIO_Init( eeprom.eeSpi.gpioMiso, &GPIO_InitStruct);

  flashHwTest();

  return;
}


// Обработка таймаута ожидания команды чтения логов
void logReadTout( uintptr_t arg ){
  (void)arg;

}


// Обработка таймаута записи состояния системы
void logWriteTout( uintptr_t arg ){
  (void)arg;

//  if( (SIM800.mqttServer.mqttconn == 0) && iSens[ISENS_1].isensFlag ){
//    //TODO: Сделать для всех логируемых устройств
//    logger( getRtcTime(), DEVID_ISENS_1, iSens[ISENS_1].isensCount );
//    iSens[ISENS_1].isensFlag = RESET;
//  }

  timerMod( &logWriteTimer, LOG_WRITE_TOUT * 1000 );
}


uint8_t stmEeBuf_Init(logBuf_t* Buffer, sLogRec * startAddr, uint16_t Size ) {
  /* Set buffer values to all zeros */
  memset(Buffer, 0, sizeof(logBuf_t));

  /* Set default values */
  Buffer->Size = Size;
  Buffer->Buffer = startAddr;

  Buffer->In = Buffer->Out = Buffer->Buffer;
  /* We are initialized */
  Buffer->Flags |= logBuf_INITIALIZED;

  /* Initialized OK */
  return 0;
}


/**
  * @brief  Сохранение в буфере записи для Архива.
  *
  * @param[in]  utime - RTC-время
  * @param[in]  devid - Идентификатор архивируемого "Устройства"/параметра
  * @param[in]  data  - Массив данных ( макс. 4 4-хбитных слова )
  * @param[in]  size  - кол-во слова
  *
  * @retval кол-во сделанных записей (1 или 0)
  */
uint8_t logger( uint32_t utime, eDevId devid, uint32_t data[], uint8_t size ){
  sLogRec logrec;

  logrec.utime = utime;            // Время RTC
  logrec.devid = devid;        // Идентификатор логируемого устройства
  for( uint8_t i = 0; i < size; i++ ){
    assert_param( data != NULL );
    logrec.data[i] = data[i];          // Значение логируемого параметра
  }
  return logBuf_Write( &logWrBuffer, &logrec, 1 );
}


// --------------------------------------------------------


void logClock( void ){
//  uint16_t num;
//  sLogRec logrec;
//
  // Проверка на наличие записей из Архива
  if( logRdBufFill ){
    // Есть хаписи для публикации из Архива
    SIM800.mqttClient.pubFlags.archPub = SET;
  }
  else {
    // Есть хаписи для публикации из Архива
    SIM800.mqttClient.pubFlags.archPub = RESET;
  }

  return;
}


/**
  * @brief  Разрешение работы интерфейса Logger.
  *
  * @param[in]  none
  *
  * @retval none
  */
void logEnable( void ){
  logBuf_t logBufHandle;

  NVIC_ClearPendingIRQ( SPI1_IRQn );

  NVIC_EnableIRQ( SPI1_IRQn );
  // ---------------- Читаем заголовок FLASH SENS -----------------------
  stmEeRead( SENS_HEADER_ADDR, (uint32_t*)&logBufHandle, sizeof(logBufHandle) );
  if( ((uint32_t)(logBufHandle.Buffer) != SENS_LOG_START_ADDR)
      || ((uint32_t)(logBufHandle.Size) != SENS_LOG_SIZE) ){
    // Первая инициализация LOG_FLASH
    flashBuf_Init( &flashSensBuffer, (sLogRec *)SENS_LOG_START_ADDR, SENS_LOG_SIZE );
    flashBufSave( &flashSensBuffer );
  }
  else {
    // Заголовок уже записан в FLASH
    flashSensBuffer = logBufHandle;
  }
  // ---------------- Читаем заголовок FLASH EVNT -----------------------
  stmEeRead( EVNT_HEADER_ADDR, (uint32_t*)&logBufHandle, sizeof(logBufHandle) );
  if( ((uint32_t)(logBufHandle.Buffer) != EVNT_LOG_START_ADDR)
      || ((uint32_t)(logBufHandle.Size) != EVNT_LOG_SIZE) ){
    // Первая инициализация LOG_FLASH
    flashBuf_Init( &flashEvntBuffer, (sLogRec *)EVNT_LOG_START_ADDR, EVNT_LOG_SIZE );
    flashBufSave( &flashEvntBuffer );
  }
  else {
    // Заголовок уже записан в FLASH
    flashEvntBuffer = logBufHandle;
  }

  timerMod( &logWriteTimer, LOG_WRITE_TOUT * 1000 );
}


void logInit( void ){
  logBuf_Init( &logWrBuffer, logWrBuf, ARRAY_SIZE(logWrBuf) );
  logBuf_Init( &logRdSensBuffer, logRdSensBuf, ARRAY_SIZE(logRdSensBuf) );
  logBuf_Init( &logRdEvntBuffer, logRdEvntBuf, ARRAY_SIZE(logRdEvntBuf) );
  stmEeInit();
  flashInit();
//  timerSetup( &logReadTimer, logReadTout, (uintptr_t)NULL );
  timerSetup( &logWriteTimer, logWriteTout, (uintptr_t)NULL );
}
