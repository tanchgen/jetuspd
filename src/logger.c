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
//#include "eeprom.h"
#include "logger.h"


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

// БУФЕР считанных записей из FLASH
sLogRec logRdBuf[SMALLEST_BUFFER_SIZE];

//logBuf_t logRdBuffer;

logBuf_t stmEeBuffer;

sFlashDev logDev;

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

uint16_t stmEeBuf_Read( logBuf_t* Buffer, sLogRec * pkt, uint16_t count);
uint16_t stmEeBuf_Write( logBuf_t* Buffer, sLogRec * pkt, uint16_t count);
HAL_StatusTypeDef  stmEeBufSave( logBuf_t* Buffer );

//---------------------------------------------------------------------

// Обработка таймаута ожидания команды чтения логов
void logReadTout( uintptr_t arg ){
  (void)arg;

}


// Обработка таймаута записи состояния системы
void logWriteTout( uintptr_t arg ){
  (void)arg;

  if( iSens[ISENS_1].isensFlag ){
    //TODO: Сделать для всех логиркемых устройств
    logger( getRtcTime(), DEVID_ISENS_1, iSens[ISENS_1].isensCount );
  }
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
  * @brief Hardware test STM_EEPROM (ring buffer)
  * @param None
  * @retval None
  */
int stmEeHwTest( void ) {
  int rc = 0;

  for( uint8_t c = 0; c < 20; c++ ){
    for( uint8_t i = 0; i < SMALLEST_BUFFER_SIZE; i++ ){
      sLogRec logrec;
      logrec.utime = 0x11111111;
      logrec.devid = DEVID_VBAT;
      logrec.data = i;
      stmEeBuf_Write( &stmEeBuffer, &logrec, 1);
    }

    for( uint8_t i = 0; i < SMALLEST_BUFFER_SIZE; i++ ){
      sLogRec logrec;

      stmEeBuf_Read( &stmEeBuffer, &logrec, 1);

      if( (logrec.utime != 0x11111111)
          || (logrec.devid != DEVID_VBAT)
          || (logrec.data != i) )
      {
        trace_puts( "LOG ERROR" );
        rc = -1;
      }
    }
  }

  trace_puts( "LOG OK" );

  return rc;
}



/**
  * @brief  Логирование сообщения о проблемме.
  *
  * @param[in]  none
  *
  * @retval none
  */
uint8_t logger( uint32_t utime, eDevId devid, uint32_t data ){
  sLogRec logrec;

  logrec.utime = utime;            // Время RTC
  logrec.devid = devid;        // Идентификатор логируемого устройства
  logrec.data = data;          // Значение логируемого параметра

  return logBuf_WriteMsg( &logWrBuffer, &logrec );
}



// --------------------------------------------------------


// Обработчик запроса Системного Лога
void logQueryProcess( void ){

  if( logRdBufFill == 0 ){
    // Буфер чтения пуст - можно читать следующие записи
    logRdBufFill = logBuf_GetFull( &stmEeBuffer );
    logRdBufFill = min( ARRAY_SIZE(logRdBuf), logRdBufFill );
    stmEeBuf_Read( &stmEeBuffer, logRdBuf, logRdBufFill );
  }

}


void logClock( void ){
  uint16_t num;
  sLogRec logrec;

  // Проверка на наличие записей для логирования
  if( (num = logBuf_GetFull( &logWrBuffer )) == 0 ){
    // Буфер пуст
    return;
  }

  num = min( num, FLASH_PAGE_SIZE );

  if( logBuf_Read( &logWrBuffer, &logrec, num ) != num ){
    // Ошибка чтения из Буфера
    trace_puts("LogBuffer ERROR: reading\n") ;
    return;
  }

  if( stmEeBuf_Write( &stmEeBuffer, &logrec, num ) != num ){
    // Ошибка записи в FLASH
    trace_puts("FLASH write failure!\n") ;
  }

  // Обработка запроса логов
  logQueryProcess();
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

  // ---------------- Читаем заголовок FLASH -----------------------
  stmEeRead( 0, (uint32_t*)&logBufHandle, sizeof(logBufHandle) );
  if( (uint32_t)(logBufHandle.Buffer) != EE_LOG_START_ADDR ){
    // Первая инициализация LOG_FLASH
    stmEeBuf_Init( &stmEeBuffer, (sLogRec *)EE_LOG_START_ADDR, EE_LOG_SIZE );
    mcuStartCount = 1;
    stmEeBufSave( &stmEeBuffer );
  }
  else {
    // Заголовок уже записан в FLASH
    stmEeBuffer = logBufHandle;
  }

  timerMod( &logWriteTimer, LOG_WRITE_TOUT * 1000 );
}


void logInit( void ){
  logBuf_Init( &logWrBuffer, logWrBuf, ARRAY_SIZE(logWrBuf) );
  stmEeInit();
//  timerSetup( &logReadTimer, logReadTout, (uintptr_t)NULL );
  timerSetup( &logWriteTimer, logWriteTout, (uintptr_t)NULL );
}
