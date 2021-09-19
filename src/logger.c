/*
 * logger.c
 *
 *  Created on: 13 апр. 2021 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#include "main.h"
#include "stm32l1xx_ll_spi.h"
#include "gpio_arch.h"
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

logBuf_t flashBuffer;

uFlashXferBuffer flashTxXfer;
uFlashXferBuffer flashRxXfer;

sFlashDev logDev;

// ----------------------- Чтения логов ------------------------
static enum eLogQuery{
  LOG_READ_NULL,
  LOG_READ_QUERY,
  LOG_READ_ACCEPT,
  LOG_READ_END
} logQuery = LOG_READ_NULL;                // Запрос логов

uint16_t logRdBufFill = 0;                  // Уровень заполненности буфера чтения логов

// Таймер таймаута ожидания команды чтения логов
struct timer_list logReadTimer;

// ----------------------- Private function ---------------------------
void logInit( void );
void logEnable( void );
void logDisable( void );
//---------------------------------------------------------------------

// Обработка таймаута ожидания команды чтения логов
void logReadTout( uintptr_t arg ){
  (void)arg;

}

/**
  * @brief  Reads a block of data from the FLASH.
  *
  * @param  pBuffer: pointer to the buffer that receives the data read from the FLASH.
  * @param  ReadAddr: FLASH's internal address to read from.
  * @param  NumByteToRead: number of bytes to read from the FLASH.
  * @retval None
  */
eFlashOperations flashRead( sSpiHandle * hspi, uint32_t ReadAddr, uint8_t * buf, uint16_t size ) {
  while( (hspi->spi->SR & SPI_SR_BSY) != RESET )
  {}

  /*
      We gonna send all commands in one packet of 3 bytes
   */

  flashTxXfer.header[0] = FLASH_READ;    // Send "Read from Memory" instruction
  flashTxXfer.header[1] = ReadAddr >> 16;  // Send 24-bit address
  flashTxXfer.header[2] = ReadAddr >> 8;
  flashTxXfer.header[3] = ReadAddr;

//  /* Send WriteAddr address byte to read from */
//  FLASH_SPI_SendInstruction( hspi, header, 4);

  spiXfer( hspi, size + 4, flashTxXfer.header, buf);

  return FLASH_STATUS_COMPLETE;
}


/**
  * @brief  Writes more than one byte to the FLASH
  *
  * @note   The number of byte can't exceed the FLASH page size.
  * @param  bufr: pointer to the buffer  containing the data to be written
  *         to the FLASH.
  * @param  WriteAddr: FLASH's internal address to write to.
  * @param  size: number of bytes to write to the FLASH.
  * @retval EepromOperations value: FLASH_STATUS_COMPLETE or FLASH_STATUS_ERROR
  */
eFlashOperations flashWrite(sSpiHandle * hspi, uint32_t WriteAddr, uint8_t * buf, uint16_t size) {
  uFlashXferBuffer txXfer;

  assert_param( size <= sizeof(sLogRec) * BIG_BUFFER_SIZE );
  assert_param( (WriteAddr + size) <= FLASH_SIZE );
  /*
   *  We gonna send commands in one packet of 3 bytes
   */

    txXfer.header[0] = FLASH_WRITE;   // Send "Write to Memory" instruction
    txXfer.header[1] = WriteAddr >> 16; // Send 24-bit address
    txXfer.header[2] = WriteAddr >> 8; // Send 16-bit address
    txXfer.header[3] = WriteAddr;

    memcpy( txXfer.data, buf, size );

    while( (hspi->spi->SR & SPI_SR_BSY) != RESET )
    {}
    spiXfer( hspi, size + 4, (uint8_t *)&txXfer, NULL );

    return FLASH_STATUS_COMPLETE;
}


uint8_t flashBuf_Init(logBuf_t* Buffer, sLogRec * flashStartAddr, uint16_t Size ) {
  /* Set buffer values to all zeros */
  memset(Buffer, 0, sizeof(logBuf_t));

  /* Set default values */
  Buffer->Size = Size;
  Buffer->Buffer = flashStartAddr;

  Buffer->In = Buffer->Out = Buffer->Buffer;
  /* We are initialized */
  Buffer->Flags |= logBuf_INITIALIZED;

  /* Initialized OK */
  return 0;
}


eFlashOperations  flashBufSave( sSpiHandle * hspi, logBuf_t* Buffer ){
  sFlashHeader header;

  header.logBufHandle = *Buffer;
  // Сохраняем весь заголовок в FLASH
  return flashWrite( hspi, 0, (uint8_t*)&header, sizeof(header) );
}

uint16_t flashBuf_Read( sSpiHandle * hspi, logBuf_t* Buffer, sLogRec * pkt, uint16_t count) {
  uint32_t i = 0, full;

  /* Check buffer structure */
  if (Buffer == NULL || count == 0) {
    return 0;
  }

//  /* Check output pointer */
//  if (Buffer->Out >= (Buffer->Buffer + Buffer->Size) ) {
//    Buffer->Out = Buffer->Buffer;
//  }

  /* Get free memory */
  full = logBuf_GetFull(Buffer);

  /* Check available memory */
  if (full < count) {
    /* If no memory, stop execution */
    if (full == 0) {
      return 0;
    }

    /* Set values for write */
    count = full;
  }

  /* We have calculated memory for write */

  /* Go through all elements */
  while (count--) {
    uint16_t wrcount;

    // TODO: непосредственная запись в FLASH
    // Проверяем ,сколько можно читать за раз (не выходя за границы Буфера и FLASH)
    if( (Buffer->Out + count) < (Buffer->Buffer + Buffer->Size) ) {      // Проверяем границу FLASH
      wrcount = count;
      count = 0;
    }
    else {
      wrcount = count - ((Buffer->Buffer + Buffer->Size) - Buffer->Out);
      count -= wrcount;
    }
    flashRead( hspi, (uint32_t)Buffer->Out, (uint8_t*)pkt, wrcount * sizeof( sLogRec ) );
    Buffer->Out += wrcount;
    pkt += wrcount;

    /* Increase pointers */
    i += wrcount;

    /* Check FLASH overflow */
    if (Buffer->Out >= (Buffer->Buffer + Buffer->Size) ) {
      Buffer->Out = Buffer->Buffer;
    }
  }

  Buffer->Flags &= ~logBuf_OVER;

  flashBufSave( hspi, Buffer );

  /* Return number of elements stored in memory */
  return i;
}


uint16_t flashBuf_Write( sSpiHandle * hspi, logBuf_t* Buffer, sLogRec * pkt, uint16_t count) {
  uint32_t i = 0;
  uint32_t free;
  FlagStatus wrFlag = RESET;

  /* Check buffer structure */
  if (Buffer == NULL || count == 0) {
    return 0;
  }

//  /* Check input pointer */
//  if (Buffer->In >= (Buffer->Buffer + Buffer->Size) ){
//    Buffer->In = Buffer->Buffer;
//  }

  /* Get free memory */
  free = logBuf_GetFree(Buffer);

  /* Check available memory */
  if (free < count) {
    /* If no memory, stop execution */
    if (free == 0) {
      return 0;
    }

    /* Set values for write */
    count = free;
    Buffer->Flags |= logBuf_OVER;
  }
  else if (free == count){
    Buffer->Flags |= logBuf_OVER;
  }

  /* We have calculated memory for write */
  if(count){
    flashWriteEnable( hspi );
    wrFlag = SET;
  }

  /* Go through all elements */
  while (count) {
    uint16_t wrcount;
    // TODO: непосредственная запись в FLASH
    if( (Buffer->In + count) < (Buffer->Buffer + Buffer->Size) ){      // Проверяем границу FLASH
      wrcount = count;
      count = 0;
    }
    else {
      wrcount = count - ((Buffer->Buffer + Buffer->Size) - Buffer->In);
      count -= wrcount;
    }
    flashWrite( hspi, (uint32_t)Buffer->In, (uint8_t *)pkt, wrcount * sizeof( sLogRec ) );
    Buffer->In += wrcount;
    pkt += wrcount;

    /* Increase pointers */
    i += wrcount;

    /* Check FLASH overflow */
    if (Buffer->In >= (Buffer->Buffer + Buffer->Size) ) {
      Buffer->In = Buffer->Buffer;
    }
  }

  if( wrFlag ){
    flashWriteDisable( hspi );
    flashBufSave( hspi, Buffer );
  }

  /* Return number of elements written */
  return i;
}



/**
  * @brief Hardware test FLASH
  * @param None
  * @retval None
  */
int flashHwTest( void ) {
  sSpiHandle *hspi = &logDev.flashSpi;
  SPI_TypeDef *SPIx;
  int rc = 0;
//  GPIO_InitTypeDef GPIO_InitStruct;

  logInit();
  logEnable();

//  // Особая конфигурация вывода NSS - без подтяжки
//  GPIO_InitStruct.GPIO_Pin = eeprom.eeSpi.gpioPinMiso;
//  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//
//  GPIO_Init( eeprom.eeSpi.gpioMiso, &GPIO_InitStruct);


  SPIx = hspi->spi;
  // --------- Тестирование FLASH -------------------
  while( (SPIx->SR & SPI_SR_BSY) != RESET)
  {}

  flashTxXfer.header[0] = FLASH_RDID;    // Send "Read from Memory" instruction
  flashTxXfer.header[1] = 0;
  spiXfer( hspi, 10, flashTxXfer.header, flashTxXfer.data);
  while( (SPIx->SR & SPI_SR_BSY) != RESET)
  {}

  memset( flashTxXfer.data, 0xAA, FLASH_PAGE_SIZE );
  flashWrite( hspi, 0x100, (uint8_t *)&flashTxXfer, FLASH_PAGE_SIZE);
  while( (SPIx->SR & SPI_SR_BSY) != RESET)
  {}
//  mDelay(20);
  flashRead( hspi, 0x100, (uint8_t *)&flashRxXfer, FLASH_PAGE_SIZE );
  while( (SPIx->SR & SPI_SR_BSY) != RESET)
  {}

  if( memcmp( flashTxXfer.data, flashRxXfer.data, FLASH_PAGE_SIZE ) != 0 ){
    logDev.statusIfaceErr = SET;
    trace_puts( "FLASH ERR" );
  }
  else {
    trace_puts( "FLASH OK" );
  }
//  logDisable();
  // -------------------------------------------------

  return rc;
}



/**
  * @brief  Логирование сообщения о проблемме.
  *
  * @param[in]  none
  *
  * @retval none
  */
uint8_t logger( eDevId devid, uint8_t idx, eLogErrId errId, eParamId prm, uint16_t data ){
  uint16_t prmAddr;
  sLogRec logrec;

  prmAddr = paramAddr( devid, idx, prm);

  logrec.startCount = mcuStartCount;      // Порядковый номер запуска MCU
  logrec.utime = 0;                       // Время RTC - зарезервированно
  logrec.mcuTick = mTick;                 // Счетчик системных тиков
  logrec.errCount = ++errCount;           // Счетчик ошибок (общий)
  logrec.errId = errId;           // Идентификатор ошибки
  logrec.tmAddr = prmAddr;          // Адрес параметра в пакете телеметрии (см. Протокол обмена)
  logrec.tmData = data;          // Значение параметра (см. Протокол обмена)

  return logBuf_WriteMsg( &logWrBuffer, &logrec );
}



// --------------------------------------------------------


// Обработчик запроса Системного Лога
void logQueryProcess( enum eLogQuery query ){

  if( query == LOG_READ_QUERY) {
    // Есть запрос
    if( dbgHnd.txh->protoNum != PROTO_LOG ){
      // Надо менять протокол
      if( xferProto[dbgHnd.txh->protoNum].frTxctx == NULL){
        // DBG_UART свободен - меняем протокол на PROTO_LOG
        uartDisable( dbgHnd.rxh, dbgHnd.txh );
        dbgHnd.txh->protoNum = PROTO_LOG;
        uartEnable( dbgHnd.rxh, dbgHnd.txh );
      }
      else {
        // UART Занят - выходим
        return;
      }
    }

    if( logRdBufFill == 0 ){
      // Буфер чтения пуст - можно читать следующие записи
      logRdBufFill = logBuf_GetFull( &flashBuffer );
      logRdBufFill = min( ARRAY_SIZE(logRdBuf), logRdBufFill );
      flashBuf_Read( &(logDev.flashSpi), &flashBuffer, logRdBuf, logRdBufFill );
    }
    // TODO: ПРОДОЖИТЬ
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

  if( logBuf_Read( &logWrBuffer, logrec, num ) != num ){
    // Ошибка чтения из Буфера
    trace_puts("LogBuffer ERROR: reading\n") ;
    return;
  }

  if( flashBuf_Write( &(logDev.flashSpi), &flashBuffer, logrec, num ) != num ){
    // Ошибка записи в FLASH
    trace_puts("FLASH write failure!\n") ;
  }

  // Обработка запроса логов
  logQueryProcess( logQuery );


}


/**
  * @brief  Разрешение работы интерфейса Logger.
  *
  * @param[in]  none
  *
  * @retval none
  */
void logEnable( void ){
  sFlashHeader flashHeader;

#if DEBUG_TRACE
  trace_puts("Function: Enable EEPROM");
#endif

#if SPI_NSS_HARD
  /* Enable NSS output for master mode */
  SPI_SSOutputCmd( FLASH_SPI, ENABLE);
#endif

#if FLASH_SPI_DMA_ENABLE
  enable_nvic_irq( logDev.flashSpi.dmaRxIRQn, SPI_IRQ_PRIORITY);
  enable_nvic_irq( logDev.flashSpi.dmaTxIRQn, SPI_IRQ_PRIORITY);
#else
  enable_nvic_irq( FLASH_SPI_IRQn, SPI_IRQ_PRIORITY);
#endif  // FLASH_SPI_DMA_ENABLE

  /* Configure SPI1 transfer interrupts */
  /* Enable RXNE  Interrupt             */
  logDev.flashSpi.spi->CR2 |= SPI_CR2_RXNEIE | SPI_CR2_TXEIE;
  /* Enable Error Interrupt             */
  logDev.flashSpi.spi->CR2 |= SPI_CR2_ERRIE;

  // ---------------- Читаем заголовок FLASH -----------------------
  flashRead( &(logDev.flashSpi), 0, (uint8_t*)&flashHeader, sizeof(flashHeader) );
  if( (uint32_t)(flashHeader.logBufHandle.Buffer) != FLASH_LOG_START_ADDR ){
    // Первая инициализация LOG_FLASH
    logBuf_Init( &flashBuffer, (sLogRec *)FLASH_LOG_START_ADDR, FLASH_LOG_SIZE );
    mcuStartCount = 1;
    flashBufSave( &logDev.flashSpi, &flashBuffer );
    // Сохраняем весь заголовок в FLASH
    flashWrite( &(logDev.flashSpi), 0, (uint8_t*)&flashHeader, sizeof(flashHeader) );
  }
  else {
    // Заголовок уже записан в FLASH
    flashBuffer = flashHeader.logBufHandle;
  }

}

/**
  * @brief  Разрешение работы интерфейса Logger.
  *
  * @param[in]  none
  *
  * @retval none
  */
void logDisable( void ){

#if DEBUG_TRACE
  trace_puts("Function: Enable EEPROM");
#endif

  NVIC_DisableIRQ( FLASH_SPI_IRQn );
  hspiNow = NULL;
}



void flashInit(){
  sGpioPin pin = {GPIOA, GPIO_PIN_7, GPIO_MODE_AF_PP, GPIO_PULLDOWN, GPIO_SPEED_FREQ_HIGH, AF5, Bit_RESET, Bit_RESET, RESET };
  sSpiHandle *hspi = &(logDev.flashSpi);

#if DEBUG_TRACE
  trace_puts("Function: Init LOGGER");
#endif
  /* Выводы FLASH_SPI
    * SPI MOSI - PA7
    * SPI MISO - PA6
    * SPI SCK  - PA5
    * SPI NSS  - PA4
    */
  /* Инициализируем дескриптор SPI FLASH. */

  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  /* (1) Enables GPIO clock and configures the SPI1 pins ********************/
  /* Enable the peripheral clock of GPIOB */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

  // Setup MOSI
  gpioPinSetup( &pin );
  // Setup MISO
  pin.pin = GPIO_PIN_6;
  gpioPinSetup( &pin );
  // Setup SCK
  pin.pin = GPIO_PIN_5;
  gpioPinSetup( &pin );
  // Setup NSS
  pin.pin = GPIO_PIN_4;
  pin.mode = GPIO_MODE_OUTPUT_PP;
  pin.af = AF0;
  gpioPinSetup( &pin );



  /* (2) Configure NVIC for SPI1 transfer complete/error interrupts **********/
  /* Set priority for SPI1_IRQn */
  NVIC_SetPriority(SPI1_IRQn, 0);
  /* Enable SPI1_IRQn           */
  NVIC_EnableIRQ(SPI1_IRQn);

  /* (3) Configure SPI1 functional parameters ********************************/

  /* Configure SPI1 communication */
  LL_SPI_SetBaudRatePrescaler(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV16);
  LL_SPI_SetTransferDirection(SPI1,LL_SPI_FULL_DUPLEX);
  LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_2EDGE);
  LL_SPI_SetClockPolarity(SPI1, LL_SPI_POLARITY_HIGH);
  /* Reset value is LL_SPI_MSB_FIRST */
  //LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);
  LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);
  LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_SOFT);
  LL_SPI_SetMode(SPI1, LL_SPI_MODE_MASTER);

  hspi->rxBuf = (uint8_t *)&flashRxXfer;
  hspi->txBuf = (uint8_t *)&flashTxXfer;

  logDev.dataNok = SET;
  logDev.statusIfaceErr = RESET;

  // Инициализация FLASH_SPI
  spiGpioInit( hspi );

  spiInit( hspi );
#if  FLASH_SPI_DMA_ENABLE
   spiDmaInit( hspi );
#endif  // FLASH_SPI_DMA_ENABLE

   MAYBE_BUILD_BUG_ON( FLASH_LOG_START_ADDR < sizeof(sFlashHeader) );


}

void logInit( void ){


  logBuf_Init( &logWrBuffer, logWrBuf, ARRAY_SIZE(logWrBuf) );
  flashInit();
  timerSetup( &logReadTimer, logReadTout, (uintptr_t)NULL );
}
