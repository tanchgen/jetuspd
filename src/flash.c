/*
 * flash.c
 *
 *  Created on: 23 сент. 2021 г.
 *      Author: jet
 */


#include "stm32l1xx.h"

#include "main.h"
#include "gpio_arch.h"
#include "buffer.log.h"
#include "logger.h"
#include "times.h"
#include "uspd.h"
#include "flash.h"


uint32_t wrAddr;
uint32_t wrData;

extern logBuf_t logRdSensBuffer;
extern logBuf_t logRdEvntBuffer;
extern logBuf_t logWrBuffer;
extern uint16_t logRdBufFill;

static uint32_t tmpTick;

sFlashDev flashDev = {
  .state = FLASH_READY,
  .flashEmpty = SET,
  .writeCount = 0
};

/** @defgroup STM32L152D_EVAL_EEPROM_Private_Variables Private Variables
  * @{
  */
__IO uint16_t  EEPROMAddress = 0;
__IO uint16_t  EEPROMPageSize = 0;
__IO uint16_t  EEPROMDataRead = 0;
__IO uint8_t   EEPROMDataWrite = 0;

logBuf_t flashSensBuffer;
logBuf_t flashEvntBuffer;

uint8_t __aligned(4) flashRxXfer[256+4];
uint8_t __aligned(4) flashTxXfer[256+4];

struct timer_list flashOpTimer;

// ================ Прототипы функций ====================================
HAL_StatusTypeDef   stmEeWrite( uint32_t addr, uint32_t * data, uint32_t datalen);

void spiIrqHandler( sSpiHandle * hspi );
// =======================================================================

void flashHwTest( void ){
  sSpiHandle *hspi = &flashDev.flashSpi;
  SPI_TypeDef *SPIx;

  gpioPinResetNow( &gpioPinFlashOn );

  mDelay(2);
  SPIx = hspi->spi;
  // --------- Тестирование MRAM -------------------
  while( (SPIx->SR & SPI_SR_BSY) != RESET)
  {}

  flashTxXfer[0] = FLASH_CMD_RDID;    // Send "Read from Memory" instruction
  flashTxXfer[1] = 0;
  spiXfer( hspi, 4, flashTxXfer, flashRxXfer);
  while( (SPIx->SR & SPI_SR_BSY) != RESET)
  {}

  if( *((uint32_t*)flashRxXfer) != 0x1628C200 ){
    goto errExit;
  }

  flashTxXfer[0] = FLASH_CMD_WREN;    // Send "Read from Memory" instruction
  spiXfer( hspi, 1, flashTxXfer, NULL);
  while( (SPIx->SR & SPI_SR_BSY) != RESET)
  {}

  flashTxXfer[0] = FLASH_CMD_RDSR;    // Send "Read from Memory" instruction
  flashTxXfer[1] = 0;
  spiXfer( hspi, 2, flashTxXfer, flashRxXfer);
  while( (SPIx->SR & SPI_SR_BSY) != RESET)
  {}

  // Стираем сектор
  flashTxXfer[0] = FLASH_CMD_SE;    // Send "Read from Memory" instruction
  flashTxXfer[1] = 0x00;
  flashTxXfer[2] = 0x01;
  flashTxXfer[3] = 0x00;
  spiXfer( hspi, 4, flashTxXfer, flashRxXfer);
  while( (SPIx->SR & SPI_SR_BSY) != RESET)
  {}

  tmpTick = mTick;
  flashTxXfer[0] = FLASH_CMD_RDSR;    // Send "Read from Memory" instruction
  flashTxXfer[1] = 0;
  do{
    spiXfer( hspi, 2, flashTxXfer, flashRxXfer);
    while( (SPIx->SR & SPI_SR_BSY) != RESET)
    {}
  } while ( flashRxXfer[1] & FLASH_WIP_FLAG );

  tmpTick = mTick - tmpTick;

  flashTxXfer[0] = FLASH_CMD_READ;    // Send "Read from Memory" instruction
  flashTxXfer[1] = 0x00;
  flashTxXfer[2] = 0x01;
  flashTxXfer[3] = 0x00;
  spiXfer( hspi, 4 + sizeof( sLogRec ) * 8, flashTxXfer, flashRxXfer);
  while( (SPIx->SR & SPI_SR_BSY) != RESET)
  {}

  if ( flashRxXfer[3 + sizeof( sLogRec ) * 8] != 0xFF ){
    goto errExit;
  }
  flashTxXfer[0] = FLASH_CMD_WREN;    // Send "Read from Memory" instruction
  spiXfer( hspi, 1, flashTxXfer, NULL);
  while( (SPIx->SR & SPI_SR_BSY) != RESET)
  {}

// Запись
  flashTxXfer[4] = 0x11;
  memset( &flashTxXfer[5], 0xAA, 254 );
  flashTxXfer[259] = 0x22;

  flashTxXfer[0] = FLASH_CMD_WRITE;    // Send "Read from Memory" instruction
  flashTxXfer[1] = 0x00;
  flashTxXfer[2] = 0x01;
  flashTxXfer[3] = 0x00;
  spiXfer( hspi, 4 + 256, flashTxXfer, NULL);
  while( (SPIx->SR & SPI_SR_BSY) != RESET)
  {}

  tmpTick = mTick;
  flashTxXfer[0] = FLASH_CMD_RDSR;    // Send "Read from Memory" instruction
  flashTxXfer[1] = 0;
  do{
    spiXfer( hspi, 2, flashTxXfer, flashRxXfer);
    while( (SPIx->SR & SPI_SR_BSY) != RESET)
    {}
  } while ( flashRxXfer[1] & FLASH_WIP_FLAG );

  tmpTick = mTick - tmpTick;

  flashTxXfer[0] = FLASH_CMD_READ;    // Send "Read from Memory" instruction
  flashTxXfer[1] = 0x00;
  flashTxXfer[2] = 0x01;
  flashTxXfer[3] = 0x00;
  spiXfer( hspi, 4 + 256, flashTxXfer, flashRxXfer);
  while( (SPIx->SR & SPI_SR_BSY) != RESET)
  {}

  if( memcmp( &flashTxXfer[4], &(flashRxXfer[4]), sizeof( sLogRec )* 8 ) == 0 ){
    trace_puts( "MRAM OK" );
    return;
  }

errExit:
  trace_puts( "MRAM ERR" );
  return;
  // -------------------------------------------------
}


void flashOpTout( uintptr_t arg ){
  flashDev.state = (eFlashState)arg;
}

void SPI1_IRQHandler( void ) {
  spiIrqHandler( &flashDev.flashSpi );
};

// =================== Работа с FLASH =============================================

void flashReadStart( sFlashDev * flash, uint32_t addr, uint32_t size ){
  // Пишем данные во флеш
  flashTxXfer[0] = FLASH_CMD_READ;
  flashTxXfer[1] = (addr >> 16) & 0xFF ;
  flashTxXfer[2] = (addr >> 8) & 0xFF ;
  flashTxXfer[3] = addr & 0xFF ;

  /*!< Send "Write Enable" instruction */
  flashDev.state = FLASH_BUSY;
  spiXfer( &(flash->flashSpi), size + 4, flashTxXfer, flashRxXfer );
}

// Коллбек по окончании разрешения записи во ФЛЕШ
FlagStatus flashWrEnProbe( sSpiHandle * spi ){
  spi->rxCallback = NULL;
  flashTxXfer[0] = FLASH_CMD_RDSR;
  flashTxXfer[1] = 0;
  spiXfer( spi, 2, flashTxXfer, flashRxXfer);
  // Ждем окончания приема
  while( spi->spi->SR & SPI_SR_BSY )
  {}
  // Проверяем флаг разрешения приема
  return (flashRxXfer[1] & FLASH_WEL_FLAG );
}


// Коллбек по окончании разрешения записи во ФЛЕШ
void flashWrEnCb( void ){
  flashDev.state = FLASH_WRITE_EN;
}


// Коллбек по передаче команды Стирания Сектора
void flashSECb( void ){
  flashDev.sectorClear = SET;
  flashDev.state = FLASH_SECTOR_CLEAR;
}


// Коллбек по окончании Стирания Сектора
void flashSEEndCb( void ){
  if( flashRxXfer[1] & FLASH_WIP_FLAG ){
    flashDev.state = FLASH_SECTOR_CLEAR;
  }
  else {
    flashDev.state = FLASH_WRITE_START;
  }
}


// Коллбек по окончании записи во ФЛЕШ
void flashWrCb( void ){
  flashDev.state = FLASH_WRITE_OK;
}


/**
  * @brief  Write WRITE_ENABLE cmd to FLASH
  * @param  None
  * @retval HAL_StatusTypeDef HAL Status
  */
void flashWrEn( sSpiHandle * spi ){
  /*!< Enable the write access to the FLASH */

  spi->rxCallback = flashWrEnCb;
  flashTxXfer[0] = FLASH_CMD_WREN;
  /*!< Send "Write Enable" instruction */
  flashDev.state = FLASH_BUSY;
  spiXfer( spi, 1, flashTxXfer, NULL );
}


void flashSectorErase( sSpiHandle * spi, uint32_t addr ){
  spi->rxCallback = flashSECb;
  flashTxXfer[0] = FLASH_CMD_SE;
  flashTxXfer[1] = (addr >> 16) & 0xFF ;
  flashTxXfer[2] = (addr >> 8) & 0xFF ;
  flashTxXfer[3] = addr & 0xFF ;
  /*!< Send "Write Enable" instruction */
  flashDev.state = FLASH_BUSY;
  spiXfer( spi, 4, flashTxXfer, NULL );
}

void flashSeEndTest( sSpiHandle * spi ){
  spi->rxCallback = flashSEEndCb;
  flashTxXfer[0] = FLASH_CMD_RDSR;    // Send "Read from Memory" instruction
  flashTxXfer[1] = 0;
  flashDev.state = FLASH_BUSY;
  spiXfer( spi, 2, flashTxXfer, flashRxXfer);
}

FlagStatus flashWriteStart( sFlashDev * flash, uint32_t addr, uint32_t * data, uint32_t size ){
  // Стереть страницу, если надо
  if( ((addr & 0xFFF) == 0) && (flash->sectorClear == RESET) ){
    // Требуется стереть страницу
    flashSectorErase( &(flash->flashSpi), addr );
    return RESET;
  }
  // Сбрасываем флаг чистого сектора
  flash->sectorClear = RESET;
  // Пишем данные во флеш
//  flashTxXfer[0] = FLASH_CMD_WREN;    // Send "Read from Memory" instruction
//  spiXfer( &(flash->flashSpi), 1, flashTxXfer, NULL);
//  while( (flash->flashSpi.spi->SR & SPI_SR_BSY) != RESET)
//  {}

  flash->flashSpi.rxCallback = flashWrCb;
  flashTxXfer[0] = FLASH_CMD_WRITE;
  flashTxXfer[1] = (addr >> 16) & 0xFF ;
  flashTxXfer[2] = (addr >> 8) & 0xFF ;
  flashTxXfer[3] = addr & 0xFF ;

  memcpy( &(flashTxXfer[4]), data, size );

  wrAddr = addr;
  wrData = *data;

  /*!< Send "Write Enable" instruction */
  flashDev.state = FLASH_BUSY;
  spiXfer( &(flash->flashSpi), size + 4, flashTxXfer, NULL );

  return SET;
}

// ==================== Работа с буферами =========================================
HAL_StatusTypeDef  flashBufSave( logBuf_t* Buffer ){
  uint32_t addr;

  // Сохраняем весь заголовок в EEPROM
  if( Buffer == &flashSensBuffer ){
    addr = SENS_HEADER_ADDR;
  }
  else {
    assert_param( Buffer == &flashEvntBuffer );
    addr = EVNT_HEADER_ADDR;
  }

  assert_param( (Buffer->Size == SENS_LOG_SIZE)
                || (Buffer->Size == EVNT_LOG_SIZE) );
  return stmEeWrite( addr, (uint32_t*)Buffer, sizeof(logBuf_t) );
}


// Коллбек по окончании счивания паетов из Буфера Журнала Сенсоров
void flashSensRxCb( void ){
  // Сохранить новые данные о буфере
  flashBufSave( &flashSensBuffer );
  logRdBufFill += logBuf_Write( &logRdSensBuffer, (sLogRec*)(flashRxXfer+4), flashDev.quant );
  // Освобождаем Флеш
  flashDev.state = FLASH_READY;
}

// Коллбек по окончании счивания паетов из Буфера Журнала Событий
void flashEvntRxCb( void ){
  // Сохранить новые данные о буфере
  flashBufSave( &flashEvntBuffer );
  logRdBufFill += logBuf_Write( &logRdEvntBuffer, (sLogRec*)(flashRxXfer+4), flashDev.quant );
  // Освобождаем Флеш
  flashDev.state = FLASH_READY;
}


/**
  * @brief  Старт чтения из буфера флешки.
  *
  * @note
  *
  * @param  Buffer Указатель на хидер буфера
  * @param  Указатель на место сохранения считанных пакетов
  * @param  Кол-во считываемых пакетов
  * @param  Указатель на коллбек после окончания считывания
  *
  * @retval EEPROM_OK (0) if operation is correctly performed, else return value
  *         different from EEPROM_OK (0) or the timeout user callback.
  */
uint16_t flashBuf_Read( sFlashDev * flash, logBuf_t* Buffer, sLogRec * pkt, uint16_t count, void (*rxCb)(void) ) {
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
  if( count ){
    /* Go through all elements */
    flash->flashSpi.rxCallback = rxCb;
    while (count) {
      uint16_t wrcount;
      uint32_t out;

      // Проверяем ,сколько можно читать за раз (не выходя за границы Буфера и FLASH)
      if( (Buffer->Out + count) <= (Buffer->Buffer + Buffer->Size) ) {      // Проверяем границу FLASH
        wrcount = count;
        count = 0;
      }
      else {
        wrcount = count - ((Buffer->Buffer + Buffer->Size) - Buffer->Out);
        count -= wrcount;
      }
      out = (uint32_t)Buffer->Out;
      // Если читается ОЧЕНЬ быстро - сохранится уже обновленное значение.
      Buffer->Out += wrcount;
      flashReadStart( flash, out, wrcount * sizeof( sLogRec ) );
      pkt += wrcount;

      /* Increase pointers */
      i += wrcount;

      /* Check FLASH overflow */
      if (Buffer->Out >= (Buffer->Buffer + Buffer->Size) ) {
        Buffer->Out = Buffer->Buffer;
      }
    }

    Buffer->Flags &= ~logBuf_OVER;

//    stmEeBufSave( Buffer );
  }

  /* Return number of elements stored in memory */
  return i;
}


uint16_t flashBuf_Write( sFlashDev * flash, logBuf_t* Buffer, sLogRec * pkt, uint16_t count ) {
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


  /* Go through all elements */
  while (count) {
    uint16_t wrcount;
    if( (Buffer->In + count) <= (Buffer->Buffer + Buffer->Size) ){      // Проверяем границу FLASH
      wrcount = count;
      count = 0;
    }
    else {
      wrcount = count - ((Buffer->Buffer + Buffer->Size) - Buffer->In);
      count -= wrcount;
    }
    if( flashWriteStart( flash, (uint32_t)Buffer->In, (uint32_t *)pkt, wrcount * sizeof( sLogRec ) ) ){
      wrFlag = SET;

      Buffer->In += wrcount;
      pkt += wrcount;

      /* Increase pointers */
      i += wrcount;

      /* Check FLASH overflow */
      if (Buffer->In >= (Buffer->Buffer + Buffer->Size) ) {
        Buffer->In = Buffer->Buffer;
      }
    }
  }

  if( wrFlag ){
    flashBufSave( Buffer );
  }

  /* Return number of elements written */
  return i;
}


void flashWriteOkProbe( sSpiHandle * spi ){
  // Проверяем окончание записи
  spi->rxCallback = NULL;
  flashTxXfer[0] = FLASH_CMD_RDSR;

  spiXfer( spi, 2, flashTxXfer, flashRxXfer );
  // Ждем окончания приема
  while( spi->spi->SR & SPI_SR_BSY )
  {}
  // Проверяем флаг разрешения приема
  if( (flashRxXfer[1] & FLASH_WIP_FLAG) != RESET ){
    return;
  }

  if( (flashRxXfer[1] & FLASH_WEL_FLAG) != RESET ){
    ErrHandler( NON_STOP );
  }
  flashTxXfer[0] = FLASH_CMD_RDSCUR;
  spiXfer( spi, 2, flashTxXfer, flashRxXfer );
  // Ждем окончания приема
  while( spi->spi->SR & SPI_SR_BSY )
  {}
  // Проверяем флаги ошибки
  if( (flashRxXfer[1] & (FLASH_EFAIL_FLAG | FLASH_PFAIL_FLAG)) == RESET ){
    // Освобождаем Флеш
    flashDev.state = FLASH_READY;
    flashDev.quant = 0;
  }
  else {
    ErrHandler( NON_STOP );
  }
}



// =================================================================================

// Обработка запросов
eLogBufType flashReadProbe( void ){
//  sLogRec * rec;
  uint16_t quant;

// ----------- Журнал Датчиков -------------------------
  if( uspd.readArchSensQuery ){
    quant = logBuf_GetFull( &flashSensBuffer );
    if(quant == 0){
      uspd.readArchSensQuery = RESET;
    }
    else {
      // Чтобы поместилось в буфер чтения SPI и буфер logRdBuffer
      quant = min( quant, (ARRAY_SIZE(flashRxXfer) - 4) / sizeof(sLogRec) );
      quant = min( quant, logBuf_GetFree( &logRdSensBuffer) );
      if( quant ){
        flashDev.quant = quant;
        return LOG_BUF_SENS;
      }
    }
  }

// ----------- Журнал Событий -------------------------
  if( uspd.readArchEvntQuery ){
    quant = logBuf_GetFull( &flashEvntBuffer );
    if(quant == 0){
      uspd.readArchEvntQuery = RESET;
    }
    else {
      // Чтобы поместилось в буфер чтения SPI и буфер logRdBuffer
      quant = min( quant, (ARRAY_SIZE(flashRxXfer) - 4) / sizeof(sLogRec) );
      quant = min( quant, logBuf_GetFull( &logRdEvntBuffer) );
      if( quant ){
        flashDev.quant = quant;
        return LOG_BUF_EVNT;
      }
    }
  }

  return LOG_BUF_NULL;
}


// Обработка записи сохранений во Флеш
eLogBufType  flashWriteProbe( void ){
  eLogBufType rc = LOG_BUF_NULL;

  if( logBuf_GetFull( &logWrBuffer) == 0 ){
    return rc;
  }
  if( (flashDev.rec = malloc(sizeof(sLogRec))) == NULL ){
    ErrHandler( NON_STOP );
    return rc;
  }

//  trace_printf( "a_flash_%x\n", flashDev.rec );

  logBuf_Read( &logWrBuffer, flashDev.rec, 1 );
  flashDev.quant = 1;

  if( flashDev.rec->devid <= DEVID_ISENS_4 ){
    rc = LOG_BUF_SENS;
  }
  else {
    assert_param( (flashDev.rec->devid > DEVID_ISENS_4) && (flashDev.rec->devid < DEVID_NUM) );
    rc = LOG_BUF_EVNT;
  }

  return rc;
}


void flashProcess( void ){

  switch( flashDev.state ){
    case FLASH_READY:{
      eFlashState fs;

      // Читаем, если надо
      if( (flashDev.lbType = flashReadProbe()) != LOG_BUF_NULL ){
        fs = FLASH_READ_START;
      }
      else if( (flashDev.lbType = flashWriteProbe()) != LOG_BUF_NULL ){
        // Есть что писать - включаем Vdd FLASH
        fs = FLASH_WRITE_START;
      }
      else {
        // Делать нечего - выключаем Vdd FLASH
        gpioPinSet( &gpioPinFlashOn );
        break;
      }
      // Есть что читать - включаем Vdd FLASH
      gpioPinReset( &gpioPinFlashOn );
      flashDev.state = FLASH_BUSY;
      // После включения - запуск чтения
      timerModArg( &flashOpTimer, 1, fs);
      break;
    }
    case FLASH_READ_START:{
      logBuf_t * buf;
      void (*rxCb)(void);

      if( flashDev.lbType == LOG_BUF_SENS ){
        buf = &flashSensBuffer;
        rxCb = flashSensRxCb;
      }
      else {
        buf = &flashEvntBuffer;
        rxCb = flashEvntRxCb;
      }
      if( flashBuf_Read( &flashDev, buf, flashDev.rec, flashDev.quant, rxCb ) == 0 ){
        flashDev.state = FLASH_READY;
      }
      break;
    }
    case FLASH_WRITE_START:
      // Разрешена запись
      flashWrEn( &flashDev.flashSpi );
      break;
    case FLASH_SECTOR_CLEAR:
      // Разрешена запись
      flashSeEndTest( &flashDev.flashSpi );
      break;
    case FLASH_WRITE_EN:{
      logBuf_t * buf;

      if( flashDev.lbType == LOG_BUF_SENS ){
        buf = &flashSensBuffer;
      }
      else {
        buf = &flashEvntBuffer;
      }

      if( flashBuf_Write( &flashDev, buf, flashDev.rec, flashDev.quant ) == flashDev.quant ){
//        trace_printf( "f_flash_%x\n", flashDev.rec );
        my_free( flashDev.rec );
      }
      break;
    }
    case FLASH_WRITE_OK:
      flashWriteOkProbe( &(flashDev.flashSpi) );
      break;
    default:
      break;
  }


}


uint8_t flashBuf_Init(logBuf_t* Buffer, sLogRec * startAddr, uint16_t Size ) {
  /* Set buffer values to all zeros */
  memset(Buffer, 0, sizeof(logBuf_t));

  /* Set default values */
  Buffer->Size = Size;
  Buffer->Buffer = startAddr;

  Buffer->In = Buffer->Out = Buffer->Buffer;
  /* We are initialized */
  Buffer->Flags |= logBuf_INITIALIZED;

  Buffer->UserParameters = &flashDev.flashSpi;
  /* Initialized OK */
  return 0;
}


void flashInit( void ){

  sSpiHandle *hspi = &(flashDev.flashSpi);

  /* Выводы MRAM_SPI
    * SPI MOSI - PA7
    * SPI MISO - PA6
    * SPI SCK  - PA5
    * SPI NSS  - PA4
    */
  /* Инициализируем дескриптор SPI MRAM. */
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  hspi->spi = SPI1;
  hspi->gpioMosi = GPIOA;
  hspi->gpioPinMosi = GPIO_PIN_7;
  hspi->gpioMiso = GPIOA;
  hspi->gpioPinMiso = GPIO_PIN_6;
  hspi->gpioSck = GPIOA;
  hspi->gpioPinSck = GPIO_PIN_5;
  hspi->gpioNss = GPIOA;
  hspi->gpioPinNss = GPIO_PIN_4;

  hspi->SPI_FirstBit = SPI_FIRSTBIT_MSB;
  hspi->SPI_DataSize = SPI_DATASIZE_8BIT;
#if SPI_DMA_ENABLE
  hspi->dmaRxStream = MRAM_DMA_RX_Stream;
  hspi->dmaRxTcif = MRAM_DMA_RX_TCIF;
  hspi->dmaRxIRQn = MRAM_DMA_RX_IRQn;
  hspi->dmaTxStream = MRAM_DMA_TX_Stream;
  hspi->dmaTxTcif = MRAM_DMA_TX_TCIF;
  hspi->dmaTxIRQn = MRAM_DMA_TX_IRQn;
#endif // SPI_DMA_ENABLE


  hspi->rxBuf = (uint8_t *)&flashRxXfer;
  hspi->txBuf = (uint8_t *)&flashTxXfer;

  // Инициализация MRAM_SPI
  spiGpioInit( hspi );

  spiInit( hspi );
#if  SPI_DMA_ENABLE
   spiDmaInit( hspi );
#endif  // MRAM_SPI_DMA_ENABLE

   timerSetup( &flashOpTimer, flashOpTout, (uintptr_t)NULL );
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



