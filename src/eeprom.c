/*
 * eeprom.c
 *
 *  Created on: 19 сент. 2021 г.
 *      Author: jet
 */
#include "main.h"
//#include "gpio_arch.h"
#include "buffer.log.h"
#include "eeprom.h"

//uint32_t EEPROMPageSize;

HAL_StatusTypeDef   stmEeRead( uint32_t addr, uint32_t * data, uint32_t datalen);

/******************************* SPI Routines**********************************/

///**
//  * @brief SPI Read 4 bytes from device
//  * @retval Read data
//*/
//static uint32_t SPIx_Read(void)
//{
//  HAL_StatusTypeDef status = HAL_OK;
//  uint32_t          readvalue = 0;
//  uint32_t          writevalue = 0xFFFFFFFF;
//
//  status = HAL_SPI_TransmitReceive(&heval_Spi, (uint8_t*) &writevalue, (uint8_t*) &readvalue, 1, SpixTimeout);
//
//  /* Check the communication status */
//  if(status != HAL_OK)
//  {
//    /* Execute user timeout callback */
//    SPIx_Error();
//  }
//
//  return readvalue;
//}
//
///**
//  * @brief SPI Write a byte to device
//  * @param Value: value to be written
//  * @retval None
//  */
//static void SPIx_Write(uint8_t Value)
//{
//  HAL_StatusTypeDef status = HAL_OK;
//
//  status = HAL_SPI_Transmit(&heval_Spi, (uint8_t*) &Value, 1, SpixTimeout);
//
//  /* Check the communication status */
//  if(status != HAL_OK)
//  {
//    /* Execute user timeout callback */
//    SPIx_Error();
//  }
//}
//
///**
//  * @brief SPI error treatment function
//  * @retval None
//  */
//void SPIx_Error (void) {
//  /* De-initialize the SPI communication BUS */
//  gpioPinReset( &gpioPinFlashOn );
//  mDelay(100);
//
//  mramInit();
//}

/*******************************************************************************/


//
///**
//  * @brief  Writes buffer of data to the EEPROM device selected.
//  * @param  pBuffer : pointer to the buffer  containing the data to be written
//  *         to the EEPROM.
//  * @param  WriteAddr : EEPROM's internal address to write to.
//  * @param  NumByteToWrite : number of bytes to write to the EEPROM.
//  * @retval EEPROM_OK (0) if operation is correctly performed, else return value
//  *         different from EEPROM_OK (0) or the timeout user callback.
//  */
//uint32_t BSP_EEPROM_WriteBuffer(uint8_t* pBuffer, uint16_t WriteAddr, uint32_t NumByteToWrite)
//{
//  uint16_t numofpage = 0, numofsingle = 0, count = 0;
//  uint16_t addr = 0;
//  uint32_t dataindex = 0;
//  uint32_t status = EEPROM_OK;
//
//  addr = WriteAddr % EEPROMPageSize;
//  count = EEPROMPageSize - addr;
//  numofpage =  NumByteToWrite / EEPROMPageSize;
//  numofsingle = NumByteToWrite % EEPROMPageSize;
//
//  if(EEPROM_SelectedDevice->WritePage == 0)
//  {
//    return EEPROM_FAIL;
//  }
//
//  /*!< If WriteAddr is EEPROM_PAGESIZE aligned  */
//  if(addr == 0)
//  {
//    /*!< If NumByteToWrite < EEPROM_PAGESIZE */
//    if(numofpage == 0)
//    {
//      /* Store the number of data to be written */
//      dataindex = numofsingle;
//      /* Start writing data */
//      status = EEPROM_SelectedDevice->WritePage(pBuffer, WriteAddr, (uint32_t*)(&dataindex));
//      if (status != EEPROM_OK)
//      {
//        return status;
//      }
//    }
//    /*!< If NumByteToWrite > EEPROM_PAGESIZE */
//    else
//    {
//      while(numofpage--)
//      {
//        /* Store the number of data to be written */
//        dataindex = EEPROMPageSize;
//        status = EEPROM_SelectedDevice->WritePage(pBuffer, WriteAddr, (uint32_t*)(&dataindex));
//        if (status != EEPROM_OK)
//        {
//          return status;
//        }
//
//        WriteAddr +=  EEPROMPageSize;
//        pBuffer += EEPROMPageSize;
//      }
//
//      if(numofsingle!=0)
//      {
//        /* Store the number of data to be written */
//        dataindex = numofsingle;
//        status = EEPROM_SelectedDevice->WritePage(pBuffer, WriteAddr, (uint32_t*)(&dataindex));
//        if (status != EEPROM_OK)
//        {
//          return status;
//        }
//          }
//        }
//    }
//  /*!< If WriteAddr is not EEPROM_PAGESIZE aligned  */
//  else
//  {
//    /*!< If NumByteToWrite < EEPROM_PAGESIZE */
//    if(numofpage== 0)
//    {
//      /*!< If the number of data to be written is more than the remaining space
//      in the current page: */
//      if (NumByteToWrite > count)
//      {
//        /* Store the number of data to be written */
//        dataindex = count;
//        /*!< Write the data contained in same page */
//        status = EEPROM_SelectedDevice->WritePage(pBuffer, WriteAddr, (uint32_t*)(&dataindex));
//        if (status != EEPROM_OK)
//        {
//          return status;
//        }
//
//        /* Store the number of data to be written */
//        dataindex = (NumByteToWrite - count);
//        /*!< Write the remaining data in the following page */
//        status = EEPROM_SelectedDevice->WritePage((uint8_t*)(pBuffer + count), (WriteAddr + count), (uint32_t*)(&dataindex));
//        if (status != EEPROM_OK)
//        {
//          return status;
//        }
//      }
//      else
//      {
//        /* Store the number of data to be written */
//        dataindex = numofsingle;
//        status = EEPROM_SelectedDevice->WritePage(pBuffer, WriteAddr, (uint32_t*)(&dataindex));
//        if (status != EEPROM_OK)
//        {
//          return status;
//        }
//        }
//      }
//    /*!< If NumByteToWrite > EEPROM_PAGESIZE */
//    else
//    {
//      NumByteToWrite -= count;
//      numofpage =  NumByteToWrite / EEPROMPageSize;
//      numofsingle = NumByteToWrite % EEPROMPageSize;
//
//      if(count != 0)
//      {
//        /* Store the number of data to be written */
//        dataindex = count;
//        status = EEPROM_SelectedDevice->WritePage(pBuffer, WriteAddr, (uint32_t*)(&dataindex));
//        if (status != EEPROM_OK)
//        {
//          return status;
//        }
//        WriteAddr += count;
//        pBuffer += count;
//      }
//
//      while(numofpage--)
//      {
//        /* Store the number of data to be written */
//        dataindex = EEPROMPageSize;
//        status = EEPROM_SelectedDevice->WritePage(pBuffer, WriteAddr, (uint32_t*)(&dataindex));
//        if (status != EEPROM_OK)
//        {
//          return status;
//        }
//        WriteAddr +=  EEPROMPageSize;
//        pBuffer += EEPROMPageSize;
//      }
//      if(numofsingle != 0)
//      {
//        /* Store the number of data to be written */
//        dataindex = numofsingle;
//        status = EEPROM_SelectedDevice->WritePage(pBuffer, WriteAddr, (uint32_t*)(&dataindex));
//        if (status != EEPROM_OK)
//        {
//          return status;
//        }
//      }
//    }
//  }
//
//  /* If all operations OK, return EEPROM_OK (0) */
//  return EEPROM_OK;
//}


HAL_StatusTypeDef  stmEeBufSave( logBuf_t* Buffer ){
  // Сохраняем весь заголовок в EEPROM
  return stmEeWrite( 0, (uint32_t*)Buffer, sizeof(logBuf_t) );
}


uint16_t stmEeBuf_Read( logBuf_t* Buffer, sLogRec * pkt, uint16_t count) {
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
    while (count) {
      uint16_t wrcount;

      // TODO: непосредственная запись в FLASH
      // Проверяем ,сколько можно читать за раз (не выходя за границы Буфера и FLASH)
      if( (Buffer->Out + count) <= (Buffer->Buffer + Buffer->Size) ) {      // Проверяем границу FLASH
        wrcount = count;
        count = 0;
      }
      else {
        wrcount = count - ((Buffer->Buffer + Buffer->Size) - Buffer->Out);
        count -= wrcount;
      }
      stmEeRead( (uint32_t)Buffer->Out, (uint32_t*)pkt, wrcount * sizeof( sLogRec ) );
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

    stmEeBufSave( Buffer );
  }

  /* Return number of elements stored in memory */
  return i;
}


uint16_t stmEeBuf_Write( logBuf_t* Buffer, sLogRec * pkt, uint16_t count) {
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
    wrFlag = SET;
  }

  /* Go through all elements */
  while (count) {
    uint16_t wrcount;
    // TODO: непосредственная запись в FLASH
    if( (Buffer->In + count) <= (Buffer->Buffer + Buffer->Size) ){      // Проверяем границу FLASH
      wrcount = count;
      count = 0;
    }
    else {
      wrcount = count - ((Buffer->Buffer + Buffer->Size) - Buffer->In);
      count -= wrcount;
    }
    stmEeWrite( (uint32_t)Buffer->In, (uint32_t *)pkt, wrcount * sizeof( sLogRec ) );
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
    stmEeBufSave( Buffer );
  }

  /* Return number of elements written */
  return i;
}



/*===============================================================================
 *                    ##### DATA EEPROM Programming functions #####
 *===============================================================================*/

/**
 * @brief  Unlocks the data memory and FLASH_PECR register access.
 * @retval HAL_StatusTypeDef HAL Status
 */
void stmEeUnlock(void) {
 if((FLASH->PECR & FLASH_PECR_PELOCK) != RESET) {
   /* Unlocking the Data memory and FLASH_PECR register access*/
   FLASH->PEKEYR = FLASH_PEKEY1;
   FLASH->PEKEYR = FLASH_PEKEY2;
 }
 return;
}

/**
 * @brief  Locks the Data memory and FLASH_PECR register access.
 * @retval HAL_StatusTypeDef HAL Status
 */
void stmEeLock(void) {
 /* Set the PELOCK Bit to lock the data memory and FLASH_PECR register access */
 FLASH->PECR |= FLASH_PECR_PELOCK;

 return;
}


HAL_StatusTypeDef   stmEeRead( uint32_t addr, uint32_t * data, uint32_t datalen) {
 HAL_StatusTypeDef status = HAL_ERROR;
 uint32_t addrEnd;

 addr += FLASH_EEPROM_BASE;
 addrEnd = addr + datalen;

 assert_param( addrEnd <= FLASH_EEPROM_END );

 /* Wait for last operation to be completed */
 if( FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE) != HAL_OK){
   return HAL_ERROR;
 }

 for( ;addr < addrEnd; addr += 4 ) {
   *data++ = *(__IO uint32_t *)addr;
 }

 return status;
}

/**
 * @brief  Program word at a specified address
 * @note   To correctly run this function, the @ref HAL_FLASHEx_DATAEEPROM_Unlock() function
 *         must be called before.
 *         Call the @ref HAL_FLASHEx_DATAEEPROM_Unlock() to he data EEPROM access
 *         and Flash program erase control register access(recommended to protect
 *         the DATA_EEPROM against possible unwanted operation).
 * @note   The function @ref HAL_FLASHEx_DATAEEPROM_EnableFixedTimeProgram() can be called before
 *         this function to configure the Fixed Time Programming.
 * @param  TypeProgram  Indicate the way to program at a specified address.
 *         This parameter can be a value of @ref FLASHEx_Type_Program_Data
 * @param  Address  specifie the address to be programmed.
 * @param  Data     specifie the data to be programmed
 *
 * @retval HAL_StatusTypeDef HAL Status
 */

HAL_StatusTypeDef   stmEeWrite( uint32_t addr, uint32_t * data, uint32_t datalen) {
 HAL_StatusTypeDef status = HAL_ERROR;
 uint32_t addrEnd;

 addr += FLASH_EEPROM_BASE;
 addrEnd = addr + datalen;

 assert_param( addrEnd <= FLASH_EEPROM_END );

 while( addr < addrEnd) {
   /* Wait for last operation to be completed */
   status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);

   if(status == HAL_OK) {
     /* Clean the error context */
     *(__IO uint32_t *)addr = *data++;
     addr += 4;
   }
   else {
     break;
   }
 }

 return status;
}


void stmEeInit( void ){
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  stmEeUnlock();
}
