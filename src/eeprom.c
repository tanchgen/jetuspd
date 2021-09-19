/*
 * eeprom.c
 *
 *  Created on: 19 сент. 2021 г.
 *      Author: jet
 */
#include "main.h"
#include "gpio_arch.h"
#include "eeprom.h"

uint32_t EEPROMPageSize;

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

/**
  * @brief SPI error treatment function
  * @retval None
  */
void SPIx_Error (void) {
  /* De-initialize the SPI communication BUS */
  gpioPinReset( &gpioPinEeOn );
  mDelay(100);

  mramInit();
}

/*******************************************************************************


/**
  * @brief  Initializes peripherals used by the SPI EEPROM driver.
  * @retval EEPROM_OK (0) if operation is correctly performed, else return value
  *         different from EEPROM_OK (0)
  */
static uint32_t EEPROM_SPI_Init(void)
{
  EEPROM_SPI_IO_Init();

  /* Check if EEPROM SPI is ready to use */
  EEPROMPageSize = EEPROM_PAGESIZE_M95040;
  if(EEPROM_SPI_WaitEepromStandbyState() != HAL_OK)
  {
    return EEPROM_FAIL;
  }
  return EEPROM_OK;
}

/**
  * @brief  Reads a block of data from the SPI EEPROM.
  * @param  pBuffer : pointer to the buffer that receives the data read from
  *         the EEPROM.
  * @param  ReadAddr : EEPROM's internal address to start reading from.
  * @param  NumByteToRead : pointer to the variable holding number of bytes to
  *         be read from the EEPROM.
  *
  *        @note The variable pointed by NumByteToRead is reset to 0 when all the
  *              data are read from the EEPROM. Application should monitor this
  *              variable in order know when the transfer is complete.
  *
  * @retval EEPROM_OK (0) if operation is correctly performed, else return value
  *         different from EEPROM_OK (0) or the timeout user callback.
  */
static uint32_t EEPROM_SPI_ReadBuffer(uint8_t* pBuffer, uint16_t ReadAddr, uint32_t* NumByteToRead)
{
  uint32_t buffersize = *NumByteToRead;
  EEPROM_SPI_IO_ReadData(ReadAddr, pBuffer, buffersize);

    /* Wait for EEPROM Standby state */
  if (EEPROM_SPI_WaitEepromStandbyState() != EEPROM_OK)
  {
    return EEPROM_FAIL;
  }

  return EEPROM_OK;
}

/**
  * @brief  Writes more than one byte to the EEPROM with a single WRITE cycle.
  *
  * @note   The number of bytes (combined to write start address) must not
  *         cross the EEPROM page boundary. This function can only write into
  *         the boundaries of an EEPROM page.
  *         This function doesn't check on boundaries condition (in this driver
  *         the function BSP_EEPROM_WriteBuffer() which calls EEPROM_WritePage() is
  *         responsible of checking on Page boundaries).
  *
  * @param  pBuffer : pointer to the buffer containing the data to be written to
  *         the EEPROM.
  * @param  WriteAddr : EEPROM's internal address to write to.
  * @param  NumByteToWrite : pointer to the variable holding number of bytes to
  *         be written into the EEPROM.
  *
  *        @note The variable pointed by NumByteToWrite is reset to 0 when all the
  *              data are written to the EEPROM. Application should monitor this
  *              variable in order know when the transfer is complete.
  *
  *
  * @retval EEPROM_OK (0) if operation is correctly performed, else return value
  *         different from EEPROM_OK (0) or the timeout user callback.
  */
static uint32_t EEPROM_SPI_WritePage(uint8_t* pBuffer, uint16_t WriteAddr, uint32_t* NumByteToWrite)
{
  uint32_t buffersize = *NumByteToWrite;

  if (EEPROM_SPI_IO_WriteData(WriteAddr, pBuffer, buffersize) !=  HAL_OK)
  {
    return EEPROM_FAIL;
  }

  /* Wait for EEPROM Standby state */
  if (EEPROM_SPI_WaitEepromStandbyState() != EEPROM_OK)
  {
    return EEPROM_FAIL;
  }

  return EEPROM_OK;
}

/**
  * @brief  Wait for EEPROM SPI Standby state.
  *
  * @note  This function allows to wait and check that EEPROM has finished the
  *        last operation. It is mostly used after Write operation.
  *
  * @retval EEPROM_OK (0) if operation is correctly performed, else return value
  *         different from EEPROM_OK (0) or the timeout user callback.
  */
static uint32_t EEPROM_SPI_WaitEepromStandbyState(void)
{
  if(EEPROM_SPI_IO_WaitEepromStandbyState() != HAL_OK)
  {
    BSP_EEPROM_TIMEOUT_UserCallback();
    return EEPROM_TIMEOUT;
  }
  return EEPROM_OK;
}

/**
  * @brief  Initializes peripherals used by the EEPROM device selected.
  * @retval EEPROM_OK (0) if operation is correctly performed, else return value
  *         different from EEPROM_OK (0)
  */
uint32_t BSP_EEPROM_Init(void)
{
  if(EEPROM_SelectedDevice->Init != 0)
  {
    return (EEPROM_SelectedDevice->Init());
  }
  else
  {
    return EEPROM_FAIL;
  }
}

/**
  * @brief  Select the EEPROM device to communicate.
  * @param  DeviceID: Specifies the EEPROM device to be selected.
  *   This parameter can be one of following parameters:
  *     @arg BSP_EEPROM_M24LR64
  *     @arg BSP_EEPROM_M24M01
  *     @arg BSP_EEPROM_M95M01
  *
  * @retval EEPROM_OK (0) if operation is correctly performed, else return value
  *         different from EEPROM_OK (0)
  */
void BSP_EEPROM_SelectDevice(uint8_t DeviceID)
{
  switch(DeviceID)
  {
  case BSP_EEPROM_M24LR64 :
    EEPROM_SelectedDevice = &EEPROM_I2C_Drv;
    break;

  case BSP_EEPROM_M95040 :
    EEPROM_SelectedDevice = &EEPROM_SPI_Drv;
    break;

  default:
    break;
  }
}

/**
  * @brief  Reads a block of data from the EEPROM device selected.
  * @param  pBuffer : pointer to the buffer that receives the data read from
  *         the EEPROM.
  * @param  ReadAddr : EEPROM's internal address to start reading from.
  * @param  NumByteToRead : pointer to the variable holding number of bytes to
  *         be read from the EEPROM.
  *
  *        @note The variable pointed by NumByteToRead is reset to 0 when all the
  *              data are read from the EEPROM. Application should monitor this
  *              variable in order know when the transfer is complete.
  *
  * @retval EEPROM_OK (0) if operation is correctly performed, else return value
  *         different from EEPROM_OK (0) or the timeout user callback.
  */
uint32_t BSP_EEPROM_ReadBuffer(uint8_t* pBuffer, uint16_t ReadAddr, uint32_t* NumByteToRead)
{
  if(EEPROM_SelectedDevice->ReadBuffer != 0)
  {
    return (EEPROM_SelectedDevice->ReadBuffer(pBuffer, ReadAddr, NumByteToRead));
  }
  else
  {
    return EEPROM_FAIL;
  }
}

/**
  * @brief  Writes buffer of data to the EEPROM device selected.
  * @param  pBuffer : pointer to the buffer  containing the data to be written
  *         to the EEPROM.
  * @param  WriteAddr : EEPROM's internal address to write to.
  * @param  NumByteToWrite : number of bytes to write to the EEPROM.
  * @retval EEPROM_OK (0) if operation is correctly performed, else return value
  *         different from EEPROM_OK (0) or the timeout user callback.
  */
uint32_t BSP_EEPROM_WriteBuffer(uint8_t* pBuffer, uint16_t WriteAddr, uint32_t NumByteToWrite)
{
  uint16_t numofpage = 0, numofsingle = 0, count = 0;
  uint16_t addr = 0;
  uint32_t dataindex = 0;
  uint32_t status = EEPROM_OK;

  addr = WriteAddr % EEPROMPageSize;
  count = EEPROMPageSize - addr;
  numofpage =  NumByteToWrite / EEPROMPageSize;
  numofsingle = NumByteToWrite % EEPROMPageSize;

  if(EEPROM_SelectedDevice->WritePage == 0)
  {
    return EEPROM_FAIL;
  }

  /*!< If WriteAddr is EEPROM_PAGESIZE aligned  */
  if(addr == 0)
  {
    /*!< If NumByteToWrite < EEPROM_PAGESIZE */
    if(numofpage == 0)
    {
      /* Store the number of data to be written */
      dataindex = numofsingle;
      /* Start writing data */
      status = EEPROM_SelectedDevice->WritePage(pBuffer, WriteAddr, (uint32_t*)(&dataindex));
      if (status != EEPROM_OK)
      {
        return status;
      }
    }
    /*!< If NumByteToWrite > EEPROM_PAGESIZE */
    else
    {
      while(numofpage--)
      {
        /* Store the number of data to be written */
        dataindex = EEPROMPageSize;
        status = EEPROM_SelectedDevice->WritePage(pBuffer, WriteAddr, (uint32_t*)(&dataindex));
        if (status != EEPROM_OK)
        {
          return status;
        }

        WriteAddr +=  EEPROMPageSize;
        pBuffer += EEPROMPageSize;
      }

      if(numofsingle!=0)
      {
        /* Store the number of data to be written */
        dataindex = numofsingle;
        status = EEPROM_SelectedDevice->WritePage(pBuffer, WriteAddr, (uint32_t*)(&dataindex));
        if (status != EEPROM_OK)
        {
          return status;
        }
          }
        }
    }
  /*!< If WriteAddr is not EEPROM_PAGESIZE aligned  */
  else
  {
    /*!< If NumByteToWrite < EEPROM_PAGESIZE */
    if(numofpage== 0)
    {
      /*!< If the number of data to be written is more than the remaining space
      in the current page: */
      if (NumByteToWrite > count)
      {
        /* Store the number of data to be written */
        dataindex = count;
        /*!< Write the data contained in same page */
        status = EEPROM_SelectedDevice->WritePage(pBuffer, WriteAddr, (uint32_t*)(&dataindex));
        if (status != EEPROM_OK)
        {
          return status;
        }

        /* Store the number of data to be written */
        dataindex = (NumByteToWrite - count);
        /*!< Write the remaining data in the following page */
        status = EEPROM_SelectedDevice->WritePage((uint8_t*)(pBuffer + count), (WriteAddr + count), (uint32_t*)(&dataindex));
        if (status != EEPROM_OK)
        {
          return status;
        }
      }
      else
      {
        /* Store the number of data to be written */
        dataindex = numofsingle;
        status = EEPROM_SelectedDevice->WritePage(pBuffer, WriteAddr, (uint32_t*)(&dataindex));
        if (status != EEPROM_OK)
        {
          return status;
        }
        }
      }
    /*!< If NumByteToWrite > EEPROM_PAGESIZE */
    else
    {
      NumByteToWrite -= count;
      numofpage =  NumByteToWrite / EEPROMPageSize;
      numofsingle = NumByteToWrite % EEPROMPageSize;

      if(count != 0)
      {
        /* Store the number of data to be written */
        dataindex = count;
        status = EEPROM_SelectedDevice->WritePage(pBuffer, WriteAddr, (uint32_t*)(&dataindex));
        if (status != EEPROM_OK)
        {
          return status;
        }
        WriteAddr += count;
        pBuffer += count;
      }

      while(numofpage--)
      {
        /* Store the number of data to be written */
        dataindex = EEPROMPageSize;
        status = EEPROM_SelectedDevice->WritePage(pBuffer, WriteAddr, (uint32_t*)(&dataindex));
        if (status != EEPROM_OK)
        {
          return status;
        }
        WriteAddr +=  EEPROMPageSize;
        pBuffer += EEPROMPageSize;
      }
      if(numofsingle != 0)
      {
        /* Store the number of data to be written */
        dataindex = numofsingle;
        status = EEPROM_SelectedDevice->WritePage(pBuffer, WriteAddr, (uint32_t*)(&dataindex));
        if (status != EEPROM_OK)
        {
          return status;
        }
      }
    }
  }

  /* If all operations OK, return EEPROM_OK (0) */
  return EEPROM_OK;
}

/**
  * @brief  Basic management of the timeout situation.
  * @retval None.
  */
__weak void BSP_EEPROM_TIMEOUT_UserCallback(void)
{
}


/*
===============================================================================
                    ##### DATA EEPROM Programming functions #####
===============================================================================

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

 access_param( addrEnd <= FLASH_EEPROM_END );

 /* Wait for last operation to be completed */
 if( FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE) != HAL_OK){
   return HAL_ERROR;
 }

 for( ;addr < addrEnd; addr++ ) {
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

 access_param( addrEnd <= FLASH_EEPROM_END );

 while( addr < addrEnd) {
   /* Wait for last operation to be completed */
   status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);

   if(status == HAL_OK) {
     /* Clean the error context */
     *(__IO uint32_t *)addr++ = *data++;
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
