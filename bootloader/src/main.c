
/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"
#include "stm32l1xx_ll_iwdg.h"
#include "stm32l1xx_ll_rcc.h"
#include "main.h"
#include "fw.h"

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
//uFwVer fw1Ver;
//uFwVer fw2Ver;
uFwRunState fwRunState;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
void Configure_IWDG(void);
void Check_IWDG_Reset(void);

/* Private function prototypes -----------------------------------------------*/
// Получение адреса последней рабочей прошивки
uint32_t fwAddrGet( void );
void flash_jump_to_app( uint32_t appAddr );
void _mDelay( uint32_t t_ms );

HAL_StatusTypeDef   stmEeRead( uint32_t addr, uint32_t * data, uint32_t datalen) ;
HAL_StatusTypeDef   stmEeWrite( uint32_t addr, uint32_t * data, uint32_t datalen) ;

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void) {
  uint32_t fwaddr;

  /* USER CODE BEGIN SysInit */
  MAYBE_BUILD_BUG_ON( FLASH_END < FW_2_END );
  /* USER CODE END SysInit */

  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  PWR->CR |= PWR_CR_DBP;

  // Получение адреса последней рабочей прошивки
  if( (fwaddr = fwAddrGet()) == 0){
    // Нет нормальной рабочей прошивки
    // TODO: Сделать частое мигание красным светодиодом
    // Включаем SysTick
    SysTick_Config( SystemCoreClock / 1000 );
    // Включаем GPIOB
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER9) | (GPIO_MODER_MODER9_0);
    while(1){
      GPIOB->BSRR = GPIO_PIN_9;
      _mDelay( 150 );
      GPIOB->BSRR = GPIO_PIN_9 << 16;
      _mDelay( 150 );
    }
  }
  else {
    // Запускаем watchdog на случай зависания прошивки
    Check_IWDG_Reset();
    Configure_IWDG();

    flash_jump_to_app( fwaddr );
  }
  /* Infinite loop */
  while (1)
  {}

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

// Получение адреса последней рабочей прошивки
uint32_t fwAddrGet( void ){
  uint32_t fwaddr = 0;
  sFwHandle * eeFwh = (sFwHandle *)FW_HANDLE_ADDR_0;
  sFw tmpfw = {0};

  uint32_t fw1Ver;
  uint32_t fw2Ver;

  fw1Ver = RTC->BKP1R;
  fw2Ver = RTC->BKP2R;
  fwRunState.u32fwst = RTC->BKP3R;

  // Ищем прошивку FW_1
  if( fw1Ver == 0 ){
    // Это первый запуск
    stmEeRead( (uint32_t)&(eeFwh->fw[FW_1]), (uint32_t*)&tmpfw, sizeof(sFw));

    if( (tmpfw.good == SET) && ((*(uint32_t *)FW_1_START_ADDR & 0xFFFF0000) == 0x20000000) ){
      // Флаг "хорошей" прошивки и Адрес вершины стека похож на правду

      /* Номер версии хранится по последнему адресу области памяти,
       * выделенной под прошивку
       */
      fw1Ver = tmpfw.fwVer;
      if( (fw1Ver < 0xFFFFFFFF) && (fw1Ver > 0) ){
        RTC->BKP1R = fw1Ver;
//        fwRunState.fwstate.fw1Bug = fwRunState.fwstate.fw1Count >= 3;
      }
      else {
        // Некорректный номер версии
        return fwaddr;
      }
    }
    else if( (*(uint32_t *)FW_1_START_ADDR & 0xFFFF0000) == 0x20000000 ){
      // Адрес вершины стека похож на правду
      /* Номер версии хранится по последнему адресу области памяти,
       * выделенной под прошивку
       */
      fw1Ver = 0x00000001;

      RTC->BKP1R = fw1Ver;
  //    fwRunState.fwstate.fw1Bug = fwRunState.fwstate.fw1Count >= 3;
    }
    else {
      // Прошивка FW_1 должна быть обязательно!
      return fwaddr;
    }
  }
  else {
//    fwRunState.fwstate.fw1Bug = fwRunState.fwstate.fw1Count >= 3;
  }
  fwRunState.fwstate.fw1Bug = fwRunState.fwstate.fw1Count >= 3;

  if( fwRunState.fwstate.fw1Bug ){
    // Прошивка #1 "плохая" !
    tmpfw.good = 0;
    stmEeWrite( (uint32_t)&(eeFwh->fw[FW_1]), (uint32_t *)&tmpfw, sizeof(sFw));
    fw1Ver = 0;
  }

  // Ищем прошивку FW_2
  if( fw2Ver == 0 ){
    stmEeRead( (uint32_t)&(eeFwh->fw[FW_2]), (uint32_t*)&tmpfw, sizeof(sFw));

    if( (tmpfw.good == SET) && ((*(uint32_t *)FW_2_START_ADDR & 0xFFFF0000) == 0x20000000) ){
      // Флаг "хорошей" прошивки и Адрес вершины стека похож на правду

      /* Номер версии хранится по последнему адресу области памяти,
       * выделенной под прошивку
       */
      fw2Ver = tmpfw.fwVer;
      if( (fw2Ver < 0xFFFFFFFF) && (fw2Ver > 0) ){
        RTC->BKP2R = fw2Ver;
        fwRunState.fwstate.fw2Bug = fwRunState.fwstate.fw2Count >= 3;
      }
      else {
        // Некорректный номер версии
        fwRunState.fwstate.fw2Bug = 1;
        fw2Ver = 0;
      }
    }
    else {
      // Прошивки нет или она плохая
      fwRunState.fwstate.fw2Bug = 1;
    }
  }
  else {
    fwRunState.fwstate.fw2Bug = fwRunState.fwstate.fw2Count >= 3;
  }

  if( fwRunState.fwstate.fw2Bug ){
    // Прошивка #2 "плохая" !
    tmpfw.good = 0;
    stmEeWrite( (uint32_t)&(eeFwh->fw[FW_2]), (uint32_t *)&tmpfw, sizeof(sFw));
    fw2Ver = 0;
  }


  if( fw1Ver && ((fw1Ver >= fw2Ver) || (fw2Ver == 0)) ){
    fwRunState.fwstate.fw1Count++;
    fwaddr = FW_1_START_ADDR;
  }
  else if( fw2Ver ){
   fwRunState.fwstate.fw2Count++;
   fwaddr = FW_2_START_ADDR;
  }

  RTC->BKP3R = fwRunState.u32fwst;

  return fwaddr;
}



/* USER CODE BEGIN 4 */
/**
 * @brief   Actually jumps to the user application.
 * @param   void
 * @return  void
 */
void flash_jump_to_app( uint32_t appAddr )
{
  /* Function pointer to the address of the user application. */
  fnc_ptr jump_to_app;
  jump_to_app = (fnc_ptr)(*(volatile uint32_t*) (appAddr + 4u));
  HAL_DeInit();
  /* Change the main stack pointer. */
  __set_MSP( *(volatile uint32_t*)appAddr );
  jump_to_app();
}


// Задержка по SysTick без прерывания
void _mDelay( uint32_t t_ms ){
//  SysTick->VAL = 0;
  while ( t_ms > 0 ){
    while ( !( SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk ) ) // wait for underflow
    {}
    t_ms--;
  }
}

/**
* @brief  This function configures IWDG
* @param  None
* @retval None
*/
void Configure_IWDG(void){
  /* Enable the peripheral clock of DBG register (uncomment for debug purpose) */
  /* ------------------------------------------------------------------------- */
  /*  LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_APB1_GRP1_IWDG_STOP); */

  /* Enable the peripheral clock IWDG */
  /* -------------------------------- */
  LL_RCC_LSI_Enable();
  while (LL_RCC_LSI_IsReady() != 1)
  {}

  /* Configure the IWDG with window option disabled */
  /* ------------------------------------------------------- */
  /* (1) Enable the IWDG by writing 0x0000 CCCC in the IWDG_KR register */
  /* (2) Enable register access by writing 0x0000 5555 in the IWDG_KR register */
  /* (3) Write the IWDG prescaler by programming IWDG_PR from 0 to 7 - LL_IWDG_PRESCALER_4 (0) is lowest divider*/
  /* (4) Write the reload register (IWDG_RLR) */
  /* (5) Wait for the registers to be updated (IWDG_SR = 0x0000 0000) */
  /* (6) Refresh the counter value with IWDG_RLR (IWDG_KR = 0x0000 AAAA) */
  LL_IWDG_Enable(IWDG);                             /* (1) */
  LL_IWDG_EnableWriteAccess(IWDG);                  /* (2) */
  LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_32);  /* (3) */
  LL_IWDG_SetReloadCounter(IWDG, 0xFEE);            /* (4) */
  while (LL_IWDG_IsReady(IWDG) != 1)                /* (5) */
  {}
  LL_IWDG_ReloadCounter(IWDG);                      /* (6) */
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
}


/**
  * @brief  This function check if the system has resumed from IWDG reset
  * @param  None
  * @retval None
  */
void Check_IWDG_Reset(void){
  if (LL_RCC_IsActiveFlag_IWDGRST())
  {
    /* clear IWDG reset flag */
    LL_RCC_ClearResetFlags();
  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line){
  (void)file;
  (void)line;
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line){
  (void)file;
  (void)line;
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

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
