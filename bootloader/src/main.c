
/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
uFwVer fw1Ver;
uFwVer fw2Ver;
uFwRunState fwRunState;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
// Получение адреса последней рабочей прошивки
uint32_t fwAddrGet( void );
void flash_jump_to_app( uint32_t appAddr );
void _mDelay( uint32_t t_ms );

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

  fw1Ver.u32fwver = RTC->BKP1R;
  fw2Ver.u32fwver = RTC->BKP2R;
  fwRunState.u32fwst = RTC->BKP3R;

  // Ищем прошивку FW_1
  if( fw1Ver.u32fwver == 0 ){
    // Это первый запуск
    if( (*(uint32_t *)FW_1_START_ADDR & 0xFFFF0000) == 0x20000000 ){
      // Адрес вершины стека похож на правду
      /* Номер версии хранится по последнему адресу области памяти,
       * выделенной под прошивку
       */
      fw1Ver.u32fwver = *(uint32_t *)(FW_1_END+1 - sizeof(uint32_t));
      if( (fw1Ver.u32fwver < 0xFFFFFFFF) && (fw1Ver.u32fwver > 0) ){
        RTC->BKP1R = fw1Ver.u32fwver;
        fwRunState.fwstate.fw1Bug = fwRunState.fwstate.fw1Count >= 3;
      }
      else {
        // Некорректный номер версии
        return fwaddr;
      }
    }
    else {
      // Прошивка FW_1 должна быть обязательно!
      return fwaddr;
    }
  }
  else {
    fwRunState.fwstate.fw1Bug = fwRunState.fwstate.fw1Count >= 3;
  }

  // Ищем прошивку FW_2
  if( fw2Ver.u32fwver == 0 ){
    if( (*(uint32_t *)FW_2_START_ADDR & 0xFFFF0000) == 0x20000000 ){
      // Адрес вершины стека похож на правду
      /* Номер версии хранится по последнему адресу области памяти,
       * выделенной под прошивку
       */
      fw2Ver.u32fwver = *(uint32_t *)(FW_2_END+1 - sizeof(uint32_t));
      if( (fw2Ver.u32fwver < 0xFFFFFFFF) && (fw2Ver.u32fwver > 0) ){
        RTC->BKP2R = fw2Ver.u32fwver;
        fwRunState.fwstate.fw2Bug = fwRunState.fwstate.fw2Count >= 3;
      }
      else {
        // Некорректный номер версии
        fwRunState.fwstate.fw2Bug = 1;
        fw2Ver.u32fwver = 0;
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

  if( ((fw1Ver.u32fwver > fw2Ver.u32fwver) && (fwRunState.fwstate.fw1Bug == 0))
      || ((fwRunState.fwstate.fw2Bug == 1) && (fwRunState.fwstate.fw1Bug == 0))){
    fwRunState.fwstate.fw1Count++;
    fwaddr = FW_1_START_ADDR;
  }
  else if( (fw2Ver.u32fwver != 0)
      && (fwRunState.fwstate.fw2Bug == 0) ){
   fwRunState.fwstate.fw2Count++;
   fwaddr = FW_2_START_ADDR;
  }

//  RTC->BKP3R = fwRunState.u32fwst;

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

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
