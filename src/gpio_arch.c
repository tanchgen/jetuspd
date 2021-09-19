#include <gpio_arch.h>
#include <led.h>
#include <stddef.h>
#include <stdbool.h>
#include <limits.h>

#include "main.h"
//#include "usart_arch.h"



/** Флаг наличия ПРОЧИХ_ОШИБОК системы */
FlagStatus  otherError = RESET;

/** Флаг наличия ОШИБОК системы */
FlagStatus  alertFlag = RESET;

/** Флаг направления включения-выключения системы:
 * SET - направление off->on
 * RESET - направление on->off
 */
FlagStatus mcuRun = RESET;

/** Флаг необходимости включения системы когда это станет возможно:
 * SET - направление off->on
 * RESET - направление on->off
 */
FlagStatus simOnNeed = RESET;

/** Флаг разрешения включения системы когда это необходимо:
 * SET - включение возможно
 * RESET - включение неразрешено
 */
FlagStatus onCan = SET;

/** Флаг ожидания окончания выполнения данного этапа
 * SET - этап выполняется
 * RESET - этап окончен
 */
FlagStatus simRunWait = SET;


/** Состояние уровня запуска системы */
eMcuState mcuState = MCUSTATE_SYS_OFF;

// -------------- POWER ------------------------------
// KEYS
/** Структура дескриптора вывода GPIO SB1_Key. */
sGpioPin  extiPinSb1Key = {GPIOB, GPIO_PIN_1, GPIO_MODE_IT_RISING_FALLING, GPIO_PULLDOWN, GPIO_SPEED_FREQ_LOW, AF0, Bit_SET, Bit_SET, RESET };
/** Структура дескриптора вывода GPIO SB2_Key. */
sGpioPin  extiPinSb2Key = {GPIOB, GPIO_PIN_3, GPIO_MODE_IT_RISING_FALLING, GPIO_PULLDOWN, GPIO_SPEED_FREQ_LOW, AF0, Bit_SET, Bit_SET, RESET };

sGpioPin  gpioPinO1 = {GPIOB, GPIO_PIN_5, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_LOW, AF0, Bit_RESET, Bit_RESET, RESET };

sGpioPin  gpioPinSimPwr = {GPIOB, GPIO_PIN_12, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_LOW, AF0, Bit_SET, Bit_SET, RESET };
sGpioPin  gpioPinPwrKey = {GPIOB, GPIO_PIN_2, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_LOW, AF0, Bit_RESET, Bit_RESET, RESET };

sGpioPin  gpioPinEeOn = {GPIOA, GPIO_PIN_12, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_LOW, AF0, Bit_RESET, Bit_RESET, RESET };
sGpioPin  gpioPinTermOn = {GPIOA, GPIO_PIN_15, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_LOW, AF0, Bit_RESET, Bit_RESET, RESET };

sGpioPin  gpioPinSensOn = {GPIOB, GPIO_PIN_4, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_LOW, AF0, Bit_SET, Bit_SET, RESET };
//    {GPIOD, GPIO_Pin_14, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET, RESET },
//    {GPIOE, GPIO_Pin_2, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET, RESET }
//};
//
//// --------------------- CMOS_BAT -----------------------------------------------------
///** Структура дескриптора вывода GPIO UP_CLRKey. Срабатывает только при ОТПУСКАНИИ кнопки*/
//sExtiPin  extiPinClrKey = {GPIOE, GPIO_Pin_7, GPIO_PuPd_UP, EXTI_Trigger_Rising_Falling, Bit_SET, Bit_SET, RESET };
///** Структуры дескрипторов выводов GPIO MCU_BAT_OFF */
//sGpioPin  gpioPinBatOff = {GPIOE, GPIO_Pin_8, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET, RESET };
///** Структуры дескрипторов выводов GPIO BAT_DISCHARGE */
//sGpioPin  gpioPinBatChk = {GPIOE, GPIO_Pin_9, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET, RESET };
//
//// --------------- FPGA --------------------------
//sGpioPin  gpioPinFpgaDone = {GPIOG, GPIO_Pin_15, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_DOWN, Bit_RESET, Bit_RESET, RESET };
//sGpioPin  gpioPinFpgaMbFin = {GPIOG, GPIO_Pin_12, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_DOWN, Bit_RESET, Bit_RESET, RESET };
//sGpioPin  gpioPinFpgaWork = {GPIOH, GPIO_Pin_12, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_DOWN, Bit_RESET, Bit_RESET, RESET };
//sExtiPin  extiPinFpgaRsrv1 = {GPIOH, GPIO_Pin_13, GPIO_PuPd_UP,  EXTI_Trigger_Rising_Falling, Bit_RESET, Bit_RESET, RESET };
//sGpioPin  gpioPinFpgaMbId = {GPIOH, GPIO_Pin_14, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_DOWN, Bit_RESET, Bit_RESET, RESET };
//
//sGpioPin  gpioPinFpgaMbSel = {GPIOG, GPIO_Pin_6, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET, RESET };
//
//sGpioPin  gpioPinFpgaPllLock = {GPIOG, GPIO_Pin_0, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET, RESET };
//sGpioPin  gpioPinFpgaRsrv2 = {GPIOG, GPIO_Pin_1, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET, RESET };
//sGpioPin  gpioPinFpgaRst = {GPIOG, GPIO_Pin_2, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET, RESET };
//sGpioPin  gpioPinFpgaProg = {GPIOG, GPIO_Pin_7, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET, RESET };
//sGpioPin  gpioPinFpgaMbAddr = {GPIOG, GPIO_Pin_8, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET, RESET };
//
//sGpioPin  gpioPinJtagSens= {GPIOD, GPIO_Pin_0, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_UP, Bit_SET, Bit_SET, RESET };
//
//sSysFlags sysFlags = {
//  .sysOffSet = RESET,
//  .sysOnSet = RESET,
//  .fpgaRstEvnt = RESET,
//  .fpgaRstSensEvnt = RESET,
//  .fpgaProgEvnt = RESET,
//  .mbAddrSet = SET,
//};
//
//// --------------- PLL ---------------------------
//sGpioPin  gpioPinPllPd = {GPIOI, GPIO_Pin_7, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET, RESET };
//sGpioPin  gpioPinPllSync = {GPIOI, GPIO_Pin_8, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET, RESET };
//sExtiPin  extiPinPllLock = {GPIOI, GPIO_Pin_5, GPIO_PuPd_NOPULL, EXTI_Trigger_Rising_Falling, Bit_RESET, Bit_RESET, RESET };
//
//// ---------------- DSW ---------------------------
///** Структура дескриптора вывода GPIO UP_DSW_+3.3V. */
//sGpioPin  gpioPinDsw = {GPIOF, GPIO_Pin_6, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET, RESET };
//
//// ---------------- INTEL -------------------------
//// INTEL POWER
//sGpioPin  gpioPinPwrOut = {GPIOD, GPIO_Pin_11, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET, RESET };
///** Структура дескриптора вывода GPIO UP_I_RST. */
//sGpioPin  gpioPinIRst = {GPIOE, GPIO_Pin_15, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET, RESET };
//
//sGpioPin  gpioPinSlpS[SLP_NUM] = {
//  {GPIOF, GPIO_Pin_7, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_UP, Bit_RESET, Bit_RESET, RESET },  //Slp_S0
//  {GPIOF, GPIO_Pin_9, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_UP, Bit_RESET, Bit_RESET, RESET },  //Slp_S3
//  {GPIOF, GPIO_Pin_10, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_UP, Bit_RESET, Bit_RESET, RESET },  //Slp_S4
//  {GPIOF, GPIO_Pin_11, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_UP, Bit_RESET, Bit_RESET, RESET },  //Slp_S5
//  {GPIOF, GPIO_Pin_14, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_UP, Bit_RESET, Bit_RESET, RESET },  //Slp_Sus
//  {GPIOF, GPIO_Pin_12, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_UP, Bit_RESET, Bit_RESET, RESET },  //Slp_A
//};
///** Структура дескриптора вывода GPIO UP_I_CATERR. */
//sGpioPin  gpioPinCaterr = {GPIOF, GPIO_Pin_15, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_UP, Bit_SET, Bit_SET, RESET };
//
//// ------------------- GPIO CTRL -----------------------------------------------------
///** Структура дескриптора вывода GPIO OIL_LEVEL. */
//sGpioPin  gpioPinOil[OIL_LVL_NUM] = {
//  {GPIOD, GPIO_Pin_0, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET, RESET },
//  {GPIOD, GPIO_Pin_1, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET, RESET },
//  {GPIOD, GPIO_Pin_2, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET, RESET },
//};
//
///** Структура дескриптора вывода GPIO RELAY. */
//sGpioPin  gpioPinRel = {GPIOE, GPIO_Pin_3, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET, RESET };
//sGpioPin  gpioPinRel380 = {GPIOE, GPIO_Pin_4, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET, RESET };
//sGpioPin  gpioPinPump = {GPIOE, GPIO_Pin_5, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET, RESET };
///** Структура дескриптора вывода GPIO PWR_EN. */
//sGpioPin  gpioPinPwrUb1 = {GPIOE, GPIO_Pin_0, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET, RESET };
//sGpioPin  gpioPinPwrUb2 = {GPIOE, GPIO_Pin_1, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_RESET, Bit_RESET, RESET };

// ------------------- LEDS -----------------------
// Определено в led.c

// -------------------- DBG_JMP -------------------
//sGpioPin  gpioPinDbgJmp1 = {GPIOF, GPIO_Pin_3, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_NOPULL, Bit_SET, Bit_SET, RESET };

// ===========================================================================================================
/** Структуры дескриптора таймера антидребезга выводов GPIO. */
struct timer_list  sb1KeyDebounceTimer;
struct timer_list  sb2KeyDebounceTimer;

///** Структура дескриптора таймера таймаута аварийного выключения системы*/
//struct timer_list  pwrOffAlrmTimer;
///** Структура дескриптора таймера таймаута изменения состояния системы при включении */
//struct timer_list  pwrOnToutTimer;
///** Структура дескриптора таймера таймаута изменения состояния системы при выключении */
//struct timer_list  pwrOffToutTimer;
///** Структура дескриптора таймера изменения состояния системы при включении*/
//struct timer_list  pwrOnUpdateTimer;
///** Структура дескриптора таймера изменения состояния системы при выключении */
//struct timer_list  pwrOffUpdateTimer;
//struct timer_list  pwrToutTimer;


///** Структура дескриптора таймера сброса состояния вывода GPIO FPGA_Rst. */
//struct timer_list  fpgaRstPulseTimer;
///** Структура дескриптора таймера сброса состояния вывода GPIO FPGA_Program DD1. */
//struct timer_list  fpgaProgPulseTimer;
/////** Структура дескриптора таймера сброса состояния вывода GPIO FPGA_RSRV2. */
////struct timer_list  fpgaPllLockTimer;
//struct timer_list  fpgaRstTestTimer;

///** Структура дескриптора таймера таймаута Старта/Стопа INTEL. */
//struct timer_list  iStartStopTimer;
///** Структура дескриптора таймера сброса состояния вывода GPIO I_PWRBtn_OUT. */
//struct timer_list  pwrBtnOutTimer;
//
///** Структура дескриптора таймера таймаута восстановления после ошибки */
struct timer_list  pwrOnCanTimer;

// =================== Прототипы функций ====================================
// ==========================================================================


/**
  * @brief  Обработчик таймаута восстановления после ошибки
  *
  * @param[in]  arg  NULL
  *
  * @retval none
  */
static void pwrOnCan(uintptr_t arg){
  (void)arg;
  // Система выключена полностью и готова к повторному включению
  // Восстанавливаем моргание зеленого светодиода
  onCan = SET;
}

/**
  * @brief  Обработчик таймаута ожидания условия при вкл/выкл питания
  *
  * @param[in]  arg  NULL
  *
  * @retval none
  */
//static void pwrTimeout(uintptr_t arg){
//  (void)arg;
//  // Включаем на постоянно светодиод LED_PWR_FAULE
//  trace_puts("PWR_TOUT");
//  ledOn( LED_R, 0 );
//}


/**
  * @brief  Обработчик тайм-аута включения системы
  *
  * @param[in]  arg  NULL
  *
  * @retval none
  */
//static void pwrOnTout(uintptr_t arg){
//
//  // Не возникло событие успешного завершения очередного этапа включения системы
//  trace_printf("PWR ON tout State: %d\n", arg);
//
//  switch( arg ){
//    case MCUSTATE_SYS_START:
//#if DEBUG_ALARM
//        trace_puts("NOTICE: Not all I2C_Devices is ready." );
//#endif
////        ledToggleSet( LED_R, LED_TOGGLE_TOUT, LED_TOGGLE_TOUT, 0 );
//      break;
//    default:
//      break;
//  }
////  errHandler(ERR_SYS_ON_TOUT);
//}

/**
  * @brief  Обработчик тайм-аута выключения системы
  *
  * @param[in]  arg  NULL
  *
  * @retval none
  */
//static void pwrOffTout(uintptr_t arg){
//  (void)arg;
//  // Не возникло событие успешного завершения очередного этапа включения системы
////  errHandler(ERR_SYS_OFF_TOUT);
//  mcuState--;
//  simRunWait = RESET;
//}

/**
  * @brief  Обработчик тайм-аута включения системы
  *
  * @param[in]  arg  NULL
  *
  * @retval none
  */
//static void pwrOnUpdate(uintptr_t arg){
//  (void)arg;
//  // Пауза закончилась - переходим в предыдущее состояние системы
//  mcuState++;
//  simRunWait = RESET;
//}

/**
  * @brief  Обработчик тайм-аута выключения системы
  *
  * @param[in]  arg  NULL
  *
  * @retval none
  */
//static void pwrOffUpdate(uintptr_t arg){
//  (void)arg;
//  // Пауза закончилась - переходим в предыдущее состояние системы
//  simRunWait = RESET;
//  if( mcuState > MCUSTATE_SYS_OFF ){
//    mcuState--;
//  }
//  else {
//    pwrOnCan( (uintptr_t)NULL );
//  }
//}


/**
  * @brief  Обработчик тайм-аута антидребезга выводов GPIO.
  *
  * @param[in]  arg данные таймера (дескриптор таймера)
  *
  * @retval none
  */
static void debounceTimeout(uintptr_t arg){
  sGpioPin *pin = (sGpioPin*)arg;
  bool st;

  // Нынешнее состояния пина
  st = ((((pin->gpio)->IDR & (pin->pin))) == pin->pin);

  if( pin->newstate != st ){
    // Состояние сохранилось - НЕ ложное срабатывание
    pin->newstate = st;
    pin->change = SET;
  }

  if( pin->mode & EXTI_IT ){
    // Включаем прерывание
    EXTI->IMR |= pin->pin;
  }
}


void SB1_KEY_TIM_IRQH( void ) {
  SB1_KEY_TIM->SR = 0;
  debounceTimeout( (uintptr_t)&extiPinSb1Key );
}

void SB2_KEY_TIM_IRQH( void ) {
  SB2_KEY_TIM->SR = 0;
  debounceTimeout( (uintptr_t)&extiPinSb2Key );
}
/*
void FRST_KEY_TIM_IRQH( void ) {
  FRST_KEY_TIM->SR = 0;
  debounceTimeout( (uintptr_t)&extiPinFpgaRstKey );
}
*/

/**
  * @brief  Настройка таймера антидребезга кнопки.
  *
  * @param[in]  arg данные таймера (дескриптор таймера)
  *
  * @retval none
  */
void keyTimInit( TIM_TypeDef * keytim ){
  FlagStatus rc;
  uint16_t psc;

  rc = timPscSet( keytim, 100000, &psc );
  assert_param( rc == RESET );
  keytim->PSC = psc;
  // Время работы таймера 30мс = 0.030с / (1/10000Гц) .
  keytim->ARR = ( 100000/1000 * 30  ) -1;
  keytim->CR1 |= TIM_CR1_OPM;
  keytim->EGR |= TIM_EGR_UG;
  while( (keytim->SR & TIM_SR_UIF) == RESET )
  {}
  keytim->SR &= ~TIM_SR_UIF;
  keytim->DIER |= TIM_DIER_UIE;
}

void keyInit( void ){

  RCC->APB2ENR |= SB1_KEY_TIM_CLK_EN;
  RCC->APB2ENR |= SB2_KEY_TIM_CLK_EN;
//  RCC->APB1ENR |= FRST_KEY_TIM_CLK_EN;

  keyTimInit( SB1_KEY_TIM );
  keyTimInit( SB2_KEY_TIM );
//  keyTimInit( FRST_KEY_TIM );

  enable_nvic_irq( SB1_KEY_TIM_IRQn, KEY_IRQ_PRIORITY);
  enable_nvic_irq( SB2_KEY_TIM_IRQn, KEY_IRQ_PRIORITY);
//  enable_nvic_irq( FRST_KEY_TIM_IRQn, KEY_IRQ_PRIORITY);
}

void gpioIrqHandler5_9( uint32_t pin ){
  (void)pin;
}


void gpioIrqHandler10_15( uint32_t pin ){
  (void)pin;
}


/**
  * @brief  Обработка данных интерфейса GPIO.
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void gpioClock( void ){

#if DEBUG_TRACE
  trace_puts("Function: Clock FPGA");
#endif

  //  Обработаем кнопки.
  // SB1_Key
  if( extiPinSb1Key.change && (extiPinSb1Key.state == Bit_SET) ){

  }

  // SB2_Key
  if( extiPinSb2Key.change && (extiPinSb1Key.state == Bit_RESET) ){

  }



}

/**
  * @brief	Разрешение работы интерфейса GPIO.
  *
  * @param[in]	self	дескриптор интерфейса
  *
  * @retval	none
  */
void gpioEnable( void ) {
#if DEBUG_TRACE
  trace_puts("Function: Enable FPGA");
#endif

  // Включаем прерывания для кнопок
  EXTI->IMR |= extiPinSb1Key.pin;
  EXTI->IMR |= extiPinSb2Key.pin;
}

/**
  * @brief	Инициализация интерфейса GPIO.
  *
  * @param[in]	self	дескриптор интерфейса
  *
  * @retval	none
  */
void gpioInit( void ){
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // ------------- OTHER PINS -------------------
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIOC pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIOH pins : PH0 PH1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIOA pins */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // -------------- POWER -----------------------
  // KEYS
  /** Структура дескриптора вывода GPIO UP_RSTKey. */
  extiPinSetup( &extiPinSb1Key );
  /** Структура дескриптора вывода GPIO UP_PWRKey. */
  extiPinSetup( &extiPinSb2Key );
  keyInit();

  gpioPinSetup( &gpioPinO1 );

  gpioPinSetup( &gpioPinSimPwr );
  gpioPinSetup( &gpioPinPwrKey );

  gpioPinSetup( &gpioPinEeOn );
  gpioPinSetup( &gpioPinTermOn );

  // ------------------- LEDS -----------------------
  // Определено в led.c
  ledInit();

	// ------------------	Конфигурация таймеров ----------------------------------
  // Таймеры таймаута дребезга кнопок
  timerSetup( &sb1KeyDebounceTimer, debounceTimeout, (uintptr_t)&extiPinSb1Key );
  timerSetup( &sb2KeyDebounceTimer, debounceTimeout, (uintptr_t)&extiPinSb2Key );

//  timerSetup( &pwrOnToutTimer, pwrOnTout, (uintptr_t)NULL);
//  timerSetup( &pwrOffToutTimer, pwrOffTout, (uintptr_t)NULL);
//  timerSetup( &pwrOnUpdateTimer, pwrOnUpdate, (uintptr_t)NULL);
//  timerSetup( &pwrOffUpdateTimer, pwrOffUpdate, (uintptr_t)NULL);
//  timerSetup( &pwrToutTimer, pwrTimeout, (uintptr_t)mcuState );

  timerSetup( &pwrOnCanTimer, pwrOnCan, (uintptr_t)NULL );

}


