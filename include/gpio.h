#ifndef _GPIO_H
#define _GPIO_H

#include <stdbool.h>

#include "stm32l1xx.h"
#include "stm32l1xx_hal.h"

#include "times.h"
//#include "gpio_arch.h"

#define GPIO_Pin_0 GPIO_PIN_0

#define KEY_IRQ_PRIORITY   3

enum {
  AF0 = 0,
  AF1 = 1,
  AF2 = 2,
  AF3 = 3,
  AF4 = 4,
  AF5 = 5,
  AF6 = 6,
  AF7 = 7,
  AF8 = 8,
  AF9 = 9,
  AF10 = 10,
  AF11 = 11,
  AF12 = 12,
  AF13 = 13,
  AF14 = 14,
  AF15 = 15,
};

typedef enum
{
  Bit_RESET = 0,
  Bit_SET
} BitAction;


//GPIO_MODE_INPUT
//GPIO_MODE_OUTPUT_PP
//GPIO_MODE_OUTPUT_OD
//GPIO_MODE_AF_PP
//GPIO_MODE_AF_OD
//
//GPIO_MODE_ANALOG
//
//GPIO_MODE_IT_RISING
//GPIO_MODE_IT_FALLING
//GPIO_MODE_IT_RISING_FALLING
//
//GPIO_MODE_EVT_RISING
//GPIO_MODE_EVT_FALLING
//GPIO_MODE_EVT_RISING_FALLING

//GPIO_NOPULL
//GPIO_PULLUP
//GPIO_PULLDOWN

//GPIO_SPEED_FREQ_LOW
//GPIO_SPEED_FREQ_MEDIUM
//GPIO_SPEED_FREQ_HIGH
//GPIO_SPEED_FREQ_VERY_HIGH

/** Структура дескриптора вывода GPIO. */
typedef struct {
  /** Дескриптор GPIO. */
  GPIO_TypeDef  *gpio;

  /** Номер вывода GPIO. */
  uint16_t  pin;

  /** Режим работы вывода GPIO. */
  uint32_t  mode;
  uint32_t pull;
  uint32_t speed;
  uint32_t af;

  /** Текущее и запрашиваемое состояние вывода GPIO. */
  BitAction  state, newstate;
  FlagStatus change;

} sGpioPin;

typedef struct {
  // ВХОДЫ - ПРЕРЫВАНИЯ
  uint16_t reserv0: 1;
  uint16_t reserv1: 1;
  uint16_t faultPiCtl: 1;       // Контроль входа прерывания GPIO FL_PI
  uint16_t pgLtm1Ctl: 1;        // Контроль входа прерывания GPIO PG_LTM1
  uint16_t pgLtm2Ctl: 1;        // Контроль входа прерывания GPIO PG_LTM2
  uint16_t pgMcuPwCtl: 1;       // Контроль входа прерывания GPIO PG_MCU_PW
  uint16_t pg5v0Ctl: 1;         // Контроль входа прерывания GPIO PG_5.0V
  uint16_t pg1v8Ctl: 1;         // Контроль входа прерывания GPIO PG_1.8V
  uint16_t reserv8: 1;
  uint16_t alrtLtm4Ctl: 1;      // Контроль входа прерывания GPIO nALERT_LTM4
  uint16_t alrtLtm3Ctl: 1;      // Контроль входа прерывания GPIO nALERT_LTM3
  uint16_t faultLtm4Ctl: 1;     // Контроль входа прерывания GPIO FAULT_LTM4
  uint16_t faultLtm3Ctl: 1;     // Контроль входа прерывания GPIO FAULT_LTM3
  uint16_t reserv13: 1;
  uint16_t uv0v85Ctl: 1;      // Контроль входа прерывания GPIO UV_0.85V
  uint16_t ov0v85Ctl: 1;      // Контроль входа прерывания GPIO OV_0.85V
} sExtiCtl;

extern sGpioPin  extiPinSb1Key;
extern sGpioPin  extiPinSb2Key;

extern sGpioPin  gpioPinO1;

extern sGpioPin  gpioPinSimPwr;
extern sGpioPin  gpioPinPwrKey;

extern sGpioPin  gpioPinFlashOn;
extern sGpioPin  gpioPinTermOn;

extern sGpioPin  gpioPinSensOn;

//// --------------------- CMOS_BAT -----------------------------------------------------
//extern sExtiPin  extiPinClrKey;
//extern sGpioPin  gpioPinBatOff;
//extern sGpioPin  gpioPinBatChk;
//
//// --------------- FPGA --------------------------
//extern sGpioPin  gpioPinFpgaDone;
//extern sGpioPin  gpioPinFpgaMbFin;
//extern sGpioPin  gpioPinFpgaWork;
//extern sExtiPin  extiPinFpgaRsrv1;
//extern sGpioPin  gpioPinFpgaMbId;
//
//extern sGpioPin  gpioPinFpgaMbSel;
//
//extern sGpioPin  gpioPinFpgaPllLock;
//extern sGpioPin  gpioPinFpgaRsrv2;
//extern sGpioPin  gpioPinFpgaRst;
//extern sGpioPin  gpioPinFpgaProg;
//extern sGpioPin  gpioPinFpgaMbAddr;
//
//extern sGpioPin  gpioPinJtagSens;
//
//// --------------- PLL ---------------------------
//extern sGpioPin  gpioPinPllPd;
//extern sGpioPin  gpioPinPllSync;
//extern sExtiPin  extiPinPllLock;
//
//// ---------------- DSW ---------------------------
//extern sGpioPin  gpioPinDsw;
//
//// ---------------- INTEL -------------------------
//// INTEL POWER
//extern sGpioPin  gpioPinPwrOut;
//extern sGpioPin  gpioPinIRst;
//
//extern sGpioPin  gpioPinSlpS[];
//extern sGpioPin  gpioPinCaterr;
//
//// ------------------- GPIO CTRL -----------------------------------------------------
//extern sGpioPin  gpioPinOil[];
//
//extern sGpioPin  gpioPinRel;
//extern sGpioPin  gpioPinRel380;
//extern sGpioPin  gpioPinPump;
//extern sGpioPin  gpioPinPwrUb1;
//extern sGpioPin  gpioPinPwrUb2;
//
//// -------------------- DBG_JMP -------------------
//extern sGpioPin  gpioPinDbgJmp1;

// ----------- TIMERS ---------------------------
/** Структуры дескриптора таймера антидребезга выводов GPIO. */
extern struct timer_list  sb1KeyDebounceTimer;
extern struct timer_list  sb2KeyDebounceTimer;

/** Структура дескриптора таймера таймаута изменения состояния системы при включении */
extern struct timer_list  pwrOnToutTimer;
/** Структура дескриптора таймера таймаута изменения состояния системы при выключении */
extern struct timer_list  pwrOffToutTimer;
/** Структура дескриптора таймера изменения состояния системы при включении*/
extern struct timer_list  pwrOnUpdateTimer;
/** Структура дескриптора таймера изменения состояния системы при выключении */
extern struct timer_list  pwrOffUpdateTimer;
extern struct timer_list  pwrToutTimer;


/** Структура дескриптора таймера сброса состояния вывода GPIO FPGA_Rst. */
extern struct timer_list  fpgaRstPulseTimer;
/** Структура дескриптора таймера сброса состояния вывода GPIO FPGA_Program DD1. */
extern struct timer_list  fpgaProgPulseTimer;
/** Структура дескриптора таймера сброса состояния вывода GPIO FPGA_RSRV2. */
extern struct timer_list  fpgaPllLockTimer;
extern struct timer_list  fpgaRstTestTimer;

/** Структура дескриптора таймера таймаута восстановления после ошибки */
extern struct timer_list  pwrOnCanTimer;

/** Структура дескриптора таймера таймаута Старта/Стопа INTEL. */
extern struct timer_list  iStartStopTimer;
/** Структура дескриптора таймера сброса состояния вывода GPIO I_PWRBtn_OUT. */
extern struct timer_list  pwrBtnOutTimer;

/**
  * @brief  Установка режима работы вывода GPIO.
  *
  * @param[in]  pin дескриптор вывода GPIO
  * @param[in]  mode  режим работы вывода GPIO - in/out
  * @param[in]  otype режим работы вывода GPIO - OD/PP
  * @param[in]  pupd  режим работы вывода GPIO - pullup/pulldown
  *
  * @retval none
  */
void setModeGpioPin(sGpioPin *pin, uint32_t mode, uint32_t pull, uint32_t speed, uint8_t af);

/**
  * @brief  Безотлогательная установка вывода GPIO.
  *
  * @param[in]  *pin указатель на дескриптор вывода GPIO
  *
  * @retval none
  */
void gpioPinSetNow( sGpioPin *pin );


/**
  * @brief  Безотлогательный сброс вывода GPIO.
  *
  * @param[in]  *pin указатель на дескриптор вывода GPIO
  *
  * @retval none
  */
void gpioPinResetNow( sGpioPin * pin );

void gpioPinSet( sGpioPin * pin );
void gpioPinReset( sGpioPin * pin );

void gpioPinCmd( sGpioPin *pin, BitAction act );
void gpioPinCmdNow( sGpioPin * pin, BitAction cmd );

//BitAction  gpioPinRead( sGpioPin * pin );
//BitAction  gpioPinReadNow( sGpioPin * pin );
//
//BitAction  extiPinRead( sExtiPin * pin );
//BitAction  extiPinReadNow( sExtiPin * pin );

void gpioInit( void);
void gpioClock( void);

/**
  * @brief  Безотлогательная установка вывода GPIO, вызываемая из таймера.
  *
  * @param[in]  *pin указатель на дескриптор вывода GPIO
  *
  * @retval none
  */
void gpioPinResetNowTout( uintptr_t arg );

/**
  * @brief  Инициализация вывода GPIO.
  *
  * @param[in]  pin дескриптор вывода GPIO
  * @param[in]  mode  режим работы вывода GPIO
  *
  * @retval none
  */
void gpioPinSetup(sGpioPin *pin);

/**
  * @brief  Инициализация входа EXTI.
  *
  * @param[in]  pin дескриптор вывода GPIO
  *
  * @retval none
  */
void extiPinSetup(sGpioPin *pin);

/**
  * @brief  Чтение состояния вывода GPIO.
  *
  * @param[in]  pin   дескриптор вывода GPIO
  *
  * @retval Bit_SET   бит установлен в состояние логической 1
  * @retval Bit_RESET бит установлен в состояние логического 0
  */
BitAction read_gpio_pin_state(sGpioPin *pin);

/**
  * @brief  Установка вывода GPIO в заданное состояние.
  *
  * @param[in]  pin   дескриптор вывода GPIO
  * @param[in]  newstate  новое состояние вывода GPIO
  *
  * @retval true    состояние изменилось
  * @retval false   состояние не изменилось
  */
bool write_gpio_pin_state(sGpioPin *pin, BitAction newstate);

/**
  * @brief  Переключение состояния вывода GPIO.
  *
  * @param[in]  pin дескриптор вывода GPIO
  *
  * @retval true  состояние изменилось
  * @retval false состояние не изменилось
  */
__STATIC_INLINE bool toggle_gpio_pin_state(sGpioPin *pin)
{
  return write_gpio_pin_state(pin, (pin->state == Bit_SET) ? Bit_RESET : Bit_SET);
}

bool changePinState(sGpioPin *pin );

/**
  * @brief  Проверка и изменение состояния вывода GPIO по таймауту таймера.
  *
  * @param[in]  agr  значение указателя на дескриптор вывода GPIO
  *
  * @retval none
  */
void changePinStateTout( uintptr_t arg );

/**
  * @brief  Инициализация подсистемы GPIO.
  *
  * @param  none
  *
  * @retval none
  */
void init_gpio(void);

/**
  * @brief  Обработчик таймаута положительного импульса
  *
  * @param[in]  arg  указатель на структуру GPIO-вывода
  *
  * @retval none
  */
void gpioPosPulseTout(uintptr_t arg);

/**
  * @brief  Обработчик таймаута отрицательного импульса
  *
  * @param[in]  arg  указатель на структуру GPIO-вывода
  *
  * @retval none
  */
void gpioNegPulseTout(uintptr_t arg);

void togglePinStateTout( uintptr_t arg );

static inline FlagStatus gpioPinState( sGpioPin *pin ){
  return (pin->gpio->IDR & pin->pin)? SET: RESET;
}

static inline uint8_t gpioPortNum( GPIO_TypeDef *port){
  assert_param(IS_GPIO_ALL_INSTANCE(port));
  uint8_t pn;

  pn = ((uint32_t)port - GPIOA_BASE)/(GPIOB_BASE - GPIOA_BASE);

  return pn;
}

static inline uint8_t gpioPinNum( uint32_t pin ){
  uint8_t i;

  assert_param(IS_GPIO_PIN(pin));

  for( i = 0; (pin & 0x1) == 0; i++ ){
    pin >>= 1;
  }
  return i;
}

static inline BitAction  gpioPinRead( sGpioPin * pin ){
  BitAction bit = (pin->gpio->IDR & pin->pin)? Bit_SET : Bit_RESET;

  if( bit != pin->newstate ){
    pin->newstate = bit;
    pin->change = SET;
  }
  return bit;
}

static inline BitAction  gpioPinReadNow( sGpioPin * pin ){
  BitAction bit = (pin->gpio->IDR & pin->pin)? Bit_SET : Bit_RESET;
  if( bit != pin->newstate ){
    pin->state = pin->newstate = bit;
    pin->change = SET;
  }
  else {
    pin->state = bit;
  }
  return bit;
}


#endif /* _GPIO_H */

