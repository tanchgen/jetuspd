#ifndef _GPIO_ARCH_H
#define _GPIO_ARCH_H

#include <stddef.h>

#include "stdint.h"
#include "stm32l1xx.h"

/* Таймаут готовности устройств на шине I2C */
#define I2C_READY_TOUT      1500

/** Тайм-аут переходного процесса при изменении состояния вывода GPIO. */
#define _GPIO_PIN_STATE_TRANSIENT_TOUT  (CONFIG_HZ)

#define SB1_KEY_TIM          TIM10
#define SB1_KEY_TIM_CLK_EN   RCC_APB2ENR_TIM10EN
#define SB1_KEY_TIM_IRQn     TIM10_IRQn
#define SB1_KEY_TIM_IRQH     TIM10_IRQHandler

#define SB2_KEY_TIM          TIM11
#define SB2_KEY_TIM_CLK_EN   RCC_APB2ENR_TIM11EN
#define SB2_KEY_TIM_IRQn     TIM11_IRQn
#define SB2_KEY_TIM_IRQH     TIM11_IRQHandler

//#define CLR_KEY_TIM          TIM14
//#define CLR_KEY_TIM_CLK_EN   RCC_APB1ENR_TIM14EN
//#define CLR_KEY_TIM_IRQn     TIM8_TRG_COM_TIM14_IRQn
//#define CLR_KEY_TIM_IRQH     TIM8_TRG_COM_TIM14_IRQHandler

//#define FRST_KEY_TIM          TIM13
//#define FRST_KEY_TIM_CLK_EN   RCC_APB1ENR_TIM13EN
//#define FRST_KEY_TIM_IRQn     TIM8_UP_TIM13_IRQn
//#define FRST_KEY_TIM_IRQH     TIM8_UP_TIM13_IRQHandler


//typedef enum {
//  I2C1_RST1,
//  I2C1_RST2,
//  I2C_RST_NUM
//} eI2cRst;

typedef enum {
  MCUSTATE_BAT_OFF,
  MCUSTATE_SYS_OFF,
  MCUSTATE_SYS_START,
  MCUSTATE_I2C_ON,
  MCUSTATE_RUN_1,
  MCUSTATE_RUN_2,
  MCUSTATE_PLL_LOCK,
  MCUSTATE_FPGA_DONE,
  MCUSTATE_ISTART,
  MCUSTATE_SYS_ON,
} eMcuState;

/* Перечеслитель состояний ВЫКЛ/НЕОПРЕДЕЛЕННОЕ/ВЫКЛ */
typedef enum {
  PWR_STATE_OFF,
  PWR_STATE_UNDEF,
  PWR_STATE_ON,
  PWR_STATE_ON_2,
} ePwrState;


// Состояние ВКЛЮЧЕНИЯ питания ПЛИС: состояния ПЕРЕД включением 1-го, 2-го или 3-го питания
#define IS_PWR_EN_ON_MCU_STATE(x) (x < MCUSTATE_SYS_ON)
// Состояние ВЫКЛЮЧЕНИЯ питания ПЛИС: состояния включенного 3-го, 2-го или 1-го питания
#define IS_PWR_EN_OFF_MCU_STATE(x) (x > MCUSTATE_SYS_OFF)

// Линии питания - связанные I2C-устройства
typedef enum {
  RUN_1,
  RUN_2,
  RUN_INTEL,
  RUN_NUM
} eRun;


typedef enum{
  OIL_LVL_1,
  OIL_LVL_2,
  OIL_LVL_3,
  OIL_LVL_NUM,
} eOilLvl;


typedef enum{
  PWR_EN_0,
  PWR_EN_1,
  PWR_EN_2,
  PWR_EN_3,
} ePwrEnState;

typedef struct {
  uint16_t temp;
  uint16_t tempRelOn;         /** < Температура FPGA_T_CH включения реле REL_ON */
  uint16_t tempRelOff;        /** < Температура FPGA_T_CH выключения реле REL_ON */
  struct gpio_pin *relOnPin; /** < Указатель на структуру GPIO_PIN для вывода REL_ON */
} sRelOnData;

typedef struct {
  uint32_t sysOffSet: 1;
  uint32_t sysOnSet: 1;
  uint32_t fpgaRstSet: 1;
  uint32_t fpgaRstEvnt: 1;
  uint32_t fpgaRstSensEvnt: 1;
  uint32_t fpgaProgSet: 1;
  uint32_t fpgaProgEvnt: 1;
  uint32_t fpgaRsrv1Evnt: 1;
  uint32_t upRstSet: 1;
  uint32_t upRstEvnt: 1;
  uint32_t dsw: 1;
  uint32_t iRstSet: 1;
  uint32_t iRstEvnt: 1;
  uint32_t caterrEvnt: 1;
  uint32_t mbAddrSet: 1;
  uint32_t mbAddrSetUpdate: 1;
  uint32_t mbIdErr: 1;
  uint32_t clrBatEvnt: 1;
} sSysFlags;


typedef union {
  struct {
#define DEVI_PROG_POS    0  // Позиция флага DONE_1
    uint32_t fpgaprog:  1;      // Флаг изменения состояния вывода
    uint32_t done:  1;       // Флаг изменения состояния вывода
    uint32_t mbid:  1;         // Флаг изменения состояния вывода
    uint32_t mbfin:  1;        // Флаг изменения состояния вывода
    uint32_t work:  1;         // Флаг изменения состояния вывода
    uint32_t rsrv1:  1;        // Флаг изменения состояния вывода
    uint32_t fpgarst:  1;       // Флаг изменения состояния вывода
    uint32_t plllock:  1;      // Флаг изменения состояния вывода
    uint32_t rsrv2:  1;        // Флаг изменения состояния вывода
    uint32_t mbaddr:  1;       // Флаг изменения состояния вывода
    uint32_t batoff:  1;      // Флаг изменения состояния вывода
    uint32_t ipwrout:  1;     // Флаг изменения состояния вывода
    uint32_t dsw:  1;         // Флаг изменения состояния вывода
    uint32_t irst:  1;        // Флаг изменения состояния вывода
    uint32_t slp0:  1;        // Флаг изменения состояния вывода
    uint32_t slp3:  1;        // Флаг изменения состояния вывода
    uint32_t slp4:  1;        // Флаг изменения состояния вывода
    uint32_t slp5:  1;        // Флаг изменения состояния вывода
    uint32_t slpsus:  1;      // Флаг изменения состояния вывода
    uint32_t caterr:  1;      // Флаг изменения состояния вывода
    uint32_t oil1:  1;        // Флаг изменения состояния вывода
    uint32_t oil2:  1;        // Флаг изменения состояния вывода
    uint32_t oil3:  1;        // Флаг изменения состояния вывода
    uint32_t rel:  1;         // Флаг изменения состояния вывода
    uint32_t rel380:  1;      // Флаг изменения состояния вывода
    uint32_t pump:  1;        // Флаг изменения состояния вывода
    uint32_t pwren1:  1;      // Флаг изменения состояния вывода
    uint32_t pwren2:  1;      // Флаг изменения состояния вывода
    uint32_t jmp1:  1;         // Флаг изменения параметра и/или порогов
    uint32_t rstkey:  1;       // Флаг изменения параметра и/или min-max
    uint32_t pwrkey:  1;       // Флаг изменения параметра и/или min-max
    uint32_t jtagsens:  1;      // Флаг изменения состояния вывода
  };
  uint32_t u32Devi;
} uGpioDeviation;

extern volatile FlagStatus sysAlrtOffQuery;
extern volatile FlagStatus sysOffQuery;

// Флаг перезагрузки MCU
extern FlagStatus mcuReset;

/** Флаг наличия ПРОЧИХ_ОШИБОК системы */
extern FlagStatus  otherError;
extern FlagStatus alrmFlag;
extern FlagStatus alertFlag;

/** Флаг направления запуска-останова системы on->off / off->on */
extern FlagStatus mcuRun;
extern FlagStatus mcuOnNeed;
extern FlagStatus onCan;

extern FlagStatus mcuRunWait;
extern eMcuState mcuState;

extern sSysFlags sysFlags;

/** Флаг направления включения-выключения системы:
 * SET - направление off->on
 * RESET - направление on->off
 */
extern FlagStatus sysRun;

void gpioIrqHandler5_9( uint32_t pin );
void gpioIrqHandler10_15( uint32_t pin );
/**
  * @brief  Обработка данных интерфейса GPIO.
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void gpioClock( void );

#endif /* _GPIO_ARCH_H */

