/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "diag/trace.h"

#include "tinyalloc.h"
#include "times.h"
#include "gpio.h"
#include "led.h"
#include "tinyalloc.h"

/* USER CODE END Includes */

#define UID_0     (*((uint32_t*)UID_BASE))
#define UID_1     (*((uint32_t*)(UID_BASE + 0x4)))
#define UID_2     (*((uint32_t*)(UID_BASE + 0x14)))

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
enum {
  NON_STOP = 0,
  STOP = 1
};
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/** Force a compilation error if condition is constant and true. */
#ifndef MAYBE_BUILD_BUG_ON
# define MAYBE_BUILD_BUG_ON(cond)   ((void)sizeof(char[1 - 2 * !!(cond)]))
#endif

/** min() macro without type-checking. */
#ifndef min
# define min(a, b)  ((a) < (b) ? (a) : (b))
#endif

/** max() macro without type-checking. */
#ifndef max
# define max(a, b)  ((a) > (b) ? (a) : (b))
#endif

/** roundup() macro without type-checking. */
#ifndef roundup
# define roundup(x, y)  ((((x) + ((y) - 1)) / (y)) * (y))
#endif

/** DIV_ROUND_UP() macro without type-checking. */
#ifndef DIV_ROUND_UP
# define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))
#endif

/** ARRAY_SIZE() macro. */
#ifndef ARRAY_SIZE
# define ARRAY_SIZE(arr)  (sizeof(arr) / sizeof((arr)[0]))
#endif

/** FIELD_SIZEOF() macro. */
#ifndef FIELD_SIZEOF
# define FIELD_SIZEOF(t, f) (sizeof(((t *)0)->f))
#endif
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler( int stop );

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
#define SIM_PWR_Pin GPIO_PIN_2
#define SIM_PWR_GPIO_Port GPIOB



extern RCC_ClocksTypeDef RCC_Clocks;

/** ARRAY_SIZE() macro. */
#ifndef ARRAY_SIZE
# define ARRAY_SIZE(arr)  (sizeof(arr) / sizeof((arr)[0]))
#endif

void enable_nvic_irq(IRQn_Type irq, uint8_t priority);
void disable_nvic_irq(IRQn_Type irq);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
