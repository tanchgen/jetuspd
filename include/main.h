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

//#include "tinyalloc.h"
#include "times.h"
#include "gpio.h"
#include "led.h"

/* USER CODE END Includes */

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

#ifndef min
#define min(a,b)      ((a<b)? a : b)
#endif //min
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
