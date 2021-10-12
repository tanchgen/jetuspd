/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "sys/cdefs.h"
#include "stm32l1xx.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

//#define FLASH_SIZE              (FLASH_END - FLASH_BASE + 1)
#define BOOTLOADER_SIZE         ((uint32_t)0x2000)
#define FW_1_START_ADDR         (FLASH_BASE + BOOTLOADER_SIZE)
#define FW_1_SIZE               ((FLASH_SIZE - BOOTLOADER_SIZE) / 2)
#define FW_1_END                (FW_1_START_ADDR + FW_1_SIZE - 1)
#define FW_1_VER_ADDR           (FW_1_START_ADDR + FW_1_SIZE - 4)
#define FW_2_START_ADDR         (FW_1_END + 1)
#define FW_2_SIZE               FW_1_SIZE
#define FW_2_END                (FW_2_START_ADDR + FW_2_SIZE - 1)
#define FW_2_VER_ADDR           (FW_2_START_ADDR + FW_2_SIZE - 4)


#define FW_HANDLE_ADDR           (EEPROM_BASE + 0x80)


/** Force a compilation error if condition is constant and true. */
#ifndef MAYBE_BUILD_BUG_ON
# define MAYBE_BUILD_BUG_ON(cond)   ((void)sizeof(char[1 - 2 * !!(cond)]))
#endif

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
struct fwversion {
  uint8_t major;
  uint8_t minor;
  uint16_t release;
} __attribute__((__packed__));

typedef union {
  struct fwversion fwVer;
  uint32_t u32fwver;
} uFwVer;

struct fwrunstate {
  uint8_t fw1Count;
  uint8_t fw2Count;
  FlagStatus fw1Bug;
  FlagStatus fw2Bug;
};

typedef union {
  struct fwrunstate fwstate;
  uint32_t u32fwst;
} uFwRunState;

//struct fwhandle {
//  uFwVer fw1Ver;
//  uFwVer fw2Ver;
//  sFwRunState fwRunState;
//};

/* Function pointer for jumping to user application. */
typedef void (*fnc_ptr)(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
