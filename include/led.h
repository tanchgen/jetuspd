/*
 * led.h
 *
 *  Created on: 17 янв. 2019 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#ifndef LED_H_
#define LED_H_

#include <gpio.h>
#include "times.h"

// -------------- Светодиоды --------------------
#define LED_BLINK_ON_TOUT     63
#define LED_BLINK_OFF_TOUT    937
#define LED_TOGGLE_TOUT       TOUT_200
#define LED_FAST_TOGGLE_TOUT  TOUT_100
#define LED_SLOW_TOGGLE_TOUT  TOUT_500

/** @defgroup Перечислитель светодиодов
  * @{
  */
typedef enum{
  LED_G = 0,
  LED_R,
  LED_NUM
} eLed;

typedef struct {
  sGpioPin ledPin;
  uint16_t ledOnTout;
  uint16_t ledOffTout;
  uint16_t baseBigTout;
  uint8_t ledToggleCount;
  FlagStatus ledToggleFlag;
  uint32_t ledBigTout;
  uint8_t baseToggleCount;
  uint32_t timTout;
} sLed;

extern sLed ledHandle[];

/**
  * @brief  Инициализация выводов и таймеров светодиодов.
  *
  * @param none
  *
  * @retval none
  */
void ledInit( void );

/**
  * @brief  Включает светодиод на определенное время
  *
  * @param[in]  led номер светодиода @ref enum eLed
  * @param[in]  endure длительность свечения
  *
  * @retval none
  */
void ledOn( eLed led, uint32_t endure);

/**
  * @brief  Выключает светодиод.
  *
  * @param[in]  arg данные таймера (номер вывода GPIO MCU_LED_N)
  *
  * @retval none
  */
void ledOff( eLed led, uint32_t endure );

/**
  * @brief  Проверка выключенр ли светодиод
  *
  * @param[in]  led номер светодиода @ref enum eLed
  *
  * @retval FlagStatus
  */
FlagStatus ledIsOff( eLed led );

/**
  * @brief  Переключает светодиод на определенное время
  *
  * @param[in]  led номер светодиода @ref enum eLed
  * @param[in]  endure длительность свечения
  *
  * @retval none
  */
void ledToggle( eLed led );

/**
  * @brief  Запускает мигание светодиодом
  *
  * @param[in]  led номер светодиода @ref enum eLed
  * @param[in]  onendure длительность свечения
  * @param[in]  offendure длительность НЕ свечения
  * @param[in]  count количество циклов мигания
  *
  * @retval none
  */
void ledToggleSet( eLed led, uint16_t onendure, uint16_t offendure, uint8_t count, uint16_t globeendure );


/**
  * @brief  Перемигивать светодиоды
  *
  * @param[in]  endure длительность свечения и несвечения
  *
  * @retval none
  */
void ledWink( uint32_t endure);

void ledAllOff( void );

/**
  * @brief  Включение светодиодной индикации "MCU работает штатно" (красный выключен, зеленый вспыхивает 1 раз в с)
  *
  * @param[in]  endure длительность свечения и несвечения
  *
  * @retval none
  */
void ledSysSndby( void );

/**
  * @brief  Включение светодиодной индикации "MCU работает штатно" (красный выключен, зеленый вспыхивает 1 раз в с)
  *
  * @param[in]  endure длительность свечения и несвечения
  *
  * @retval none
  */
void ledSysOff( void );


#endif /* LED_H_ */
