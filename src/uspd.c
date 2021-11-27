/*
 * uspd.c
 *
 *  Created on: 24 нояб. 2021 г.
 *      Author: jet
 */

#include "uspd.h"

eUspdCfg uspdCfg = {
  // Режим работы входов датчиков
  .isensType = {
    SENS_TYPE_COUNT,
    SENS_TYPE_COUNT,
    SENS_TYPE_COUNT,
    SENS_TYPE_COUNT,
    SENS_TYPE_RES,
    SENS_TYPE_RES
  },
  .outState = OUT_UPDOWN,                     // UP/DOWN вывода Выхода
  .arxTout = 3600,                            // Период записи данных в архив
  .arxSend = SET,                             // Флаг разрешения отправки архива на сервер
  .arxCalend = { 0, 8, {1, 11, 21, 0, 0} },   // Календарь отправки архива
  .autonamur = RESET,                         // Автоматическое определение уровней срабатывания по сопротивлению
  .simSel = SIM_SEL_AUTO,                     // Режим выбора SIM
  .gprsClass = GPRS_CLASS_12,                 // Класс GPRS
  .gprsConnTout = 120,                        // Макс. длительность соединения GPRS
  .simcfg.pin = -1,                           // PIN-код. -1 - отключено
  .simcfg.pinAuto = RESET,                    // Генерация PIN-кода из MCU_ID
  .simcfg.plmn = 0,                           // Код PLMN сотового ператора
  .simcfg.gprsUser = "",
  .simcfg.gprsPass = "",
  .simcfg.gprsApn = "internet",
  .simcfg.simActiv = RESET,                   // Флаг периодической активации SIM
  .simcfg.simActivTout = 45,                  // Период активации SIM в ДНЯХ
  .simcfg.simActivMax = 1,                    // Макс. кол-во попыток активации SIM
  // Текущие значения
  .simcfg.simActivDay = 0,                    // Количество прошедших дней с последней активации
  .simcfg.simActivCount = 0,                  // Кол-во попыток активации SIM

  .mqttHost = "test.mosquitto.org",
  .port = 1883,
  .mqttUser = "",
  .mqttPass = "",
  .termGate = RESET                           // Прозрачный режим терминала
};


