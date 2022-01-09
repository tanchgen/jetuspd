/*
 * events.h
 *
 *  Created on: 6 янв. 2022 г.
 *      Author: jet
 */

#ifndef EVENTS_H_
#define EVENTS_H_

#include <stdint.h>

typedef union {
  struct {
// --------------------- Пользовыательские события ----------------------------
    uint32_t isens5: 1;
    uint32_t isens6: 1;      // срабатывание датчика протечки
    uint32_t sb1: 1;         // нажатие кнопки sb1
    uint32_t sb2: 1;         // открытие крышки прибора / отрыв от стены
    uint32_t consump: 1;      // Аномальный расход
    uint32_t namur1: 1;      // разрыв линии NAMUR SENS_1
    uint32_t namur2: 1;      // разрыв линии NAMUR SENS_1
    uint32_t namur3: 1;      // разрыв линии NAMUR SENS_1
    uint32_t namur4: 1;      // разрыв линии NAMUR SENS_1
    uint32_t pulse1: 1;      // превышение частоты счета импульсов SENS_1
    uint32_t pulse2: 1;      // превышение частоты счета импульсов SENS_1
    uint32_t pulse3: 1;      // превышение частоты счета импульсов SENS_1
    uint32_t pulse4: 1;      // превышение частоты счета импульсов SENS_1
// --------------------- Технологические события ----------------------------
    uint32_t iwdg: 1;         // срабатывание watchdog
    uint32_t uspdOn: 1;      // включение устройства
    uint32_t swrst: 1;     // программная перезагрузка устройства
    uint32_t fwUpd: 1;      // обновление прошивки
    uint32_t cfgLoad: 1;     // загрузка конфигурации
    uint32_t gsmFault: 1;    // ошибка установки соединения GSM
    uint32_t gprsFault: 1;    // ошибка установки соединения GPRS
    uint32_t tcpFault: 1;    // ошибка установки соединения TCP
    uint32_t ntpFault: 1;    // ошибка синхронизации времени
    uint32_t mqttFault: 1;   // ошибка подключения к серверу
    uint32_t mqttClose: 1;   // MQTT обрыв соединения
// ----------------------------------------------------------------------------
    uint32_t logSize: 1;   // Размер журнала
  };
  uint32_t u32evnt;
} uEventFlag;

extern uEventFlag evntFlags;

#endif /* EVENTS_H_ */

