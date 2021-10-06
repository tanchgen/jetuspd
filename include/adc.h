/*
 * adc.h
 *
 *  Created on: 26 дек. 2018 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#ifndef ADC_H_
#define ADC_H_

#define ADC_CH_NUM      2

#define ADC_AVRG_IDX    3

/* Temperature sensor calibration value address */
#define VREF_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FF80078))
#define TEMP110_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FF8007E))
#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FF8007A))

#define VREF_CAL  (*VREF_CAL_ADDR)
#define TEMP110_CAL (*TEMP110_CAL_ADDR)
#define TEMP30_CAL  (*TEMP30_CAL_ADDR)

#define VDD_CALIB ((uint16_t)(3000))

//  Коэффициеньы пересчета

/** Признак отсутствия готовности данных по TM_ADC. */
#define ADC_TM_DATA_NOT_READY_STATUS_BIT    (1U << 1)
/** Признак отсутствия готовности данных по TM_ADC. */
#define ADC_TM_PEAK_DATA_NOT_READY_STATUS_BIT    (1U << 2)
/** Признак аварии по достижении порогового значения TM_ADC_LOW_ALARM_LIMIT. */
#define ADC_TM_LOW_ALARM_LIMIT_STATUS_BIT  (1U << 3)
/** Признак аварии по достижении порогового значения TM_ADC_HIGH_ALARM_LIMIT. */
#define ADC_TM_HIGH_ALARM_LIMIT_STATUS_BIT  (1U << 4)

/** Маска признаков аварий ADC. */
#define ADC_ALARM_STATUS_MASK  (0xFFF8)

#define ADC_NOT_READY_STATUS_MASK  (0x0006)

//  ADC_TM_STATUS,      /** <Биты состояния 1: */
//  ADC_TM,             /** <Мгновенное значение температуры VTM1. */
//  ADC_PEAK_MIN_TM,    /** <Минимальное измеренное значение температуры TM1_VTM1. */
//  ADC_PEAK_MAX_TM,    /** <Максимальное измеренное значение температуры TM1_VTM1. */
//  ADC_TM_LOW_ALARM_LIMIT,     /** <Аварийное нижнее пороговое значение температуры VTM1. */
//  ADC_TM_HIGH_ALARM_LIMIT,    /** <Аварийное верхнее пороговое значение температуры VTM1. */

enum {
  ADC_VBAT,
  ADC_TEMP,
};

typedef struct {
  int16_t adcTemp;       /** < Значение АЦП - TM */
  uint16_t adcVbat;
  uint16_t adcData[2];    /* Структура данных АЦП */

  FlagStatus adcOk;   /** < Флаг признака готовности данных вообще */
} sAdcHandle;

extern sAdcHandle adcHandle;

void adcInit( void );
void adcStart( void );
void adcStop( void );
void adcProcess( void );

#endif /* ADC_H_ */
