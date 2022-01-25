/*
 * adc.c
 *
 *  Created on: 27 дек. 2018 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#include "stm32l1xx.h"
#include "adc.h"
#include "times.h"

/** Структура дескриптора модуля АЦП: Бат. CMOS (Bat_CMOS, vbat) и Ток нагревателя (Heater_current, hcur) */
sAdcHandle adcHandle;

struct timer_list adcStartTimer;

// Расчет скользящего среднего
inline void movAvgU( uint16_t *avg, uint32_t pt ){
  const uint32_t a = 2000 / (1+ ADC_AVRG_IDX);
  uint32_t tmp = *avg;
  *avg = (uint16_t)((pt * a + (tmp * (1000 - a)) + 500)/1000);
}

// Расчет скользящего среднего
inline void movAvgS( int16_t *avg, int32_t pt ){
  const int32_t a = 2000 / (1+ ADC_AVRG_IDX);
  int32_t tmp = *avg;
  *avg = (int16_t)((pt * a + (tmp * (1000 - a)) + 500)/1000);
}


void adcStartTout( uintptr_t arg ){
  (void)arg;

  adcStart();
}

// ADC sensor init function
void adcInit(void){
// --------- ADC ------------------------------------

  // Вкл тактирование АЦП
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  ADC1->CR2 &= ~ADC_CR2_ADON;

  // Конфигурация тактирования АЦП - HSI (16MHz)
  ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0;

  // Включаем: программный запуск, AWD, AWD-CH0, непрерывный режим, DMA
  ADC1->CR1 = (ADC1->CR1 & ADC_CR1_DISCEN) | (ADC_CR1_PDI | ADC_CR1_PDD | ADC_CR1_SCAN);

  ADC1->CR2 = ADC_CR2_DELS | ADC_CR2_EOCS;// | ADC_CR2_CONT;

  ADC1->SMPR2 = ADC_SMPR2_SMP16_1 | ADC_SMPR2_SMP17_1;

  ADC1->SQR1 = ADC_SQR1_L_0;    // Конвертирование 2-х каналов

  ADC1->SQR5 = (17) | (16<<5);  // Включаем в последовательность измерение каналов TEMP и VREF

  // Прерывание по ушибке переполнения
  ADC1->CR1 |= ADC_CR1_EOCIE | ADC_CR1_OVRIE;
  ADC->CCR |= ADC_CCR_TSVREFE;

  ADC1->CR2 |= ADC_CR2_ADON;
  NVIC_EnableIRQ( ADC1_IRQn );
  NVIC_SetPriority( ADC1_IRQn, 2 );

  timerSetup( &adcStartTimer, adcStartTout, (uintptr_t)NULL );
}

/**
  * @brief  Настройка вывода GPIO для АЦП Батареи BAT-CMOS.
  * @param  None
  *
  * @retval None
  */
inline void adcGpioInit( void ){
  // ADC CH0 - PA0
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

  GPIOA->MODER |= GPIO_MODER_MODER0;      // TM_BCM
}

void adcStart( void ){
  // Вкл тактирование АЦП
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
//  ADC->CCR |= ADC_CCR_VBATEN;

  // Ждем, когда запустится VREFINT
  while( (PWR->CSR & PWR_CSR_VREFINTRDYF) == 0 )
  {}

  adcHandle.adcOk = RESET;
  if( (ADC1->SR & ADC_SR_ADONS) != RESET ){
    ADC1->CR2 |= ADC_CR2_ADON;
  }
  ADC1->CR2 |= ADC_CR2_SWSTART;
}


/**
  * @brief  Остановка преобразований АЦП.
  * @param  None
  *
  * @retval RESET - Остановка прошла успешно
  *         SET - АЦП не остановлен: Работает "Выключение Bat_CMOS" или "Нагреватель"
  */
void adcStop( void ){

  if( ADC1->SR & ADC_SR_ADONS ){
    // ADC запущен - принудительно останавливавем
    ADC1->CR2 &= ~ADC_CR2_ADON;
  }
  while( ADC1->SR & ADC_SR_ADONS )
  {}

  adcHandle.adcOk = RESET;

  return;
}

/**
  * @brief  This function handles ADC interrupt request.
  *         It manages the ADC in case of overrun
  *         the ADC is stopped but not disabled,
  * @param  None
  * @retval None
  */
void ADC1_IRQHandler( void ){
  static FlagStatus seq;

  if ((ADC1->SR & ADC_SR_OVR) != 0) {
    ADC1->SR &= ~ADC_SR_OVR;
    ADC1->CR2 |= ADC_CR2_SWSTART;
  }
  if ((ADC1->SR & ADC_SR_EOC) != 0) {
    adcHandle.adcData[seq] = ADC1->DR;
    seq = !seq;
    if(seq == 0){
      adcHandle.adcOk = SET;
    }
  }
}

/**
  * @brief  Периодически процесс модуля АЦП.
  * @param  None
  *
  * @retval None
  */
void adcProcess( void ){
  int32_t tmpS;

  if( adcHandle.adcOk == RESET ){
    // Новые данные АЦП не готовы
    return;
  }

  adcHandle.adcVbat = (VDD_CALIB * VREF_CAL) / adcHandle.adcData[ADC_VBAT];

  tmpS = ((int32_t)(adcHandle.adcData[ADC_TEMP]) * adcHandle.adcVbat * 100) / VDD_CALIB;
  tmpS -= (int32_t)TEMP30_CAL * 100;
  tmpS *= (int32_t)(110 - 30);
  adcHandle.adcTemp = (int16_t)(((tmpS / (int32_t)(TEMP110_CAL - TEMP30_CAL)) + 3000) / 10);

  adcHandle.adcOk = RESET;

  timerMod( &adcStartTimer, TOUT_1000 );
}


// --------------------------------------------------------------------------
