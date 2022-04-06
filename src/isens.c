/*
 * isens.c
 *
 *  Created on: 30 авг. 2021 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */
#include <string.h>
#include <ctype.h>

#include "times.h"
#include "logger.h"
#include "flash.h"
#include "main.h"
#include "uspd.h"
#include "events.h"
#include "isens.h"

// -------------- ДЛЯ ТЕСТА ----------------------------------
#define ISENS_ARCH_TOUT        3600  // 3000 мс
#define ARCH_READ_TOUT         TOUT_1000 // Каждую секунду

extern eGsmRunPhase gsmRunPhase;

struct timer_list tIsArchTimer;
struct timer_list tArchPubTimer;

void isensDbTout( uintptr_t arg );
HAL_StatusTypeDef   stmEeRead( uint32_t addr, uint32_t * data, uint32_t datalen);
HAL_StatusTypeDef   stmEeWrite( uint32_t addr, uint32_t * data, uint32_t datalen);


sISens iSens[ISENS_NUM] = {
  {
    .pinIn = {GPIOA, GPIO_PIN_0, 0, 0},
    .isensCount = 11,
    .state = ISENS_DOWN,
  },
  {
    .pinIn = {GPIOA, GPIO_PIN_1, 0, 1},
    .isensCount = 22,
    .state = ISENS_DOWN,
  },
  {
    .pinIn = {GPIOA, GPIO_PIN_2, 0, 2},
    .isensCount = 33,
    .state = ISENS_DOWN,
  },
  {
    .pinIn = {GPIOA, GPIO_PIN_3, 0, 3},
    .isensCount = 44,
    .state = ISENS_DOWN,
  },
//  {
//    .pinIn = {GPIOB, GPIO_PIN_0, GPIO_MODE_IT_RISING_FALLING, GPIO_PULLDOWN, GPIO_SPEED_FREQ_LOW, AF0, Bit_SET, Bit_SET, RESET },
//    .isensCount = 0,
//    .state = ISENS_DOWN,
//  },
//  {
//    .pinIn = {GPIOB, GPIO_PIN_1, GPIO_MODE_IT_RISING_FALLING, GPIO_PULLDOWN, GPIO_SPEED_FREQ_LOW, AF0, Bit_SET, Bit_SET, RESET },
//    .isensCount = 0,
//    .state = ISENS_DOWN,
//  },
};

// ======================= Клендарь отправки архива сенсоров ======================================
char * defCal = "45 0-21 6,11 * *";

tRtc oldCalAlrm = {0};

const struct limit {
  uint8_t begLim;
  uint8_t endLim;
} lim[TS_NUM] = {
    {0,59},
    {0,59},
    {1,31}
};

eTimeSect tsect;
// ================================================================================================

// XXX: !!! FOR TEST SENSOR SEND !!!
/*
void sensSendTout( uintptr_t arg ){
  (void)arg;

  if( (uspd.readArchEvntQuery == RESET)
      && (uspd.readArchSensQuery == RESET) ){
    trace_puts( "SENS send");
    SIM800.mqttClient.pubFlags.archPubEnd = SET;
  }
  else {
    timerMod( &sensSendTimer, 200 );
  }
}
*/

void isArchTout( uintptr_t arg ){
  uint32_t tout = *((uint32_t *)arg);
  uspd.archWrFlag = SET;
  uspd.runMode = RUN_MODE_SENS_WRITE;
  rtcTimMod( &tIsArchTimer, tout );
}

// =================== Функции календаря =========================

// -------------------- Archive Calendar --------------------------------------------
int calPars( struct list_head * head, char ** pstr ){
  sCal * pxc;
  uint8_t * pnum;
  char * str = *pstr;
  struct list_head * lst = head;

  while(1){
    if( list_is_last(lst, head) ){
      // Дальше елементов нет
      if( (pxc = malloc(sizeof(sCal))) == NULL ){
        perror("malloc:");
        exit(EXIT_FAILURE);
      }
      else {
        // Сохраняем в списке
        list_add( &(pxc->node), lst);
      }
    }
    else {
      pxc = list_first_entry( lst, sCal, node );
    }

    pnum = &(pxc->beg);
    for( eField f = F_BEGIN; (f < F_NUM) && (*str != '\0'); f++ ){
      if( *str == '*' ){
        pxc->beg = lim[tsect].begLim;
        pxc->end = lim[tsect].endLim;
        pxc->step = 1;
        while( !isspace((int)*str) ){
          str++;
        }
        break;
      }

      *pnum = strtoul( str, pstr, 10 );
      str = *pstr;
      if( str == NULL ){
        perror("String format!");
        return -1;
      }
      else if( *str == '-') {
        str++;
        pnum = &(pxc->end);
      }
      else if( *str == '/' ){
        pnum = &(pxc->step);
      }
      else {
        // Строка закончилась
        if( f == F_BEGIN){
          // "end" не обрабатывался
          pxc->end = pxc->beg;
          // "step" не обрабатывался
          pxc->step = 1;
        }
        else if( f == F_END ){
          // "step" не обрабатывался
          pxc->step = 1;
        }
        while( isspace((int)*str) ){
          str++;
        }
        if(*str != ','){
          goto end_field;
        }
        lst = lst->next;
        str++;
        break;
      }
    }
  }

end_field:
  lst = lst->next->next;
  while( lst != head ){
    struct list_head * lst2 = lst;
    lst = lst2->next;
    pxc = list_entry( lst2, sCal, node );
    list_del( lst2 );
    free( pxc );
  }

  *pstr = str;
  return 0;
}


void nextMon( sCalend * cal, tRtc * cltime, tRtc * alr, eCmp cmp ){
  sCal * cl;

  cl = list_entry( cal->hQ.next, sCal, node );
  alr->hour = cl->beg;
  cl = list_entry( cal->mQ.next, sCal, node );
  alr->min = cl->beg;
  alr->sec = 2;
  if( cmp == SMALLER ){
    // Все переносится на следующий месяц
    cl = list_entry( cal->dQ.next, sCal, node );
    alr->date = cl->beg;
    if(cltime->month == 11){
      // Сейчас декабрь -> Следующий год, январь
      alr->year = cltime->year + 1;
      alr->month = 0;
    }
    else {
      // Следующий месяц
      alr->year = cltime->year;
      alr->month = cltime->month + 1;
    }
  }
  else if( cmp == GREATER ){
    alr->month = cltime->month;
  }
}

eCmp searchAlrm( sCalend * cal, tRtc * alr, tRtc * old ){
  tRtc cltime;
  struct list_head *dcurr, *dhead;
  struct list_head *hcurr, *hhead;
  struct list_head *mcurr, *mnext;
  sCal * cl;
  eCmp cmp;
  eTimeSect curr = TS_DAY;

  dhead = &(cal->dQ);
  hhead = &(cal->hQ);
  hcurr = hhead->next;

  getRtcTime();
  cltime = rtc;

  // -------------------------- Поиск -----------------------------------
  for( cmp = SMALLER; cmp != GREATER; ){
    // Day
    if( curr == TS_DAY){
      hhead = &(cal->hQ);

      for (dcurr = dhead->next; dcurr != &(cal->dQ); dcurr = dcurr->next){
        cl = list_entry(dcurr, sCal, node);

        alr->date = cl->beg;
        for( uint8_t d = cl->beg; d <= cl->end; d += cl->step ){
          if( d > cltime.date ){
            cmp = GREATER;
            alr->date = d;
            break;
          }
          else if( d == cltime.date ){
            cmp = EQUAL;
            alr->date = d;
            break;
          }
        }
        if(cmp != SMALLER){
          // Сегодня или в будущем - выходим
          hhead = &(cal->hQ);
          curr = TS_HOUR;
          break;
        }
      }

      if( cmp != EQUAL ){
        // День не совпадает: минимальные час, минута и секунда
        nextMon( cal, &cltime, alr, cmp );
        return GREATER;
      }
    }

    if( curr == TS_HOUR){
      // День - сегодняшний
      //Hour
      cmp = SMALLER;
      for (hcurr = hhead->next; hcurr != &(cal->hQ); hcurr = hcurr->next){
        cl = list_entry(hcurr, sCal, node);

        alr->hour = cl->beg;
        for( uint8_t h = cl->beg; h <= cl->end; h += cl->step ){
          if( h > cltime.hour ){
            cmp = GREATER;
            alr->hour = h;
            break;
          }
          else if( h == cltime.hour ){
            cmp = EQUAL;
            alr->hour = h;
            break;
          }
        }
        if(cmp != SMALLER){
          // На этот час или в будущем - выходим
          curr = TS_MIN;
          break;
        }
      }
      if(cmp == SMALLER ){
        // На сегодня уже все  выполнено. Возвращаемся - ищем следующие дни
        cltime.date += 1;
        cltime.hour = 0;
        dhead = dcurr;
        curr = TS_DAY;
        continue;
      }
      else if( cmp == GREATER ){
        // Сегодня в следующие часы: минимальные минута и секунда
        cl = list_entry( cal->mQ.next, sCal, node );
        alr->min = cl->beg;
        alr->sec = 2;
        // Нашли
        return GREATER;
      }
    }

    if( curr == TS_MIN){
      // Час - текущий
      //Minute
//      cmp = SMALLER;
      list_for_each_safe( mcurr, mnext, &(cal->mQ) ) {
        cl = list_entry(mcurr, sCal, node);

        alr->min = cl->beg;
        for( uint8_t m = cl->beg; m <= cl->end; m += cl->step ){
          if( m > cltime.min ){
            cmp = GREATER;
            alr->min = m;
            alr->sec = 2;
            // Нашли
            return GREATER;
          }
          else if( (m == cltime.min) && (m > old->min) ){
            cmp = GREATER;
            alr->min = m;
            // На всякий случай на 2 сек вперед
            alr->sec = cltime.sec + 2;
            // Нашли
            return GREATER;
          }
        }
      }
//      if(cmp == SMALLER ){

      // На этот час ничего не нашли. Возвращаемся - ищем следующие часы
      cltime.hour += 1;
      cltime.min = 0;
      hhead = hcurr->prev;
      curr = TS_HOUR;
      continue;
//    }
    }

    // Совпадают месяц, час, минута календаря и текущие;
    if( cmp == SMALLER ) {
      nextMon( cal, &cltime, alr, cmp );
      return GREATER;
    }
  }

  return cmp;
}

// ----------------------------------------------------------------------------------
void sensPubAlrmSet( sCalend * cal ){
  tRtc tmpalrm;
  eCmp cmp;
  tUxTime ut;

  assert_param( !list_empty(&(cal->mQ)) );
  rtcGetDate( &tmpalrm );
  cmp = searchAlrm( cal, &tmpalrm, &oldCalAlrm );
  assert_param( cmp == GREATER );
  ut = xTm2Utime( &tmpalrm );
  trace_printf( "a0:%d\n", ut );
  ut -= getRtcTime();
  rtcTimMod( &tArchPubTimer, ut );
  // Делаем установленный будильник "старым"
  oldCalAlrm = tmpalrm;
}


// Выполняется по календарю, когда надо передавать данные из архива
void calWkupFunc( uintptr_t arg ){
  (void)arg;
  mTick = 0;
  // Предварительное разрешение передачи записей архива
  uspd.runMode = RUN_MODE_SENS_SEND;
}


char * fillField( char * str, struct list_head * xqu ){
  struct list_head * curr;
  sCal * cl;

  // Minute string
  list_for_each( curr, xqu ){
    cl = list_entry(curr, sCal, node);
    utoa( cl->beg, str, 10 );
    while( *str != '\0' ){
      str++;
    }
    if( cl->beg != cl->end ){
      *str++ = '-';
      utoa( cl->end, str, 10 );
      while( *str != '\0' ){
        str++;
      }
    }
    if( cl->step != 1 ){
      *str++ = '/';
      utoa( cl->step, str, 10 );
      while( *str != '\0' ){
        str++;
      }
    }
    // ',' - между элементами списка
    *str++ = ',';
  }
  if( *(str-1) == ','){
    *(str-1) = ' ';
  }

  return str;
}


void calStrCreate( char str[], sCalend * cal ){
  char * beg = str;

  // Minute string
  if( list_empty(&(cal->mQ))
      || list_empty(&(cal->hQ))
      || list_empty(&(cal->dQ)))
  {
    str[0] = '\0';
  }
  {
    str = fillField( str, &(cal->mQ) );
    str = fillField( str, &(cal->hQ) );
    str = fillField( str, &(cal->dQ) );

    strcpy( str, "* *" );

    assert_param( strlen(beg) < 21 );
    (void)beg;
  }
}

// ===============================================================


void isensProcess( void ){

  if( uspd.runMode == RUN_MODE_SENS_WRITE ){
    // Работаем только если надо записать датчики во флеш
    if( uspd.archWrFlag ){
      // Настало время записи датчиков в Архив
      uint32_t isdata[ISENS_NUM];
      sLogRec logrec = {0};

      for( eIsens s = 0; s < ISENS_NUM; s++ ){
        isdata[s] = iSens[s].isensCount;
      }

      assert_param( ISENS_NUM <= 4 );

      // Запись в EEPROM состояние датчиков
      stmEeWrite( USPD_SENS_ADDR, isdata, (ISENS_NUM * 4) );

      if( logger( &logrec, getRtcTime(), DEVID_ISENS_1, isdata, ISENS_NUM ) == 1){
        // Записано в Архив успешно
        uspd.archWrFlag = RESET;
      }
      else {
        ErrHandler( NON_STOP );
      }
    }
    else if( flashDev.state == FLASH_READY ){
      // Запись датчиков окончена
      gsmReset = SIM_RESET;
      gsmRun = RESET;
      gsmRunPhase = PHASE_OFF_OK;
      toSleep( RESET );
    }
  }
}


void isensIrqHandler( eIsens is ){
  uint32_t t0;

  // Обновим
  uxTime = getRtcTime();

  // Разница времени при сработывании датчика
  t0 = ((uxTime - iSens[is].tstime) % 1000) * 1000;
  // Вычисляем доли секунды
  t0 += ((int32_t)rtc.ss - iSens[is].tsss) * 1000 / 0x129;

  if( t0 > 50 ){
    // Нормальное срабатывание - счетчик импульсов
    // Состояние сохранилось в "1" - НЕ ложное срабатывание
    iSens[is].isensCount++;
    // Метка времени
    iSens[is].tstime = uxTime;
    iSens[is].tsss = rtc.ss;
  }
  else if( t0 > 10 ){
    // Не дребезг, но слишком частое срабатывание - отмечаем событие
    uEventFlag evnt;

    evnt.u32evnt = 0;
    evnt.pulse1 = SET;
    evntFlags.u32evnt |= evnt.u32evnt << is;
  }
  else {
    // Бребезг - игнорируем
  }
}


// Обработка сигналов с датчиков Холла
void ISENS_IRQHandler( void ){
  eIsens is;
//  uint32_t tm;
//  uint32_t dtime;

//  tm = getRtcTime();

  if(ISENS_TIM->SR & TIM_SR_CC1IF){
    // Сработал Датчик 1
    is = ISENS_1;
    ISENS_TIM->SR &= ~TIM_SR_CC1IF;
  }
  else if(ISENS_TIM->SR & TIM_SR_CC2IF){
    // Сработал Датчик 2
    is = ISENS_2;
    ISENS_TIM->SR &= ~TIM_SR_CC2IF;
  }
  else if(ISENS_TIM->SR & TIM_SR_CC3IF){
    // Сработал Датчик 3
    is = ISENS_3;
    ISENS_TIM->SR &= ~TIM_SR_CC3IF;
  }
  else if(ISENS_TIM->SR & TIM_SR_CC4IF){
    // Сработал Датчик 4
    is = ISENS_4;
    ISENS_TIM->SR &= ~TIM_SR_CC4IF;
  }
  else {
//    while(1)
//    {}
    ISENS_TIM->SR = 0;
    return;
  }

  isensIrqHandler( is );
//  dtime = (tm - iSens[is].tstime) * 1000 + rtc.ss;
//  dtime -= iSens[is].tsss;
//  if( dtime < 12 ){
//    // Слишком короткий период - выставляем флаг события
//
//    uEventFlag evnt;
//
//    evnt.u32evnt = 0;
//    evnt.pulse1 = SET;
//    evntFlags.u32evnt |= evnt.u32evnt << is;
//  }
//  else {
//    timerStack( &(iSens[is].dbTimer), ISENS_DB_TOUT, TIMER_MOD );
//  }
}

// Переустановка на новое время при переводе часов
void isensTimCorr( void ){
  // Переустанавливаем RTC-таймеры
  sensPubAlrmSet( &(uspd.arxCal) );
  rtcTimMod( &tIsArchTimer, uspdCfg.arxTout );
}


/**
  * @brief  Обработчик тайм-аута антидребезга выводов GPIO.
  *
  * @param[in]  arg данные таймера (дескриптор таймера)
  *
  * @retval none
  */
void isensDbTout( uintptr_t arg ){
  sISens * sens = (sISens *)arg;

  sens->debounceTout = 0;
  // Нынешнее состояния пина
  if( ((sens->pinIn.gpio)->IDR & (sens->pinIn.pin)) ){
    // Состояние сохранилось в "1" - НЕ ложное срабатывание
    sens->isensCount++;
    // Метка времени
    sens->tstime = getRtcTime();
    sens->tsss = rtc.ss;
  }

  return;
}


void isensPinInit( sISens * sens ){
  GPIO_TypeDef * gpio;
  uint8_t pinNum;

  assert_param( sens->pinIn.gpio != NULL );
  // ------------  Вывод Геркона -----------------------
  gpio = sens->pinIn.gpio;
  pinNum = sens->pinIn.pinNum;

  gpio->MODER = (gpio->MODER & ~(0x3 << (pinNum * 2) )) | (0x2 << (pinNum * 2) );
  gpio->PUPDR = (gpio->PUPDR & ~(0x3 << (pinNum * 2) )); // | (0x2 << (pinNum * 2) );
  // Alternate function TIM1_CH1-4 = 6
  gpio->AFR[pinNum / 8] = (gpio->AFR[pinNum / 8] & ~(0xF << ((pinNum % 8) * 4))) | (AF1 << ((pinNum % 8) * 4));
}


void isensTimInit( void ){
  // TIM Init
  uint16_t psc;

  ISENS_TIM_CLK_EN;

  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM2_STOP;

  // Расчет предделителя
  psc = (rccClocks.PCLK1_Frequency/ISENS_TIM_FREQ) - 1;

  ISENS_TIM->PSC = psc;

  ISENS_TIM->CCER = 0;

  ISENS_TIM->ARR = -1;
  ISENS_TIM->CCMR1 = (ISENS_TIM->CCMR1 & ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S))
                      | (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);
  ISENS_TIM->CCMR2 = (ISENS_TIM->CCMR2 & ~(TIM_CCMR2_CC3S | TIM_CCMR2_CC4S))
                      | (TIM_CCMR2_CC3S_0 | TIM_CCMR2_CC4S_0);
  ISENS_TIM->CCER |= TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P | TIM_CCER_CC4P;  // Растущий фронт

  ISENS_TIM->CR1 |= TIM_CR1_URS;
  ISENS_TIM->DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE;

  NVIC_EnableIRQ( ISENS_TIM_IRQn );
  NVIC_SetPriority( ISENS_TIM_IRQn, 0);
}

void isensInit( void ){
  // Вывод
  gpioPinSetup( &gpioPinSensOn );

  for( uint8_t i = 0; i < ISENS_NUM; i++ ){
    isensPinInit( &(iSens[i]) );
    timerSetup( &(iSens[i].dbTimer), isensDbTout, (uintptr_t)&(iSens[i]) );
  }
  // Инициализация таймера, на который подключены ISENS
  isensTimInit();

  // Инициализация списка календаря
  INIT_LIST_HEAD( &(uspd.arxCal.mQ) );
  INIT_LIST_HEAD( &(uspd.arxCal.hQ) );
  INIT_LIST_HEAD( &(uspd.arxCal.dQ) );

  gpioPinResetNow( &gpioPinSensOn );

  rtcTimSetup( &tArchPubTimer, calWkupFunc, (uintptr_t)&(uspd.arxCal) );
  rtcTimSetup( &tIsArchTimer, isArchTout, (uintptr_t)&(uspdCfg.arxTout) );
}


void isensEnable( void ){

  assert_param( ISENS_NUM <= 4 );

  tUxTime tm;
  // Ввод

  // Выбор канала
  ISENS_TIM->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);
  ISENS_TIM->CR1 |= TIM_CR1_CEN;

  tm = getRtcTime();
  for( uint8_t i = 0; i < ISENS_NUM; i++ ){
    // Метка времени
    iSens[i].tstime = tm;
    iSens[i].tsss = rtc.ss;
    // Чтение в EEPROM состояние датчиков
    stmEeRead( USPD_SENS_ADDR + (i * FIELD_SIZEOF(sISens, isensCount)),
               &(iSens[i].isensCount),
               FIELD_SIZEOF(sISens, isensCount)
             );
    if( iSens[i].isensCount == ~0UL ){
      iSens[i].isensCount = 0;
    }
    if( iSens[i].isensCount == 0 ){
      iSens[i].isensCount = (i + 1) * 11;
    }
  }

  // XXX: !!! FOR TEST ONLY !!!
  uspdCfg.arxTout = 360;

  rtcTimMod( &tIsArchTimer, uspdCfg.arxTout );
}


/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  (void)GPIO_Pin;
  while(1)
  {}
}

