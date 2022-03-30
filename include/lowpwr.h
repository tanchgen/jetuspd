/*
 * lowpwr.h
 *
 *  Created on: 1 февр. 2022 г.
 *      Author: jet
 */

#ifndef LOWPWR_H_
#define LOWPWR_H_

#include "diag/trace.h"
#include "times.h"

void rtcSetWut( uint32_t mks );
//--------------------------------------------------------------------------------------
void rtcWakeupCb( void );

// XXX: UNTIL DEVELOPING: WUT wakeup flag
extern volatile FlagStatus wutSlFlag;
// XXX: UNTIL DEVELOPING: WUT wakeup flag
extern volatile FlagStatus sleepFlag;

extern volatile FlagStatus sleepPreFlag;

extern uint32_t ssleep;

//--------------------------------------------------------------------------------------

// Предванительная или окончательная отправка в сон
static inline void toSleep( FlagStatus pre ){
  if( pre ){
    sleepPreFlag = SET;
  }
  else {
    sleepStartFlag = SET;
  }
}

/**
  * @brief  Sleep only, no callbacks will be run
  *
  * @retval none
  */
static inline FlagStatus wutSleep( uint32_t mks ){
  uint32_t wuttime = getRtcTime() + mks/1e6 + 1;

  if( ssleep && (wuttime >= ssleep) ){
    trace_printf("w%u-t%u\n", wuttime, ssleep);
    mDelay( mks / 1000 );
  }
  else {
    rtcSetWut( mks );
    toSleep( RESET );
  }

  return 0;
}


#endif /* LOWPWR_H_ */
