/*
 * lowpwr.h
 *
 *  Created on: 1 февр. 2022 г.
 *      Author: jet
 */

#ifndef LOWPWR_H_
#define LOWPWR_H_

#include "times.h"

void rtcSetWut( uint32_t mks );
//--------------------------------------------------------------------------------------
void rtcWakeupCb( void );

// XXX: UNTIL DEVELOPING: WUT wakeup flag
extern volatile FlagStatus wutSlFlag;
// XXX: UNTIL DEVELOPING: WUT wakeup flag
extern volatile FlagStatus sleepFlag;

//--------------------------------------------------------------------------------------
void wutTimeCheck( uint32_t mks );

static inline void toSleep( void ){
  sleepStartFlag = SET;
}

/**
  * @brief  Sleep only, no callbacks will be run
  *
  * @retval none
  */
static inline void wutSleep( uint32_t mks ){
  wutTimeCheck( mks );
  rtcSetWut( mks );
  toSleep();
}


#endif /* LOWPWR_H_ */
