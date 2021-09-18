#include <stdint.h>

#include "main.h"

void enable_nvic_irq(IRQn_Type irq, uint8_t priority){

	/* Clear pending status. */
	NVIC_ClearPendingIRQ(irq);

  NVIC_SetPriority( irq, priority );
  NVIC_EnableIRQ( irq );
}

void disable_nvic_irq(IRQn_Type irq){

  /* Clear pending status. */
  NVIC_ClearPendingIRQ(irq);

  NVIC_SetPriority( irq, 0 );
  NVIC_DisableIRQ( irq );
}
