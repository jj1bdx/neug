#include "config.h"
#include "ch.h"
#include "hal.h"

/*
 * Board-specific initialization code.
 */
void boardInit(void)
{
}

void
USB_Cable_Config (int NewState)
{
  /* No functionality to stop USB.  */
  (void)NewState;
}

void
set_led (int value)
{
  if (value)
    palSetPad (IOPORT1, GPIOA_LED);
  else
    palClearPad (IOPORT1, GPIOA_LED);
}
