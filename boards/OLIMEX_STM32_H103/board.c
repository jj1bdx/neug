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
  if (NewState != DISABLE)
    palClearPad (IOPORT3, GPIOC_DISC);
  else
    palSetPad (IOPORT3, GPIOC_DISC);
}

void
set_led (int value)
{
  if (value)
    palClearPad (IOPORT3, GPIOC_LED);
  else
    palSetPad (IOPORT3, GPIOC_LED);
}
