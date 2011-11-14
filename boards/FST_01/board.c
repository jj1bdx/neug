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
    palSetPad (IOPORT1, GPIOA_USB_ENABLE);
  else
    palClearPad (IOPORT1, GPIOA_USB_ENABLE);
}

void
set_led (int value)
{
  if (value)
    palSetPad (IOPORT2, GPIOB_LED);
  else
    palClearPad (IOPORT2, GPIOB_LED);
}
