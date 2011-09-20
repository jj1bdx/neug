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
    palClearPad (IOPORT4, GPIOD_USB_DISC);
  else
    palSetPad (IOPORT4, GPIOD_USB_DISC);
}

void
set_led (int value)
{
  if (value)
    palClearPad (IOPORT4, GPIOD_LED1);
  else
    palSetPad (IOPORT4, GPIOD_LED1);
}
