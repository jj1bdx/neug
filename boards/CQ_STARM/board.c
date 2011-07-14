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
  /* CQ STARM has no functionality to stop USB.  */
  /*
   * It seems that users can add the functionality with USB_DC (PD9)
   * though
   */
  (void)NewState;
}

void
set_led (int value)
{
  if (value)
    palSetPad (IOPORT3, GPIOC_LED);
  else
    palClearPad (IOPORT3, GPIOC_LED);
}
