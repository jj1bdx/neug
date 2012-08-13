#include "config.h"
#include "ch.h"
#include "hal.h"

/*
 * Board-specific initialization code.
 */
void boardInit(void)
{
  /*
   * Disable JTAG and SWD, done after hwinit1_common as HAL resets AFIO
   */
  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_DISABLE;
  /* We use LED2 as optional "error" indicator */
  palSetPad (IOPORT1, GPIOA_LED2);
}
