#include "config.h"
#include "ch.h"
#include "hal.h"

/*
 * Board-specific initialization code.
 */
void boardInit(void)
{
#if !defined(DFU_SUPPORT)
  if (palReadPad (IOPORT3, GPIOC_BUTTON) == 0)
    /*
     * Since LEDs are connected to JTMS/SWDIO and JTDI pin,
     * we can't use LED to let know users in this state.
     */
    for (;;);		       /* Wait for JTAG debugger connection */
#endif

  /*
   * Disable JTAG and SWD, done after hwinit1_common as HAL resets AFIO
   */
  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_DISABLE;
  /* We use LED2 as optional "error" indicator */
  palSetPad (IOPORT1, GPIOA_LED2);
}
