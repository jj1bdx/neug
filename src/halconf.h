/* HAL configuration file for ChibiOS/RT */

#ifndef _HALCONF_H_
#define _HALCONF_H_

#include "mcuconf.h"

#define HAL_USE_PAL              TRUE
#define HAL_USE_ADC              TRUE
#define HAL_USE_CAN              FALSE
#define HAL_USE_GPT              FALSE
#define HAL_USE_I2C              FALSE
#define HAL_USE_ICU              FALSE
#define HAL_USE_MAC              FALSE
#define HAL_USE_MMC_SPI          FALSE
#define HAL_USE_PWM              FALSE
#define HAL_USE_SDC              FALSE
#define HAL_USE_SERIAL           FALSE
#define HAL_USE_SERIAL_USB       FALSE
#define HAL_USE_SPI              FALSE
#define HAL_USE_UART             FALSE
#define HAL_USE_USB              FALSE

#define ADC_USE_WAIT             FALSE
#define ADC_USE_MUTUAL_EXCLUSION FALSE

#define SERIAL_BUFFERS_SIZE         16



#endif /* _HALCONF_H_ */
