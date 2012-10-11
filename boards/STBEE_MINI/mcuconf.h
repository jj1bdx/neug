/*
 * HAL driver system settings.
 */
#define STM32_SW                    STM32_SW_PLL
#define STM32_PLLSRC                STM32_PLLSRC_HSE
#define STM32_PLLXTPRE              STM32_PLLXTPRE_DIV1
#define STM32_PLLMUL_VALUE          6
#define STM32_HPRE                  STM32_HPRE_DIV1
#define STM32_PPRE1                 STM32_PPRE1_DIV2
/*
 * Don't know the reason, but ADC doesn't work well with 12MHz clock.
 * So, it is configured as 9MHz
 */
#define STM32_PPRE2                 STM32_PPRE2_DIV2
#define STM32_ADCPRE                STM32_ADCPRE_DIV4
#define STM32_RTCSEL                STM32_RTCSEL_NOCLOCK

#include "mcuconf-common.h"
