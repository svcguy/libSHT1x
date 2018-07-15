#ifndef DWT_STM32_DELAY_H
#define DWT_STM32_DELAY_H

#include <stdint.h>

// Using the HAL Version macro to determine the core family
#ifdef __STM32F0xx_HAL_VERSION
#include "stm32f1xx_hal.h"
#endif /* __STM32F0xx_HAL_VERSION */
#ifdef __STM32F1xx_HAL_VERSION
#include "stm32f1xx_hal.h"
#endif /* __STM32F1xx_HAL_VERSION */
#ifdef __STM32F4xx_HAL_VERSION
#include "stm32f4xx_hal.h"
#endif /* __STM32F4xx_HAL_VERSION */
#ifdef __STM32F7xx_HAL_VERSION
#include "stm32f7xx_hal.h"
#endif /* __STM32F7xx_HAL_VERSION */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief Initialize DWT_Cycle_Count for DWT_Delay_us function
 * @return Error DWT Counter
 *          1: DWT counter Error
 *          0: DWT counter Success
 */
uint32_t DWT_Delay_Init(void);

/**
 * @brief This function provides a delay in microseconds
 * @param microseconds: Delay in us
 */
__STATIC_INLINE void DWT_Delay_us( volatile uint32_t microseconds )
{
    uint32_t clk_cycle_start = DWT->CYCCNT;

    // Go to number of cycles for system
    microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);

    // Delay till end
    while( ( DWT->CYCCNT - clk_cycle_start ) < microseconds );
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* DWT_STM32_DELAY_H */