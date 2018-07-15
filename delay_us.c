#include "delay_us.h"

/**
 * @brief Initialize DWT_Cycle_Count for DWT_Delay_us function
 * @return Error DWT Counter
 *          1: DWT counter Error
 *          0: DWT counter Success
 */
uint32_t DWT_Delay_Init(void)
{
    // Disable TRC
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
    // Enable TRC
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // Disable clock cycle counter
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
    // Enable clock cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Reset the clock cycle counter value
    DWT->CYCCNT = 0;

    // 3 NOP
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");

    // Check if clock cycle counter has started
    if(DWT->CYCCNT)
    {   // Clock cycle count started
        return 0;
    }
    else
    {   // Clock cycle not started
        return 1;
    }
}