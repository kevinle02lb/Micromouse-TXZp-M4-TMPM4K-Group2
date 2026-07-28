/**
 * @file        systick.c
 * @brief       SysTick delay driver for TMPM4Ky RISC microcontrollers.
 * @version     V1.0.0
 * @date        29-05-2026
 *
 * @details
 *   Blocking delay using ARM SysTick timer. No ISR required.
 *   System clock: 10 MHz → 10 cycles = 1 µs. 
 *                 160 MHz → 160 cycles = 1 µs. 
 *
 *   Reference:
 *   - ARM DDI0403: https://developer.arm.com/documentation/ddi0403/ee
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "systick.h"

/* ==========================================================================
 *   Config
 * ========================================================================== */
#define SYSTICK_CLK_HZ          160000000UL                       /*!< Core clock feeding SysTick () */
#define SYSTICK_CYCLES_PER_US   (SYSTICK_CLK_HZ / 16000000UL)     /*!< 160 cycles = 1 µs @ 160 MHz */

/* ==========================================================================
 *   Delay Functions
 * ========================================================================== */

/**
 * @brief  Blocking delay in microseconds.
 * @param  val  Delay in µs (max = 0xFFFFFF / SYSTICK_CYCLES_PER_US ≈ 1.68 s @ 10 MHz).
 *
 *   LOAD = (val × SYSTICK_CYCLES_PER_US) - 1 cycles.
 *   Uses processor clock (CLKSOURCE = 1), no interrupt (TICKINT = 0).
 */
void SysTick_us(uint32_t val)
{
    if (val == 0U) { return; }

    /* Clamp to max LOAD (0xFFFFFF) */
    if (val > (SysTick_LOAD_RELOAD_Msk / SYSTICK_CYCLES_PER_US)) 
    {
        val = SysTick_LOAD_RELOAD_Msk / SYSTICK_CYCLES_PER_US;
    }

    SysTick->LOAD = (uint32_t)((val * SYSTICK_CYCLES_PER_US) - 1U);
    SysTick->VAL  = 0;                                            /* Clear current value */
    SysTick->CTRL = (SysTick_CTRL_CLKSOURCE_Msk |                 /* Processor clock */
                     SysTick_CTRL_ENABLE_Msk);                    /* Start counter */

    while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)) { ; }   /* Poll until done */

    SysTick->CTRL = 0;                                            /* Disable for next call */
}

/**
 * @brief  Blocking delay in milliseconds.
 * @param  val  Delay in ms.
 * @note   Loops SysTick_us(1000) because max single delay is ~104 ms.
 */
void SysTick_ms(uint32_t val)
{
    for (uint32_t i = 0U; i < val; i++) {
        SysTick_us(1000U);
    }
}