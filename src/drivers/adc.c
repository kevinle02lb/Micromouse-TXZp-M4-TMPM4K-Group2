/**
 * @file        adc.c
 * @brief       ADC-I driver implementation for TMPM4Ky RISC microcontrollers.
 * @version     V1.0.0
 * @date        29-05-2026
 *
 * @details
 *   Configures ADC Units A and C
 *   ADCLK = 160 MHz raw; SCLK = 40 MHz after /4 prescaler.
 *
 *   Pin Assignments:
 *   - Unit A: PL0 (AINA16), PL1 (AINA15) — Left IR sensors
 *   - Unit C: PJ0 (AINC00), PJ1 (AINC01) — Right IR sensors
 *  
 *   - PL0 = Far Left IR, PL1 = Left IR (Left ADC)
 *   - PJ1 = Right IR, PJ0 = Far Right IR (Right ADC)
 *
 *   Reference Documents (Toshiba):
 *   - Product Info:        https://toshiba.semicon-storage.com/info/TXZP-PINFO-M4K(2)_en_20231225.pdf?did=70854
 *   - ADC-I RM:            https://toshiba.semicon-storage.com/info/RM-ADC-I_en_20251205.pdf?did=166835
 *   - I/O Ports RM:        https://toshiba.semicon-storage.com/info/TXZP-PORT-M4K(2)_en_20250620.pdf?did=70850
 *   - EXCEPT-M4K(2) RM:    https://toshiba.semicon-storage.com/info/TXZP-EXCEPT-M4K(2)_en_20230414.pdf?did=70852
 *   - App Note:            https://toshiba.semicon-storage.com/info/COM_ADC_MON-ANE_application_note_en_20231016.pdf?did=156383&prodName=TMPM4KNF10AFG
 * 
 *   Reference Documents (ARM — NVIC addresses and functions):
 *   - core_cm4.h    — CMSIS NVIC helper functions (__NVIC_EnableIRQ, etc.)
 *   - cmsis_gcc.h   — Compiler barriers and core register access
 *   - startup_TMPM4KNA.s — Vector table with IRQ handler names
 *
 * @note
 *   NVIC register names (ISER, ICER, IPR) are ARM standard.
 *   Toshiba manual uses different names (<SETENA>, <CLRENA>, <PRI_n>).
 *   See EXCEPT-M4K(2) §5.6 for the mapping.
 *
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "adc.h"
#include "systick.h"
#include "gpio.h"
 
/* ==========================================================================
 *   Initialization
 * ========================================================================== */

/**
 * @brief  Initialize both ADC units (A and C) for single conversion.
 */
void ADC_Init(void)
{
    AINA_Init();
    AINC_Init();
}

/**
 * @brief  Initialize ADC Unit A (AINA15, AINA16).
 * @details
 *   Follows Section 3.2.2 of the ADC-I RM for single-conversion setup.
 *   Steps: clock enable → disable ADEN → set prescaler → configure sampling
 *   time → set mode → program TSET registers → enable ADC.
 * 
 *   Setup sequence per ADC-I Reference Manual:
 *   1. Enable peripheral clock in CG->FSYSMENB
 *   2. Disable ADC (ADEN=0) before changing config
 *   3. Enable ADCLK in CG->SPCLKEN
 *   4. Set prescaler (/4) and sampling time in CLK register
 *   5. Enable ADC analog circuit (DACON=1), exit low-power (RCUT=0)
 *   6. Wait 3 us for stabilization
 *   7. Set MOD1/MOD2 for conversion timing
 *   8. Configure TSET0/TSET1 — which channel goes to which result register
 *   9. Re-enable ADC (ADEN=1)
 *
 *   Result routing:
 *     TSET0: AINA16 -> REG0
 *     TSET1: AINA15 -> REG1
 */
void AINA_Init(void)
{
    /* GPIO ADC inputs (Unit A) */
    PORT_L_Init();      

    /* [1] Enable ADC Unit A peripheral clock */
    TSB_CG->FSYSMENB |= AINA_CG_FSYSMENB_IPMENB02;

    /* [2] Disable ADC before configuration (ADEN must be 0 to change settings) */
    TSB_ADA->CR0 &= ~ADxCR0_ADEN;

    /* [3] Enable ADC conversion clock (ADCLK) */
    TSB_CG->SPCLKEN |= CG_CGSPCLKEN_ADCKEN0;

    /* [4] ADCLK prescaler = /4 -> SCLK = ADCLK/4. EXAZ left at reset (m=1) */
    TSB_ADA->CLK &= ~ADxCLK_VADCLK_MASK;

    /* [5] Enable ADC analog circuit, exit low-power mode */
    TSB_ADA->MOD0 |= ADxMOD0_DACON;
    TSB_ADA->MOD0 &= ~ADxMOD0_RCUT;

    /* [6] Wait for analog stabilization (3 us minimum per datasheet) - Note3: 5.2.5. [ADxMOD0] (Mode Setting Register0)*/
    SysTick_us(3U);

    /* [7] Conversion timing — keep a DOCUMENTED MOD1 value (the 40 MHz recipe).
     *     Do NOT drop to reset 0x00004000: it is not a characterized operating
     *     config. At 2.5 MHz SCLK. */
    TSB_ADA->MOD1 = ADxMOD1_N5_SGL;
    TSB_ADA->MOD2 = 0;

    /* [8] Select EXAZ0 sampling time for both channels */
    TSB_ADA->EXAZSEL &= ~(ADxEXAZSEL_AINA15 | ADxEXAZSEL_AINA16);

    /* [9] Program conversion sequence:
     *   TSET0: AINA16 -> REG0
     *   TSET1: AINA15 -> REG1
     */
    TSB_ADA->TSET0 = (ADxTSETn_TRGSn_SGL | ADxTSETn_AINSTn_AINx16);
    TSB_ADA->TSET1 = (ADxTSETn_TRGSn_SGL | ADxTSETn_AINSTn_AINx15);

    /* [10] No transfer request. Results are read by the CPU. */
    TSB_ADA->CR1 = 0U;

    /* [11] Re-enable ADC to start operation */
    TSB_ADA->CR0 |= ADxCR0_ADEN;
}

/**
 * @brief  Initialize ADC Unit C (AINC00, AINC01).
 * @details  Same sequence as @ref AINA_Init(), applied to Unit C.
 */
void AINC_Init(void)
{
    /* GPIO ADC inputs (Unit C) */
    PORT_J_Init();  

    /* [1] Enable ADC Unit C peripheral clock */
    TSB_CG->FSYSMENB |= AINC_CG_FSYSMENB_IPMENB04;

    /* [2] Disable ADC before configuration */
    TSB_ADC->CR0 &= ~ADxCR0_ADEN;

    /* [3] Enable ADC conversion clock (ADCLK) */
    TSB_CG->SPCLKEN |= CG_CGSPCLKEN_ADCKEN2;

    /* [4] Set ADCLK prescaler = /4, sampling time for EXAZ0/EXAZ1 */
    TSB_ADC->CLK &= ~ADxCLK_VADCLK_MASK;
    

    /* [5] Enable ADC analog circuit, exit low-power mode */
    TSB_ADC->MOD0 |= ADxMOD0_DACON;
    TSB_ADC->MOD0 &= ~ADxMOD0_RCUT;

    /* [6] Wait for analog stabilization */
    SysTick_us(3U);

    /* [7] Set conversion timing */
    TSB_ADC->MOD1 = ADxMOD1_N5_SGL;
    TSB_ADC->MOD2 = 0;

    /* [8] Select EXAZ0 sampling time for both channels */
    TSB_ADC->EXAZSEL &= ~(ADxEXAZSEL_AINC00 | ADxEXAZSEL_AINC01);

    /* [9] Program conversion sequence:
     *   TSET0: AINC01 -> REG0
     *   TSET1: AINC00 -> REG1
     */
    TSB_ADC->TSET0 = (ADxTSETn_TRGSn_SGL | ADxTSETn_AINSTn_AINx01);
    TSB_ADC->TSET1 = (ADxTSETn_TRGSn_SGL | ADxTSETn_AINSTn_AINx00);

    /* [10] No transfer request. Results are read by the CPU. */
    TSB_ADC->CR1 = 0U;

    /* [11] Re-enable ADC to start operation */
    TSB_ADC->CR0 |= ADxCR0_ADEN;
}

/* ==========================================================================
 *   Conversion Control
 * ========================================================================== */

/**
 * @brief  Trigger a single conversion on ADC Unit A.
 */
void AINA_StartSGL(void)
{
    TSB_ADA->CR0 |= ADxCR0_SGL;
    while (!(TSB_ADA->ST & ADxST_SNGF)) { ; }   /* wait for SNGF to assert (up to 5 SCLK latency) */
    while (  TSB_ADA->ST & ADxST_SNGF ) { ; }   /* then wait for it to clear = done */
}

/**
 * @brief  Trigger a single conversion on ADC Unit C.
 */
void AINC_StartSGL(void)
{
    TSB_ADC->CR0 |= ADxCR0_SGL;
    while (!(TSB_ADC->ST & ADxST_SNGF)) { ; }   /* wait for SNGF to assert (up to 5 SCLK latency) */
    while (  TSB_ADC->ST & ADxST_SNGF ) { ; }   /* then wait for it to clear = done */
}

/* ==========================================================================
 *   Blocking Read
 * ========================================================================== */

/**
 * @brief  Blocking read of one channel from ADC Unit A.
 * @param  channel  15 for AINA15, 16 for AINA16.
 * @return 12-bit result right-aligned in a 16-bit word.
 * @note   Polls ADxST.SNGF.
 */
uint16_t AINA_Read(uint8_t channel)
{
    uint16_t result = 0;

    AINA_StartSGL();
    while (TSB_ADA->ST & ADxST_SNGF) { ; }

    if (channel == 16) 
    {
        result = (uint16_t)((TSB_ADA->REG0 & ADxREGn_ADRn) >> 4U);
    } 
    else if (channel == 15) 
    {
        result = (uint16_t)((TSB_ADA->REG1 & ADxREGn_ADRn) >> 4U);
    }
    return result;
}

/**
 * @brief  Blocking read of one channel from ADC Unit C.
 * @param  channel  0 for AINC00, 1 for AINC01.
 * @return 12-bit result right-aligned in a 16-bit word.
 * @note   Polls ADxST.SNGF.
 */
uint16_t AINC_Read(uint8_t channel)
{
    uint16_t result = 0;

    AINC_StartSGL();
    while (TSB_ADC->ST & ADxST_SNGF) { ; }

    if (channel == 1) 
    {
        result = (uint16_t)((TSB_ADC->REG0 & ADxREGn_ADRn) >> 4U);
    } 
    else if (channel == 0) 
    {
        result = (uint16_t)((TSB_ADC->REG1 & ADxREGn_ADRn) >> 4U);
    }
    return result;
}

/**
 * @brief  Trigger a single-conversion pair on ADC Unit A and read both results.
 * @param  p_reg0  Destination for REG0 (AINA16), right-aligned 12-bit.
 * @param  p_reg1  Destination for REG1 (AINA15), right-aligned 12-bit.
 * @note   Polls ADxREGn.ADRFn on REG1, which converts last, so both results
 *         are valid before readout.
 */
void AINA_ReadPair(uint16_t *p_reg0, uint16_t *p_reg1)
{
    TSB_ADA->CR0 |= ADxCR0_SGL;
    while (!(TSB_ADA->REG1 & ADxREGn_ADRFn)) { ; }   /* wait for pair to post */

    *p_reg0 = (uint16_t)((TSB_ADA->REG0 & ADxREGn_ADRn) >> 4U);
    *p_reg1 = (uint16_t)((TSB_ADA->REG1 & ADxREGn_ADRn) >> 4U);
}


/**
 * @brief  Trigger a single-conversion pair on ADC Unit C and read both results.
 * @param  p_reg0  Destination for REG0 (AINC01), right-aligned 12-bit.
 * @param  p_reg1  Destination for REG1 (AINC00), right-aligned 12-bit.
 * @note   Polls ADxREGn.ADRFn on REG1, which converts last, so both results
 *         are valid before readout.
 */
void AINC_ReadPair(uint16_t *p_reg0, uint16_t *p_reg1)
{
    TSB_ADC->CR0 |= ADxCR0_SGL;
    while (!(TSB_ADC->REG1 & ADxREGn_ADRFn)) { ; }   /* wait for pair to post */

    *p_reg0 = (uint16_t)((TSB_ADC->REG0 & ADxREGn_ADRn) >> 4U);
    *p_reg1 = (uint16_t)((TSB_ADC->REG1 & ADxREGn_ADRn) >> 4U);
}