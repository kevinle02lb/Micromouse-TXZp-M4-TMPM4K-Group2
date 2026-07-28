/**
 * @file        adc.c
 * @brief       ADC-I driver implementation for TMPM4Ky RISC microcontrollers.
 * @version     V1.0.0
 * @date        29-05-2026
 *
 * @details
 *   Configures ADC Units A and C for single-conversion mode with DMA.
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
#include "dma.h"
#include "gpio.h"

/* ==========================================================================
 *   INTERRUPT NOTES (Read this first)
 * ==========================================================================
 *
 *   ADC single-conversion interrupts:
 *     INTADASGL = IRQ #44  (ADC Unit A done)
 *     INTADCSGL = IRQ #58  (ADC Unit C done)
 *
 *   These are "direct" interrupts (Route H in Toshiba §4.3.1).
 *   They bypass INTIF and go straight to the NVIC.
 *
 *   To enable an interrupt manually (without CMSIS):
 *     NVIC_ISER[IRQn >> 5] = (1UL << (IRQn & 0x1F));
 *
 *   NVIC base address: 0xE000E100 (ARM standard, not in Toshiba doc)
 *   ISER offset: +0x000, ICER: +0x080, ICPR: +0x180, IPR: +0x300
 *
 *   To enable with CMSIS (recommended):
 *     __NVIC_SetPriority(INTADASGL_IRQn, 5);
 *     __NVIC_ClearPendingIRQ(INTADASGL_IRQn);
 *     __NVIC_EnableIRQ(INTADASGL_IRQn);
 *
 *   CURRENTLY: DMA handles data movement. These interrupts are NOT enabled
 *   in the NVIC, so the CPU never enters the handlers below.
 *   The handlers are stubs for future use.
 * ========================================================================== */

 
/* ==========================================================================
 *   Initialization
 * ========================================================================== */

/**
 * @brief  Initialize both ADC units (A and C) for single-conversion DMA.
 * @note   DMA channel control data is re-armed inside @ref Start_ADC() before
 *         each conversion pair. Do not call @ref DMA_SetupForADC() here.
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
 *   time → set mode → program TSET registers → enable DMA request → enable ADC.
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
 *   9. Enable DMA request (SGLDMEN)
 *   10. Re-enable ADC (ADEN=1)
 *
 *   Result routing:
 *     TSET0: AINA16 -> REG0
 *     TSET1: AINA15 -> REG1, with interrupt flag (ENINTn)
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

    /* [6] Wait for analog stabilization (3 us minimum per datasheet) */
    SysTick_us(3U);

    /* [7] Conversion timing — keep a DOCUMENTED MOD1 value (the 40 MHz recipe).
     *     Do NOT drop to reset 0x00004000: it is not a characterized operating
     *     config. At 2.5 MHz SCLK. */
    TSB_ADA->MOD1 = ADxMOD1_N5_SGL;
    TSB_ADA->MOD2 = 0;

    /* [8] Select EXAZ0 sampling time for both channels */
    TSB_ADA->EXAZSEL &= ~(ADxEXAZSEL_AINA15 | ADxEXAZSEL_AINA16);

    /* [9] Program conversion sequence:
     *   TSET0: AINA16 -> REG0 (single trigger, no interrupt flag)
     *   TSET1: AINA15 -> REG1 (single trigger, WITH interrupt flag ENINT1)
     *   ENINTn generates the DMA request / interrupt when this conversion completes
     */
    TSB_ADA->TSET0 = (ADxTSETn_TRGSn_SGL | ADxTSETn_AINSTn_AINx16);
    TSB_ADA->TSET1 = (ADxTSETn_TRGSn_SGL | ADxTSETn_AINSTn_AINx15 |
                        ADxTSETn_ENINTn_MASK);

    /* [10] Enable DMA request on single conversion complete */
    TSB_ADA->CR1 = ADxCR1_SGLDMEN;

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
     *   TSET0: AINC01 -> REG0 (single trigger, no interrupt flag)
     *   TSET1: AINC00 -> REG1 (single trigger, WITH interrupt flag ENINT1)
     */
    TSB_ADC->TSET0 = (ADxTSETn_TRGSn_SGL | ADxTSETn_AINSTn_AINx01);
    TSB_ADC->TSET1 = (ADxTSETn_TRGSn_SGL | ADxTSETn_AINSTn_AINx00 |
                        ADxTSETn_ENINTn_MASK);

    /* [10] Enable DMA request on single conversion complete */
    TSB_ADC->CR1 = ADxCR1_SGLDMEN;

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

/**
 * @brief  Start a paired ADC conversion with DMA re-arm.
 * @details
 *   Must be called every time fresh samples are needed.
 *   The PL230/DMAC-B controller invalidates (sets cycle_ctrl = 0) the channel
 *   control structure after each transfer completes. Therefore,
 *   @ref DMA_SetupForADC() must be called first to restore the descriptors.
 *
 *   Flow:
 *   1. DMA_SetupForADC() restores cycle_ctrl for channels 16 & 18
 *   2. AINA_StartSGL() triggers Unit A conversion
 *   3. AINC_StartSGL() triggers Unit C conversion
 *   4. DMA moves results to adc_a_buffer[] and adc_c_buffer[] without CPU
 */
void Start_ADC(void)
{
    DMA_SetupForADC();   /* Re-arm DMA channels (restore cycle_ctrl) */
    AINA_StartSGL();
    AINC_StartSGL();
}

/* ==========================================================================
 *   Blocking Read (DMA bypass / debug fallback)
 * ========================================================================== */

/**
 * @brief  Blocking read from ADC Unit A (no DMA).
 * @param  channel  15 for AINA15, 16 for AINA16.
 * @return 12-bit result right-aligned in a 16-bit word.
 * @note   Polls ADxST.SNGF; do not use while DMA is active on the same unit.
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
 * @brief  Blocking read from ADC Unit C (no DMA).
 * @param  channel  0 for AINC00, 1 for AINC01.
 * @return 12-bit result right-aligned in a 16-bit word.
 * @note   Polls ADxST.SNGF; do not use while DMA is active on the same unit.
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

/* ==========================================================================
 *   Interrupt Handlers
 * ========================================================================== */

/**
 * @brief  ADC Unit A single-conversion interrupt handler (IRQ #44).
 * @note   STUB — not currently enabled in NVIC.
 *
 *   To enable this interrupt:
 *     __disable_irq();
 *     __NVIC_SetPriority(INTADASGL_IRQn, 5);     // Priority 5 (0 = highest)
 *     __NVIC_ClearPendingIRQ(INTADASGL_IRQn);    // Clear any stale pending
 *     __NVIC_EnableIRQ(INTADASGL_IRQn);          // Set NVIC ISER bit 12
 *     __enable_irq();
 *
 *   INTADASGL is a "direct" interrupt (Route H, §4.3.1) — no INTIF config needed.
 *   If using DMA, the DMA controller handles data movement; this ISR is optional.
 *   If NOT using DMA, read REG0/REG1 here and clear ADC status flags.
 */
void INTADASGL_IRQHandler(void)
{
    /* TODO: Add application logic if interrupt is enabled */
    /* If using DMA: DMA already moved data to adc_a_buffer[] */
    /* If NOT using DMA: read TSB_ADA->REG0/REG1 here, then clear flags */
}

/**
 * @brief  ADC Unit C single-conversion interrupt handler (IRQ #58).
 * @note   STUB — not currently enabled in NVIC.
 *
 *   To enable this interrupt:
 *     __disable_irq();
 *     __NVIC_SetPriority(INTADCSGL_IRQn, 5);
 *     __NVIC_ClearPendingIRQ(INTADCSGL_IRQn);
 *     __NVIC_EnableIRQ(INTADCSGL_IRQn);          // Set NVIC ISER bit 26
 *     __enable_irq();
 *
 *   INTADCSGL is a "direct" interrupt (Route H, §4.3.1) — no INTIF config needed.
 */
void INTADCSGL_IRQHandler(void)
{
    /* TODO: Add application logic if interrupt is enabled */
    /* If using DMA: DMA already moved data to adc_c_buffer[] */
    /* If NOT using DMA: read TSB_ADC->REG0/REG1 here, then clear flags */
}