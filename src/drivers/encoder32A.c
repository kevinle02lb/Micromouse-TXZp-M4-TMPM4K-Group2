/**
 * @file        encoder32A.c
 * @brief       A-ENC32 driver implementation for TMPM4Ky microcontrollers.
 * @version     V1.0.0
 * @date        25-05-2026
 *
 * @details
 *   Hardware quadrature encoder interface for motor feedback.
 *   Uses 2-phase decode (no Z input, no Phase 3).
 *
 *   Pin Assignments:
 *   - ENC0 (Left):  PN0 = ENC0A, PN1 = ENC0B
 *   - ENC2 (Right): PD3 = ENC2A, PD4 = ENC2B
 *
 *   Reference:
 *   - RM-A-ENC32-A: https://toshiba.semicon-storage.com/info/RM-A-ENC32-A_en_20250221.pdf?did=160602
 *   - App Note:      https://toshiba.semicon-storage.com/info/COM_A_ENC32-ANE_application_note_en_20231016.pdf?did=156382&prodName=TMPM4KNF10AFG
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "encoder32A.h"
#include "gpio.h"

/* ==========================================================================
 *   IMPORTANT NOTE
 * ==========================================================================
 *   When setting <ENRUN> = 1, do NOT change other bits at the same time.
 *   All operation settings must be configured BEFORE setting ENRUN = 1.
 * ========================================================================== */

/* ==========================================================================
 *   Initialization
 * ========================================================================== */

/**
 * @brief  Initialise all encoder modules
 * @note   Separately inits each module
 */
void ENC32A_Init(void)
{
    ENC0_Init();   /* Left motor encoder */
    ENC2_Init();   /* Right motor encoder */
}

void ENC0_Init(void)
{
    PORT_N_Init();
    
    /* [1] Enable peripheral clock */
    TSB_CG->FSYSMENB |= ENC0_CG_FSYSMENB_IPMENB06;

    /* [2] Stop encoder before config (ENRUN = 0) */
    TSB_EN0->TNCR &= ~TNCR_ENRUN_MASK;

    /* [3] Operation mode: Encoder (000), no Z, no Phase 3 */
    TSB_EN0->TNCR &= ~(TNCR_MODE_MASK | TNCR_P3EN_MASK | TNCR_ZEN_MASK);
    /* Clear counter */
    TSB_EN0->TNCR |= TNCR_ENCLR_MASK;
    TSB_EN0->TNCR &= ~TNCR_ENCLR_MASK;
    

    /* [4] Input circuit: 10 MHz sample clock, 400 ns noise cancellation */
    TSB_EN0->CLKCR = CLKCR_SPLCKS_8DIV;                          /* Fsys/8 = 10 MHz */
    TSB_EN0->INPCR &= ~(INPCR_SYNCSPLEN_MASK | INPCR_SYNCSPLMD_MASK | INPCR_SYNCNCZEN_MASK);
    TSB_EN0->INPCR = (TSB_EN0->INPCR & ~INPCR_NCT_MASK) | INPCR_NCT_4CYC;

    /* [5] Decoder: CW/CCW edge detection (default) */
    TSB_EN0->TNCR &= ~TNCR_DECMD_MASK;

    /* [6] Counter: max reload value */
    TSB_EN0->RELOAD = RELOAD_MAX;

    /* [7] Interrupts: disabled */
    TSB_EN0->INT = 0;
    TSB_EN0->INTCR &= ~INTCR_CLEAR_ALL;

    ENC0_Start();
}

void ENC2_Init(void)
{
    PORT_D_Init();

    /* [1] Enable peripheral clock */
    TSB_CG->FSYSMENB |= ENC2_CG_FSYSMENB_IPMENB08;

    /* [2] Stop encoder before config */
    TSB_EN2->TNCR &= ~TNCR_ENRUN_MASK;

    /* [3] Operation mode: Encoder, no Z, no Phase 3 */
    TSB_EN2->TNCR &= ~(TNCR_MODE_MASK | TNCR_P3EN_MASK | TNCR_ZEN_MASK);
    /* Clear counter */
    TSB_EN2->TNCR |= TNCR_ENCLR_MASK;
    TSB_EN2->TNCR &= ~TNCR_ENCLR_MASK;
    

    /* [4] Input circuit: 1.25 MHz sample / 3.2 µs @ 10 MHz Fsys */
    TSB_EN2->CLKCR = CLKCR_SPLCKS_8DIV;
    TSB_EN2->INPCR &= ~(INPCR_SYNCSPLEN_MASK | INPCR_SYNCSPLMD_MASK | INPCR_SYNCNCZEN_MASK);
    TSB_EN2->INPCR = (TSB_EN2->INPCR & ~INPCR_NCT_MASK) | INPCR_NCT_4CYC;

    /* [5] Decoder: CW/CCW edge detection */
    TSB_EN2->TNCR &= ~TNCR_DECMD_MASK;

    /* [6] Counter: max reload */
    TSB_EN2->RELOAD = RELOAD_MAX;

    /* [7] Interrupts: disabled */
    TSB_EN2->INT = 0;
    TSB_EN2->INTCR &= ~INTCR_CLEAR_ALL;

    ENC2_Start();
}

/* ==========================================================================
 *   Control
 * ========================================================================== */

void ENC0_Start(void)  { TSB_EN0->TNCR |= TNCR_ENRUN_MASK; }
void ENC2_Start(void)  { TSB_EN2->TNCR |= TNCR_ENRUN_MASK; }
void ENC0_Stop(void)   { TSB_EN0->TNCR &= ~TNCR_ENRUN_MASK; }
void ENC2_Stop(void)   { TSB_EN2->TNCR &= ~TNCR_ENRUN_MASK; }

void ENC0_ClearCNT(void)
{
    ENC0_Stop();
    TSB_EN0->TNCR |= TNCR_ENCLR_MASK;
    TSB_EN0->TNCR &= ~TNCR_ENCLR_MASK;
    ENC0_Start();
}

void ENC2_ClearCNT(void)
{
    ENC2_Stop();
    TSB_EN2->TNCR |= TNCR_ENCLR_MASK;
    TSB_EN2->TNCR &= ~TNCR_ENCLR_MASK;
    ENC2_Start();
}

/* ==========================================================================
 *   Status & Read
 * ========================================================================== */

/**
 * @brief  Get rotation direction for ENC0 (Left motor).
 * @return true = CW, false = CCW.
 * @note   Returns 0 when ENRUN = 0.
 */
bool ENC0_GetStatus(void)     { return (TSB_EN0->STS & STS_UD_MASK) != 0; }
bool ENC2_GetStatus(void)     { return (TSB_EN2->STS & STS_UD_MASK) != 0; }

int32_t ENC0_ReadCount(void)  { return (int32_t)TSB_EN0->CNT; }
int32_t ENC2_ReadCount(void)  { return (int32_t)TSB_EN2->CNT; }