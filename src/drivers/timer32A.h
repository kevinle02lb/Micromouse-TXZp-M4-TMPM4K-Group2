/**
 * @file        timer32A.h
 * @brief       Timer32A driver for TMPM4Ky microcontrollers.
 * @version     V1.0.0
 * @date        11-05-2026
 *
 * @details
 *   PPG (PWM) configuration for motor control.
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#ifndef TIMER32A_H
#define TIMER32A_H

#include "TMPM4KyA.h"
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

extern volatile bool T32A01AC_IRQ_Fire;  

/* ==========================================================================
 *   Config
 * ========================================================================== */
#define T32A_PHIT0_HZ       80000000UL                /*!< ΦT0 timer clock — set to your actual value (80 MHz @ PLL-on) */
#define T32A_CH0_PERIOD     (T32A_PHIT0_HZ / 20000)   /*!< ΦT0 / 1:1 / 20 kHz PWM */
#define T32A_CH3_PERIOD     (T32A_PHIT0_HZ / 20000)   /*!< Same for CH3 PPG  */
#define T32A_CH1_1KHZ       (T32A_PHIT0_HZ / 1000)    /*!< ΦT0 / 1:1 / 1 kHz control loop */

#define T32A_MODE_16BIT     0
#define T32A_MODE_32BIT     1
#define T32A_PRSCL_1TO1     0

/* ==========================================================================
 *   Clock Gating
 * ========================================================================== */
#define T32A0_CG_FSYSMENA_IPMENA28  ((uint32_t)0x01 << 28U)   /*!< T32A ch0 clock */
#define T32A3_CG_FSYSMENA_IPMENA31  ((uint32_t)0x01 << 31U)   /*!< T32A ch3 clock */
#define T32A1_CG_FSYSMENA_IPMENA29  ((uint32_t)0x01 << 29U)   /*!< T32A ch1 clock */

/* ==========================================================================
 *   Mode / Run Registers
 * ========================================================================== */
#define T32A_MODE_MASK          ((uint32_t)0x03)          /*!< Mode bits [1:0] */
#define T32A_RUNx_RUNFLGx_MASK  ((uint32_t)0x01 << 4U)    /*!< RUN flag (read-only) */
#define T32A_RUNx_MASK          ((uint32_t)0x01 << 0U)    /*!< RUN control */

#define T32A_RUNx_SFTSTAx       ((uint32_t)0x01 << 1U)    /*!< Starts the operation by program. */
#define T32A_RUNx_SFTSTPx       ((uint32_t)0x01 << 2U)    /*!< Stops the operation by program. */

/* ==========================================================================
 *   Counter Control (CRA/CRB/CRC)
 * ========================================================================== */
#define T32A_CRx_PRSCLC_MASK    ((uint32_t)0x07 << 28U)   /*!< Prescaler mask */
#define T32A0_CRA_PRSCLA        ((uint32_t)T32A_PRSCL_1TO1 << 28U)
#define T32A0_CRB_PRSCLB        ((uint32_t)T32A_PRSCL_1TO1 << 28U)
#define T32A3_CRA_PRSCLA        ((uint32_t)T32A_PRSCL_1TO1 << 28U)
#define T32A3_CRB_PRSCLB        ((uint32_t)T32A_PRSCL_1TO1 << 28U)
#define T32A1_CRC_PRSCLC        ((uint32_t)T32A_PRSCL_1TO1 << 28U)

#define T32A_CRx_STARTx_MASK    ((uint32_t)0x07 << 0U)    /*!< Sets the counter start condition mask*/
#define T32A_CRx_STOPx_MASK     ((uint32_t)0x07 << 4U)    /*!< Sets the counter stop condition mask*/
#define T32A_CRx_RELDx_MASK     ((uint32_t)0x07 << 8U)    /*!< Reload timing mask */
#define T32A_CRx_UPDNx_MASK     ((uint32_t)0x03 << 16U)   /*!< Count direction mask */
#define T32A_UPDNx              ((uint32_t)0x00 << 16U)   /*!< 00 = Up, 01 = Down */
#define T32A_CRx_WBFx_MASK      ((uint32_t)0x01 << 20U)   /*!< Double-buffer enable */
#define T32A_CRx_CLKx_MASK      ((uint32_t)0x07 << 24U)   /*!< Selects the count clock mask*/


/* ==========================================================================
 *   Output Control
 * ========================================================================== */
#define T32A_OUTCRx0_ORCx_MASK  ((uint32_t)0x03 << 0U)    /*!< OUTCRx0 mask */
#define T32A_OUTCRx1_MASK       ((uint32_t)0xFF << 0U)    /*!< OUTCRx1 full mask */

/* Compare match output actions */
#define OUTCRxx_FTN_INVALID     0x00
#define OUTCRxx_FTN_SET         0x01
#define OUTCRxx_FTN_CLEAR       0x02
#define OUTCRxx_FTN_REVERSED    0x03

#define T32A_OUTCRx1_OCRCMPx0_SET   (OUTCRxx_FTN_SET   << 0U)
#define T32A_OUTCRx1_OCRCMPx0_CLEAR (OUTCRxx_FTN_CLEAR << 0U)
#define T32A_OUTCRx1_OCRCMPx1_SET   (OUTCRxx_FTN_SET   << 2U)
#define T32A_OUTCRx1_OCRCMPx1_CLEAR (OUTCRxx_FTN_CLEAR << 2U)

/* Preset output modes */
#define T32A_OUTPUT_HIGH        (T32A_OUTCRx1_OCRCMPx0_SET   | T32A_OUTCRx1_OCRCMPx1_SET)
#define T32A_OUTPUT_LOW         (T32A_OUTCRx1_OCRCMPx0_CLEAR | T32A_OUTCRx1_OCRCMPx1_CLEAR)
#define T32A_OUTPUT_PPG         (T32A_OUTCRx1_OCRCMPx0_SET   | T32A_OUTCRx1_OCRCMPx1_CLEAR)   /*!< SET on CMP0, CLEAR on CMP1 */
#define T32A_OUTPUT_INVERTED    (T32A_OUTCRx1_OCRCMPx0_CLEAR | T32A_OUTCRx1_OCRCMPx1_SET)     /*!< CLEAR on CMP0, SET on CMP1 */

/* ==========================================================================
 *   Interrupts
 * ========================================================================== */
#define T32A_IMx_IMx0           ((uint32_t)0x01 << 0U)     /*!< Control to mask the match detection interrupt request ([T32AxRGx0]) */
#define T32A_IMx_IMx1           ((uint32_t)0x01 << 1U)     /*!< Control to mask the match detection interrupt request ([T32AxRGx1]) */
#define T32A1_IRQ_PRIORITY      5

#define T32A1_STx_INTC0          ((uint32_t)0x01 << 0U)     /*!< Indicates a match flag ([T32AxRGx0]) */
#define T32A1_STx_INTC1          ((uint32_t)0x01 << 1U)     /*!< Indicates a match flag ([T32AxRGx1]) */

/* ==========================================================================
 *   Function Prototypes
 * ========================================================================== */
void T32A_Init(void);
void T32A0_Init(uint16_t period);
void T32A3_Init(uint16_t period);
void T32A1_Init(void);
void T32A0_Start(void);
void T32A3_Start(void);
void T32A1_Start(void);
void T32A0_Stop(void);
void T32A3_Stop(void);
void T32A1_Stop(void);

/* Timer compare registers (duty/period) */
void T32A0_SetTimerA0(uint16_t val);   /* RGA0 */
void T32A0_SetTimerA1(uint16_t val);   /* RGA1 */
void T32A0_SetTimerB0(uint16_t val);   /* RGB0 */
void T32A0_SetTimerB1(uint16_t val);   /* RGB1 */
void T32A3_SetTimerA0(uint16_t val);
void T32A3_SetTimerA1(uint16_t val);
void T32A3_SetTimerB0(uint16_t val);
void T32A3_SetTimerB1(uint16_t val);

/* Output control */
void T32A0_SetOutCRA1(uint32_t mode);  /* OUTCRA1 */
void T32A0_SetOutCRB1(uint32_t mode);  /* OUTCRB1 */
void T32A3_SetOutCRA1(uint32_t mode);
void T32A3_SetOutCRB1(uint32_t mode);

/* Interrupt */
void T32A1_Interrupt_Enable(void);

#ifdef __cplusplus
}
#endif
#endif /* TIMER32A_H */