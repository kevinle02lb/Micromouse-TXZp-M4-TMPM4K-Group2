/**
 * @file        encoder32A.h
 * @brief       A-ENC32 driver for TMPM4Ky microcontrollers.
 * @version     V1.0.0
 * @date        25-05-2026
 *
 * @details
 *   Hardware quadrature encoder interface definitions.
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#ifndef ENCODER32A_H
#define ENCODER32A_H

#include "TMPM4KyA.h"
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 *   Clock Gating
 * ========================================================================== */
#define ENC0_CG_FSYSMENB_IPMENB06   ((uint32_t)0x01 << 6U)   /*!< A-ENC32 ch0 clock */
#define ENC2_CG_FSYSMENB_IPMENB08   ((uint32_t)0x01 << 8U)   /*!< A-ENC32 ch2 clock */

/* ==========================================================================
 *   TNCR — Operation Control
 * ========================================================================== */
#define TNCR_MODE_MASK      ((uint32_t)0x07 << 17U)   /*!< Mode select mask */
#define TNCR_ZEN_MASK       ((uint32_t)0x01 << 7U)    /*!< Z input enable */
#define TNCR_P3EN_MASK      ((uint32_t)0x01 << 16U)   /*!< Phase 3 input enable */
#define TNCR_ENRUN_MASK     ((uint32_t)0x01 << 6U)    /*!< Encoder input enable */
#define TNCR_ENCLR_MASK     ((uint32_t)0x01 << 10U)   /*!< Counter clear */
#define TNCR_DECMD_MASK     ((uint32_t)0x03 << 22U)   /*!< Decoder direction select */

/* ==========================================================================
 *   INPCR — Input Procedure Control
 * ========================================================================== */
#define INPCR_SYNCSPLEN_MASK    ((uint32_t)0x01 << 0U)   /*!< PWM sync sampling enable */
#define INPCR_SYNCSPLMD_MASK    ((uint32_t)0x01 << 1U)   /*!< PWM sync sampling mode */
#define INPCR_SYNCNCZEN_MASK    ((uint32_t)0x01 << 2U)   /*!< Noise cancel at PWM-on */
#define INPCR_NCT_MASK          ((uint32_t)0x7F << 8U)   /*!< Noise cancellation time mask */
#define INPCR_NCT_4CYC          ((uint32_t)0x04 << 8U)   /*!< 4 × sample clock cycles */

/* ==========================================================================
 *   CLKCR — Sample Clock
 * ========================================================================== */
#define CLKCR_SPLCKS_8DIV       ((uint32_t)0x03 << 0U)   /*!< Fsys / 8 */
#define CLKCR_SPLCKS_DIV_MASK   ((uint32_t)0x03 << 0U)   /*!< Fsys mask 0x11 */

/* ==========================================================================
 *   STS — Status
 * ========================================================================== */
#define STS_UD_MASK         ((uint32_t)0x01 << 13U)  /*!< CW/CCW direction flag */

/* ==========================================================================
 *   RELOAD / INTCR
 * ========================================================================== */
#define RELOAD_MAX          ((uint32_t)0xFFFFFFFF)   /*!< Max counter value */
#define INTCR_CLEAR_ALL     ((uint32_t)0x3F << 0U)   /*!< Clear all interrupt enables */

/* ==========================================================================
 *   Function Prototypes
 * ========================================================================== */
void ENC32A_Init(void);
void ENC0_Init(void);
void ENC2_Init(void);
void ENC0_Start(void);
void ENC2_Start(void);
void ENC0_Stop(void);
void ENC2_Stop(void);
void ENC0_ClearCNT(void);
void ENC2_ClearCNT(void);

bool ENC0_GetStatus(void);
bool ENC2_GetStatus(void);
int32_t ENC0_ReadCount(void);
int32_t ENC2_ReadCount(void);

#ifdef __cplusplus
}
#endif
#endif /* ENCODER32A_H */