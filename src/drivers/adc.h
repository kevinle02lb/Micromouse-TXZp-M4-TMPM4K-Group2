/**
 * @file        adc.h
 * @brief       ADC-I driver for TMPM4Ky RISC microcontrollers.
 * @version     V1.0.0
 * @date        29-05-2026
 *
 * @details
 *   Configures ADC Units A and C for polled single-conversion reads.
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

#ifndef ADC_H
#define ADC_H

#include "TMPM4KyA.h"
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif


/* ==========================================================================
 *   Channel Position Constants
 * ========================================================================== */
#define AINA15_POS                              15U
#define AINA16_POS                              16U
#define AINC00_POS                              0U
#define AINC01_POS                              1U

/* ==========================================================================
 *   Clock Gating (CG / FSYSMENB)
 * ========================================================================== */
/** @name FSYSMENB — peripheral clock enables */
#define AINA_CG_FSYSMENB_IPMENB02               ((uint32_t)0x01 << 2U)   /*!< ADC Unit A clock gate */
#define AINC_CG_FSYSMENB_IPMENB04               ((uint32_t)0x01 << 4U)   /*!< ADC Unit C clock gate */

/** @name SPCLKEN — ADC conversion clock (ADCLK) enables */
#define CG_CGSPCLKEN_ADCKEN0                    ((uint32_t)0x01 << 16U)  /*!< ADC Unit A ADCLK */
#define CG_CGSPCLKEN_ADCKEN1                    ((uint32_t)0x01 << 17U)  /*!< ADC Unit B ADCLK */
#define CG_CGSPCLKEN_ADCKEN2                    ((uint32_t)0x01 << 18U)  /*!< ADC Unit C ADCLK */

/* ==========================================================================
 *   ADxCLK — Conversion Clock Setting
 * ========================================================================== */
#define ADxCLK_VADCLK_MASK                      ((uint32_t)0x07 << 0U)   /*!< ADCLK prescaler mask */
#define ADxCLK_VADCLK                           ((uint32_t)0x01 << 0U)   /*!< 000: ADCLK/4, 001: ADCLK/8 */

#define ADxCLK_EXAZ0_MASK                       ((uint32_t)0x0F << 3U)   /*!< EXAZ0 sampling time mask */
#define ADxCLK_EXAZ1_MASK                       ((uint32_t)0x0F << 8U)   /*!< EXAZ1 sampling time mask */
#define ADXCLK_EXAZ0_40MHZ                      ((uint32_t)0x01 << 3U)   /*!< EXAZ0 = 0.96 µs @ 40 MHz SCLK */
#define ADXCLK_EXAZ1_40MHZ                      ((uint32_t)0x01 << 8U)   /*!< EXAZ1 = 0.96 µs @ 40 MHz SCLK */

/* ==========================================================================
 *   ADxCR0 — Control Register 0
 * ========================================================================== */
#define ADxCR0_ADEN                             ((uint32_t)0x01 << 7U)   /*!< ADC enable */
#define ADxCR0_SGL                              ((uint32_t)0x01 << 1U)   /*!< Single conversion trigger */
#define ADxCR0_CNT                              ((uint32_t)0x01 << 0U)   /*!< Continuous conversion trigger */

/* ==========================================================================
 *   ADxCR1 — Control Register 1
 * ========================================================================== */
#define ADxCR1_TRGDMEN                          ((uint32_t)0x01 << 4U)   /*!< General-purpose trigger DMA request */
#define ADxCR1_SGLDMEN                          ((uint32_t)0x01 << 5U)   /*!< Single conversion DMA request */
#define ADxCR1_CNTDMEN                          ((uint32_t)0x01 << 6U)   /*!< Continuous conversion DMA request */
/* None of the above are set. Results are read by the CPU. */

/* ==========================================================================
 *   ADxST — Status Register
 * ========================================================================== */
#define ADxST_TRGF                              ((uint32_t)0x01 << 1U)   /*!< General-purpose trigger flag */
#define ADxST_SNGF                              ((uint32_t)0x01 << 2U)   /*!< Single conversion flag */
#define ADxST_CNTF                              ((uint32_t)0x01 << 3U)   /*!< Continuous conversion flag */
#define ADxST_ADBF                              ((uint32_t)0x01 << 7U)   /*!< ADC busy / operation flag */

/* ==========================================================================
 *   ADxMOD0 — Mode Setting Register 0
 * ========================================================================== */
#define ADxMOD0_DACON                           ((uint32_t)0x01 << 0U)   /*!< ADC operation enable (DACON) */
#define ADxMOD0_RCUT                            ((uint32_t)0x01 << 1U)   /*!< Low-power mode (RCUT) */

/* ==========================================================================
 *   ADxMOD1 — Mode Setting Register 1
 * ========================================================================== */
#define ADxMOD1_N5_SGL                          ((uint32_t)0x00306122)   /*!< MOD1 n=5 sampling recipe (0.96 µs @ 40 MHz SCLK; runs proportionally slower at lower SCLK) */

/* ==========================================================================
 *   ADxEXAZSEL — AIN Sampling Time Selection
 * ========================================================================== */
#define ADxEXAZSEL_AINA15                       ((uint32_t)0x01 << AINA15_POS)
#define ADxEXAZSEL_AINA16                       ((uint32_t)0x01 << AINA16_POS)
#define ADxEXAZSEL_AINC00                       ((uint32_t)0x01 << AINC00_POS)
#define ADxEXAZSEL_AINC01                       ((uint32_t)0x01 << AINC01_POS)

/* ==========================================================================
 *   ADxTSETn — General Purpose Start-up Factor Program Register
 * ========================================================================== */
#define ADxTSETn_AINSTn_AINx15                  ((uint32_t)0x0F << 0U)
#define ADxTSETn_AINSTn_AINx16                  ((uint32_t)0x10 << 0U)
#define ADxTSETn_AINSTn_AINx00                  ((uint32_t)0x00 << 0U)
#define ADxTSETn_AINSTn_AINx01                  ((uint32_t)0x01 << 0U)

#define ADxTSETn_TRGSn_MASK                     ((uint32_t)0x03 << 5U)
#define ADxTSETn_TRGSn_CNT                      ((uint32_t)0x01 << 5U)   /*!< Continuous conversion */
#define ADxTSETn_TRGSn_SGL                      ((uint32_t)0x02 << 5U)   /*!< Single conversion */

#define ADxTSETn_ENINTn_MASK                    ((uint32_t)0x01 << 7U)   /*!< Interrupt enable per TSETn */

/* ==========================================================================
 *   ADxREGn — Conversion Result Storage Register
 * ========================================================================== */
#define ADxREGn_ADRFn                           ((uint32_t)0x01 << 0U)   /*!< Result valid flag */
#define ADxREGn_ADOVRFn                         ((uint32_t)0x01 << 1U)   /*!< Overrun flag */
#define ADxREGn_ADRn                            ((uint32_t)0xFFF << 4U)  /*!< 12-bit ADC result [15:4] */

/* ==========================================================================
 *   Function Prototypes
 * ========================================================================== */
void ADC_Init(void);
void AINA_Init(void);
void AINC_Init(void);

void AINA_StartSGL(void);
void AINC_StartSGL(void);

uint16_t AINA_Read(uint8_t channel);
uint16_t AINC_Read(uint8_t channel);

void AINA_ReadPair(uint16_t *p_reg0, uint16_t *p_reg1);
void AINC_ReadPair(uint16_t *p_reg0, uint16_t *p_reg1);

#ifdef __cplusplus
}
#endif
#endif /* ADC_H */