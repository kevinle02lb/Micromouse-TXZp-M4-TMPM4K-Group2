/**
 * @file        dma.h
 * @brief       DMAC-B driver for TMPM4Ky RISC microcontrollers.
 * @version     V1.0.0
 * @date        01-06-2026
 *
 * @details
 *   Provides initialization and channel configuration for the DMAC-B (PL230).
 *   The control table must be 1024-byte aligned; CTRLBASEPTR ignores bits [9:0].
 *
 *   Reference:
 *   - DMAC-B RM:    https://toshiba.semicon-storage.com/info/RM-DMAC-B_en_20241031.pdf?did=160537
 *   - ARM PL230:    https://developer.arm.com/documentation/ddi0417/a?lang=en
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#ifndef DMA_H
#define DMA_H

#include "TMPM4KyA.h"
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 *   Channel Control Data Structure (16 bytes per entry)
 * ========================================================================== */
typedef struct
{
    __IO uint32_t SrcEndPtr;     /*!< Source end address (end-pointer arithmetic) */
    __IO uint32_t DstEndPtr;     /*!< Destination end address */
    __IO uint32_t ChnlCfg;       /*!< Transfer mode / cycle control */
         uint32_t RESERVED;      /*!< Padding , must not be written */
} DMA_ChnlCtrlData_t;

/* ==========================================================================
 *   DMAChnlCfg Bit-Field Macros (PL230 / DMAC-B)
 * ========================================================================== */

/* dst_inc [31:30] */
#define DMA_DST_INC_BYTE        ((uint32_t)0x0U << 30U)   /*!< +1 byte  */
#define DMA_DST_INC_HWORD       ((uint32_t)0x1U << 30U)   /*!< +2 bytes */
#define DMA_DST_INC_WORD        ((uint32_t)0x2U << 30U)   /*!< +4 bytes */
#define DMA_DST_INC_NONE        ((uint32_t)0x3U << 30U)   /*!< No increment */

/* dst_size [29:28] */
#define DMA_DST_SIZE_BYTE       ((uint32_t)0x0U << 28U)   /*!< 8-bit  */
#define DMA_DST_SIZE_HWORD      ((uint32_t)0x1U << 28U)   /*!< 16-bit */
#define DMA_DST_SIZE_WORD       ((uint32_t)0x2U << 28U)   /*!< 32-bit */

/* src_inc [27:26] */
#define DMA_SRC_INC_BYTE        ((uint32_t)0x0U << 26U)   /*!< +1 byte  */
#define DMA_SRC_INC_HWORD       ((uint32_t)0x1U << 26U)   /*!< +2 bytes */
#define DMA_SRC_INC_WORD        ((uint32_t)0x2U << 26U)   /*!< +4 bytes */
#define DMA_SRC_INC_NONE        ((uint32_t)0x3U << 26U)   /*!< No increment */

/* src_size [25:24] */
#define DMA_SRC_SIZE_BYTE       ((uint32_t)0x0U << 24U)   /*!< 8-bit  */
#define DMA_SRC_SIZE_HWORD      ((uint32_t)0x1U << 24U)   /*!< 16-bit */
#define DMA_SRC_SIZE_WORD       ((uint32_t)0x2U << 24U)   /*!< 32-bit */

/* R_power [17:14] , arbitration interval */
#define DMA_R_POWER_1           ((uint32_t)0x0U << 14U)   /*!< Arbitrate every 1 transfer  */
#define DMA_R_POWER_2           ((uint32_t)0x1U << 14U)   /*!< Arbitrate every 2 transfers */
#define DMA_R_POWER_4           ((uint32_t)0x2U << 14U)   /*!< Arbitrate every 4 transfers */
#define DMA_R_POWER_8           ((uint32_t)0x3U << 14U)   /*!< Arbitrate every 8 transfers */

/* n_minus_1 [13:4] */
#define DMA_N_MINUS_1(n)        ((uint32_t)((n) - 1U) << 4U)

/* next_useburst [3] */
#define DMA_NEXT_USEBURST       ((uint32_t)0x01 << 3U)    /*!< Force burst on next cycle */

/* cycle_ctrl [2:0] */
#define DMA_CYCLE_CTRL_MASK     ((uint32_t)0x07U << 0U)   /*!< Field width, for completion checks */
#define DMA_CYCLE_CTRL_SUSPEND  ((uint32_t)0x0U << 0U)   /*!< Stop / Invalid (auto-cleared after xfer) */
#define DMA_CYCLE_CTRL_UNIT     ((uint32_t)0x1U << 0U)   /*!< Unit (normal) */
#define DMA_CYCLE_CTRL_CNT      ((uint32_t)0x2U << 0U)   /*!< Auto-request / Continuous normal */
#define DMA_CYCLE_CTRL_REPEAT   ((uint32_t)0x3U << 0U)   /*!< Ping-pong / Repeat */

/* ==========================================================================
 *   DMAC-B Register Bit Masks
 * ========================================================================== */

/* Clock gating */
#define DMAC_CG_FSYSMENB_IPMENB17   ((uint32_t)0x01 << 17U)

/* Master enable */
#define DMAxStatus_Master_enable    ((uint32_t)0x01 << 0U)
#define DMAxCfg_Master_enable       ((uint32_t)0x01 << 0U)

/* Channel masks */
#define DMAxChnlUseburstSet_MASK    ((uint32_t)0xFFFFFFFF)
#define DMAxChnlReqMaskSet_MASK     ((uint32_t)0xFFFFFFFF)
#define DMAxChnlReqMaskClr_MASK     ((uint32_t)0xFFFFFFFF)
#define DMAxChnlEnableSet_MASK      ((uint32_t)0xFFFFFFFF)
#define DMA_CHANNEL_MASK(ch)        ((uint32_t)0x1U << (ch))

/* ==========================================================================
 *   Channel & Table Constants
 * ========================================================================== */
#define DMA_ADASGL_DMAREQ           16   /*!< ADC Unit A single-conversion DMA request */
#define DMA_ADCSGL_DMAREQ           18   /*!< ADC Unit C single-conversion DMA request */
#define DMA_CHANNEL_COUNT           32U  /*!< Fixed by DMAC-B architecture */

/* ==========================================================================
 *   Trigger Selector (TRGSEL)
 *
 *   Channels 16 to 31 take their request through the trigger selector rather
 *   than straight from the peripheral. It picks one of several sources and
 *   gates the output; both default to off, so a request never arrives until
 *   this is programmed.
 *
 *   TSEL16 and TSEL18 are clocked from the ADC unit clock gates, so the
 *   register only holds a write once ADC_Init() has run.
 *
 *   PINFO-M4K(2) 2.2.4.1, [TSEL0CR0].
 * ========================================================================== */

/* Channel 16: ADC unit A */
#define TSEL_EN0                ((uint32_t)0x01U << 0U)    /*!< Trigger output enable */
#define TSEL_OUTSEL0            ((uint32_t)0x01U << 1U)    /*!< Edge detection enable */
#define TSEL_INSEL0_ADA_SGL     ((uint32_t)0x01U << 4U)    /*!< Source = ADC unit A single conversion */

/* Channel 18: ADC unit C */
#define TSEL_EN2                ((uint32_t)0x01U << 16U)   /*!< Trigger output enable */
#define TSEL_OUTSEL2            ((uint32_t)0x01U << 17U)   /*!< Edge detection enable */
#define TSEL_INSEL2_ADC_SGL     ((uint32_t)0x01U << 20U)   /*!< Source = ADC unit C single conversion */

#define TSEL_ADC_DMA_ROUTE      (TSEL_INSEL0_ADA_SGL | TSEL_OUTSEL0 | TSEL_EN0 | \
                                 TSEL_INSEL2_ADC_SGL | TSEL_OUTSEL2 | TSEL_EN2)

/* ==========================================================================
 *   Function Prototypes
 * ========================================================================== */
void DMAC_Init(void);
void DMA_ConfigChannel(uint8_t channel, uint32_t src_end, uint32_t dst_end, uint32_t chnl_cfg);
void DMA_SetupForADC(void);

bool DMA_ADCTransferDone(void);

volatile uint16_t* DMA_GetADCABuffer(void);
volatile uint16_t* DMA_GetADCCBuffer(void);

#ifdef __cplusplus
}
#endif
#endif /* DMA_H */