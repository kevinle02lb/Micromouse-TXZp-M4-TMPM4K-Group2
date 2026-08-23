/**
 * @file        dma.c
 * @brief       DMAC-B driver implementation for TMPM4Ky RISC microcontrollers.
 * @version     V1.0.0
 * @date        01-06-2026
 *
 * @details
 *   Configures the DMAC-B (PL230-compatible) for ADC burst transfers.
 *   The control table must reside in RAM and be 1024-byte aligned because
 *   CTRLBASEPTR ignores bits [9:0].
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

#include "dma.h"

/* ==========================================================================
 *   Control Table (1024-byte aligned)
 * ========================================================================== */
/**
 * @brief  Primary + alternate control table (32 channels × 2 = 64 entries).
 * @note   64 entries × 16 bytes = 1024 bytes. Alignment = 1024 because
 *         CTRLBASEPTR masks off bits [9:0].
 */
__attribute__((aligned(1024))) static DMA_ChnlCtrlData_t DMA_CtrlTable[DMA_CHANNEL_COUNT * 2U];

/* ==========================================================================
 *   ADC Result Buffers
 *
 *   Each transfer is a verbatim halfword copy of ADxREGn, so the 12-bit result
 *   arrives in bits [15:4] with the ADRFn and ADOVRFn flags below it. Callers
 *   mask and shift before use.
 * ========================================================================== */
static volatile uint16_t adc_a_buffer[2];   /*!< AINA16 [0], AINA15 [1] */
static volatile uint16_t adc_c_buffer[2];   /*!< AINC01 [0], AINC00 [1] */

/* ==========================================================================
 *   Initialization
 * ========================================================================== */

/**
 * @brief  Initialize the DMAC-B controller.
 * @details
 *   1. Enable the peripheral clock.
 *   2. Point CTRLBASEPTR at the aligned control table.
 *   3. Common register initialization per DMAC-B RM 3.3.3: master enable,
 *      mask all requests, enable all channels.
 *   4. Force burst mode on the ADC channels.
 *
 *   Request masks are left set. DMA_SetupForADC() releases them once a
 *   descriptor exists, so an early request cannot hit an empty channel.
 */
void DMAC_Init(void)
{
    /* [1] Clock enable */
    TSB_CG->FSYSMENB |= DMAC_CG_FSYSMENB_IPMENB17;

    /* [2] Control table base address */
    TSB_DMAA->CTRLBASEPTR = (uint32_t)DMA_CtrlTable;

    /* [3] Common init: master enable, mask all, then enable all */
    TSB_DMAA->CFG = DMAxCfg_Master_enable;
    TSB_DMAA->CHNLREQMASKSET = DMAxChnlReqMaskSet_MASK;
    TSB_DMAA->CHNLENABLESET = DMAxChnlEnableSet_MASK;

    /* [4] Force burst for ADC channels. Channels 16 and 18 have no single
     *     transfer request, only burst. PINFO-M4K(2) Table 2.28. */
    TSB_DMAA->CHNLUSEBURSTSET = (DMA_CHANNEL_MASK(DMA_ADASGL_DMAREQ) |
                                 DMA_CHANNEL_MASK(DMA_ADCSGL_DMAREQ));
}

/* ==========================================================================
 *   ADC Channel Setup
 * ========================================================================== */

/**
 * @brief  Re-arm DMA channels 16 and 18 for one ADC conversion pair.
 * @details
 *   Called before every ADC sample. The controller clears cycle_ctrl to 0b000
 *   and drops the channel enable bit at the end of each cycle, so both are
 *   restored here.
 *
 *   Transfer configuration:
 *   - Source: ADC result registers REG0 and REG1, 4 bytes apart, 16-bit reads.
 *   - Dest:   uint16_t buffer, 2 bytes apart, 16-bit writes.
 *   - Mode:   continuous normal, 2 transfers, arbitrate after each transfer.
 *
 *   End-pointer arithmetic (DMAC-B RM 3.2.2.1):
 *   - src_end = &REG1, src_inc = +4:
 *       xfer 1: &REG1 - (1 × 4) = &REG0
 *       xfer 2: &REG1 - (0 × 4) = &REG1
 *   - dst_end = &buffer[1], dst_inc = +2:
 *       xfer 1: &buffer[1] - (1 × 2) = &buffer[0]
 *       xfer 2: &buffer[1] - (0 × 2) = &buffer[1]
 */
void DMA_SetupForADC(void)
{
    const uint32_t ch_mask = DMA_CHANNEL_MASK(DMA_ADASGL_DMAREQ) |
                             DMA_CHANNEL_MASK(DMA_ADCSGL_DMAREQ);

    /* Route the ADC single-conversion requests to channels 16 and 18, enable
     * edge detection, and open the trigger outputs. Written here rather than in
     * DMAC_Init() because TSEL16 and TSEL18 run off the ADC unit clock gates,
     * which ADC_Init() turns on. The register also carries channels 17 and 19,
     * which this project does not use. */
    TSB_TSEL0->CR0 = TSEL_ADC_DMA_ROUTE;

    /* Hold off requests while the descriptors are rewritten */
    TSB_DMAA->CHNLREQMASKSET = ch_mask;

    /* Common ChnlCfg for both ADC units */
    uint32_t chnl_cfg = DMA_DST_INC_HWORD      /* dest: +2 bytes per transfer */
                      | DMA_DST_SIZE_HWORD     /* dest: 16-bit writes */
                      | DMA_SRC_INC_WORD       /* src: +4 bytes (REG0 → REG1) */
                      | DMA_SRC_SIZE_HWORD     /* src: 16-bit reads (lower half of 32-bit reg) */
                      | DMA_R_POWER_1          /* arbitrate after each transfer */
                      | DMA_N_MINUS_1(2)       /* 2 transfers total */
                      | DMA_CYCLE_CTRL_CNT;    /* cycle_ctrl = 010: continuous normal */

    /* Channel 16: ADC Unit A → adc_a_buffer */
    DMA_ConfigChannel(DMA_ADASGL_DMAREQ,
                      (uint32_t)&TSB_ADA->REG1,
                      (uint32_t)&adc_a_buffer[1],
                      chnl_cfg);

    /* Channel 18: ADC Unit C → adc_c_buffer */
    DMA_ConfigChannel(DMA_ADCSGL_DMAREQ,
                      (uint32_t)&TSB_ADC->REG1,
                      (uint32_t)&adc_c_buffer[1],
                      chnl_cfg);

    /* Primary descriptor, release the mask, then enable. The controller clears
     * the enable bit at the end of every cycle, so it is set again each time.
     * DMAC-B RM 5.1 puts the enable last. */
    TSB_DMAA->CHNLPRIALTCLR  = ch_mask;
    TSB_DMAA->CHNLREQMASKCLR = ch_mask;
    TSB_DMAA->CHNLENABLESET  = ch_mask;
}

/* ==========================================================================
 *   Low-Level Channel Configuration
 * ========================================================================== */

/**
 * @brief  Write a single channel's primary control data.
 * @param  channel   DMA channel number (0-31).
 * @param  src_end   Source end address (used for end-pointer arithmetic).
 * @param  dst_end   Destination end address.
 * @param  chnl_cfg  32-bit DMAChnlCfg word.
 * @note   Does not write the RESERVED word at offset +12.
 */
void DMA_ConfigChannel(uint8_t channel, uint32_t src_end, uint32_t dst_end, uint32_t chnl_cfg)
{
    if (channel >= DMA_CHANNEL_COUNT)
        return;


    DMA_CtrlTable[channel].SrcEndPtr = src_end;
    DMA_CtrlTable[channel].DstEndPtr = dst_end;
    DMA_CtrlTable[channel].ChnlCfg   = chnl_cfg;
}

/* ==========================================================================
 *   Transfer Status
 * ========================================================================== */

/**
 * @brief  Report whether both ADC channels have finished their transfer cycle.
 * @return true once cycle_ctrl has returned to 0b000 on channels 16 and 18.
 * @note   The controller writes cycle_ctrl back into the control table when a
 *         cycle completes, so this reads RAM rather than a peripheral register.
 */
bool DMA_ADCTransferDone(void)
{
    return ((DMA_CtrlTable[DMA_ADASGL_DMAREQ].ChnlCfg & DMA_CYCLE_CTRL_MASK) == 0U) &&
           ((DMA_CtrlTable[DMA_ADCSGL_DMAREQ].ChnlCfg & DMA_CYCLE_CTRL_MASK) == 0U);
}

/* ==========================================================================
 *   Buffer Accessors
 * ========================================================================== */

/**
 * @brief  Get pointer to ADC Unit A DMA result buffer.
 * @return volatile uint16_t*  Layout: [0]=AINA16, [1]=AINA15, raw ADxREGn format.
 */
volatile uint16_t* DMA_GetADCABuffer(void)
{
    return adc_a_buffer;
}

/**
 * @brief  Get pointer to ADC Unit C DMA result buffer.
 * @return volatile uint16_t*  Layout: [0]=AINC01, [1]=AINC00, raw ADxREGn format.
 */
volatile uint16_t* DMA_GetADCCBuffer(void)
{
    return adc_c_buffer;
}