/**
 * @file        uart.h
 * @brief       Asynchronous Serial Communication driver (UART-C) Header file
 * @version     V1.0.0
 * @date        09-07-2026
 *
 * @details
 *   UART driver for Pins PC0 & PC1

 *   Alias:
 *   UART Channel 0 - Port C
 *      UT0RXD:  PC0 - RX 
 *      UT0TXDA: PC1 - TX
 *   
 *   Target/Goal: @ 10Mhz
 *   BAUD Rate: 115200 Bd (baud)
 *   Data Length: 8 bits
 *   Parity: N/A
 *   Stop bit: 1-bit
 * 
 *   Baud divider (UART-C RM Section 5, N + (64-K)/64 division, KEN=1):
 *       Baud = ΦT0 / (16 * (N + (64 - K)/64))
 *
 *   Computed for ΦT0 = 10 MHz, <PRSEL> = 0000 (1/1), <KEN> = 1:
 *   bps            <BRK>           <BRN>           Calculated Value (bps)
 *   115200         0x25 (K=37)     0x05 (N=5)              115274   (+0.06%)
 *
 *   Alt, if 5 MHz (core MCKSEL = ΦT0/2):
 *   115200         0x12 (K=18)     0x02 (N=2)              114943   (-0.22%)
 * 
 *   Reference:
 *   - Product Info:  https://toshiba.semicon-storage.com/info/TXZP-PINFO-M4K(2)_en_20231225.pdf?did=70854
 *   - UART-C RM:     https://toshiba.semicon-storage.com/info/RM-UART-C_en_20240510.pdf?did=158673
 *   - I/O Ports RM:  https://toshiba.semicon-storage.com/info/TXZP-PORT-M4K(2)_en_20250620.pdf?did=70850
 *    
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#ifndef UART_H
#define UART_H

#include "TMPM4KyA.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif


/* ==========================================================================
 *   Clock Gating (CG / FSYSMENA)
 * ========================================================================== */
#define CG_FSYSMENA_IPMENA21                        ((uint32_t)0x01 << 21U)     /*!< Clock enable of UART ch0 (TSEL21) */

/* ==========================================================================
 *   [UARTxSWRST] (Software Reset Register)
 * ========================================================================== */
#define UARTx_SWRST_SWRSTF                          ((uint32_t)0x01 << 7U )     /*!< Software reset flag. 0: Not asserted 1 | 1 : Asserted */

/* ==========================================================================
 *   UART Clock & Presaler
 * ========================================================================== */
#define UARTx_CLK_PRSEL_MASK                        ((uint32_t)0x0F << 4U )     /*!< Prescaler dividing ratio selection mask. PRSEL[3:0] */ 

/* ==========================================================================
 *   [UARTxCR0] (Control Register0)
 * ========================================================================== */
#define CR0_SM_POS                                  0U

#define UARTx_CR0_SM_MASK                           ((uint32_t)0x03 << 0U )     /*!< Data length - SM[1:0] mask */
#define UARTx_CR0_PE_MASK                           ((uint32_t)0x01 << 2U )     /*!< Parity addition - Enable/Disable Mask */
#define UARTx_CR0_EVEN_MASK                         ((uint32_t)0x01 << 3U )     /*!< Even parity selection mask  */
#define UARTx_CR0_SBLEN_MASK                        ((uint32_t)0x01 << 4U )     /*!< STOP bit length mask  */
#define UARTx_CR0_DIR_MASK                          ((uint32_t)0x01 << 5U )     /*!< Data transfer order. 0: LSB first | 1: MSB first  */

/* ==========================================================================
 *   [UARTxBRD] (Baud Rate Register)
 * ========================================================================== */
#define BRD_BRK_POS                                 16U
#define BRD_BRN_POS                                 0U

#define UARTx_BRD_KEN_MASK                          ((uint32_t)0x01 << 23U )            /*!< N + (64 - K) /64 dividing control Enable/Disable */
#define UARTx_BRD_BRK_MASK                          ((uint32_t)0x3F << BRD_BRK_POS )    /* K value setting of the N + (64 - K) /64 dividing - Decimal */
#define UARTx_BRD_BRN_MASK                          ((uint32_t)0xFFFF << BRD_BRN_POS)   /* N value setting of the N + (64 - K) /64 dividing or N dividing - Integer */


/* ==========================================================================
 *   [UARTxTRANS] (Transfer Enable Register) 
 * ========================================================================== */
#define UARTx_TRANS_RXE_MASK                        ((uint32_t)0x01 << 0U )            /*!< Reception control. 0: Disabled | 1: Enabled */
#define UARTx_TRANS_TXE_MASK                        ((uint32_t)0x01 << 1U )            /*!< Transmission control. 0: Disabled | 1: Enabled */
#define UARTx_TRANS_TXTRG_MASK                      ((uint32_t)0x01 << 2U )            /*!< Trigger transmission control. 0: Disabled | 1: Enabled */
#define UARTx_TRANS_BK_MASK                         ((uint32_t)0x01 << 3U )            /*!< Break transmission */

/* ==========================================================================
 *   [UARTxDR] (Data Register)
 * ========================================================================== */
#define UARTx_DR_DR_MASK                            ((uint32_t)0x1FF << 0U )           /*!< Reception/Transmission Data */
#define UARTx_DR_BERR_MASK                          ((uint32_t)0x01 << 16U )           /*!< Break error flag */
#define UARTx_DR_FERR_MASK                          ((uint32_t)0x01 << 17U )           /*!< Framing error */
#define UARTx_DR_PERR_MASK                          ((uint32_t)0x01 << 18U )           /*!< Parity error */

/* ==========================================================================
 *   [UARTxSR] (Status Register)
 * ========================================================================== */
#define SR_TLVL_POS                                 8U

#define UARTx_SR_SUE                                ((uint32_t)0x01 << 31U)     /*!< Setting enable status flag. 0: Setting is enabled | 1: disabled */
#define UARTx_SR_TXRUN_MASK                         ((uint32_t)0x01 << 15U)     /*!< Transmission operating flag. 0: Stop | 1: Operating */
#define UARTx_SR_TXEND_MASK                         ((uint32_t)0x01 << 14U)     /*!< Transmission completion flag. R: Complete | W: Flag Clear */
#define UARTx_SR_TLVL_MASK                          ((uint32_t)0x0F << SR_TLVL_POS)     /*!< Transmit FIFO data storage level. This field shows the current stage count of the data stored in the transmit FIFO.*/


/* ==========================================================================
 *   Function Prototypes
 * ========================================================================== */
void UART_Init(void);
void UART_SendByte(uint8_t byte);
void UART_SendUint(uint16_t val);
void UART_SendInt(int32_t val);
void UART_SendString(const char *str);
void UART_Flush(void);
void UART_CRLF(void);

#ifdef __cplusplus
}
#endif
#endif /* UART_H */