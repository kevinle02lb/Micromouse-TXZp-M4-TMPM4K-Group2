/**
 * @file        uart.c
 * @brief       Asynchronous Serial Communication driver (UART-C)
 * @version     V1.0.0
 * @date        09-07-2026
 *
 * @details
 *   UART driver for Pins PC0 & PC1
 * 
 *   Handles input Clock of 80Mhz
 *   
 *   Alias:
 *   UART Channel 0 - Port C
 *      UT0RXD:  PC0 - RX 
 *      UT0TXDA: PC1 - TX
 *   
 *   Target/Goal: @ 80Mhz
 *   BAUD Rate: 115200 Bd (baud)
 *   Data Length: 8 bits
 *   Parity: N/A
 *   Stop bit: 1-bit
 * 
 *   Check Section 5. Usage Example
 *      5.1. Baud Rate Setting Value:
 *          Table 5.2 Setting Example at ΦTx =80MHz, <PRSEL> =0000, and <KEN> =1
 *   bps            <BRK>           <BRN>           Calculated Value (bps)
 *   115200         0x26            0x02B                   115191
 *   115200         0x27            0x02B                   115232
 * 
 *          Computed for ΦTx = fsysm = 10 MHz, <PRSEL> = 0000 (1/1), <KEN> = 1:
 *   bps            <BRK>           <BRN>           Calculated Value (bps)
 *   115200         0x25 (K=37)     0x05 (N=5)              115274   (+0.06%)
 *
 *   Alt, if fsysm = 5 MHz (core MCKSEL = fsysh/2):
 *   115200         0x12 (K=18)     0x02 (N=2)              114943   (-0.22%)
 * 
 *   
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

/* ==========================================================================
 *   Includes/Defines
 * ========================================================================== */
#include "uart.h"
#include "gpio.h"
#include <stddef.h>

#define UARTx_BRD_BRN               0x2B        /* N = 43 */
#define UARTx_BRD_BRK               0x26        /* K = 38 */

#define UARTx_CR0_SM_8BIT           0x01        /* SM[1:0] = 01: 8-bit data length */
#define UART_TX_FIFO_STAGES         8U          /* Transmit FIFO depth in stages (UART-C RM, Section 3.4.2) */

#define ASCII_CR                    0x0D
#define ASCII_LF                    0x0A


/* ==========================================================================
 *   Private Function/Helpers
 * ========================================================================== */
/**
 * @brief  Configures PC0/PC1 for the UART channel 0 function.
 * @details
 *   I/O Ports RM, Section 4.2.4 (PORT C): both pins use the FR2 function.
 *   - PC0 (UT0RXD):  function input  -> CR = 0, IE = 1
 *   - PC1 (UT0TXDA): function output -> CR = 1, IE = 0
 */
static void UART_PORT_C_Init(void)
{
    /* PC0 - UT0RXD */
    TSB_PC->IE |= Px0_MASK;
    TSB_PC->CR &= ~Px0_MASK;
    PC_FRn_Clear(Px0_MASK);
    TSB_PC->FR2 |= Px0_MASK;


    /* PC1 - UT0TXDA */
    PC_FRn_Clear(Px1_MASK);
    TSB_PC->FR2 |= Px1_MASK;
    TSB_PC->IE &= ~Px1_MASK;
    TSB_PC->CR |= Px1_MASK;
}

/* ==========================================================================
 *   Initialization
 * ========================================================================== */
/**
 * @brief  Initializes UART channel 0 for 115200 Bd 8-N-1 transmission.
 * @details
 *   Sequence:
 *   1. Enable clock supply to Port C and UART ch0 (CG).
 *   2. Route PC0/PC1 to the UART function (I/O Ports RM, Section 4.2.4).
 *   3. Wait for [UARTxSR]<SUE> = 0. [UARTxCR0], [UARTxCR1], [UARTxCLK],
 *      [UARTxBRD] and [UARTxFIFOCLR] are only writable while <SUE> = 0,
 *      i.e. while neither transmission nor reception is in progress
 *      (UART-C RM, Section 4.2.2).
 *   4. Select the 1/1 prescaler
 *      ([UARTxCLK]<PRSEL> = 0000, UART-C RM, Section 4.2.4).
 *   5. Set the frame format Ex: 8-bit, no parity, 1 STOP bit, LSB first
 *      ([UARTxCR0]<SM> = 01, UART-C RM, Section 4.2.2).
 *   6. Program the baud rate divider: KEN = 1, N = 43, K = 38
 *      ([UARTxBRD], UART-C RM, Section 4.2.5 and Table 5.2).
 *   7. Enable transmission ([UARTxTRANS]<TXE> = 1, UART-C RM, Section 4.2.6).
 *
 * @note   Reception is not enabled. Set [UARTxTRANS]<RXE> and configure
 *         the receive path if needed.
 */
void UART_Init(void)
{
    /* [1] Enable clock supply: Port C + UART ch0 */
    TSB_CG->FSYSMENA |= CG_PORTC;                       /* Port C Clk En */
    TSB_CG->FSYSMENA |= CG_FSYSMENA_IPMENA21;           /* UART CH0 Clk En */

    /* [2] Route PC0 (UT0RXD) / PC1 (UT0TXDA) to the UART function & Reset UART */
    UART_PORT_C_Init();
    TSB_UART0->SWRST = 0x02U;                            /* Software Reset */
    TSB_UART0->SWRST = 0x01U;
    while ((TSB_UART0->SWRST & UARTx_SWRST_SWRSTF) != 0U) { ; }  /* wait SWRSTF=0 */

    /* [3] Wait until the setting registers are writable (<SUE> = 0) */
    while ( (TSB_UART0->SR & UARTx_SR_SUE) != 0U ) { ; }

    /* [4] Prescaler: 1/1 */
    TSB_UART0->CLK &= ~UARTx_CLK_PRSEL_MASK;

    /* [5] Frame format: 8-bit, no parity, 1 STOP bit, LSB first */
    TSB_UART0->CR0 = UARTx_CR0_SM_8BIT << CR0_SM_POS;

    /* [6] Baud rate: KEN = 1, N = 43, K = 38 -> 115191 Bd */
    TSB_UART0->BRD = UARTx_BRD_KEN_MASK | (UARTx_BRD_BRN << BRD_BRN_POS) | (UARTx_BRD_BRK << BRD_BRK_POS);

    /* [7] Enable transmission (normal operation, no trigger) */
    TSB_UART0->TRANS |= UARTx_TRANS_TXE_MASK;
}



/* ==========================================================================
 *   Public Functions
 * ========================================================================== */
/**
 * @brief   Transmits one byte over UART (blocking).
 * @param   byte  8-bit value to transmit.
 * @details Waits until the 8-stage transmit FIFO has a free slot
 *          ([UARTxSR]<TLVL> < 8, UART-C RM, Section 3.4.2), then writes
 *          the byte to [UARTxDR]. In 8-bit mode ([UARTxCR0]<SM> = 01)
 *          only DR[7:0] carry data; hardware frames the byte as START,
 *          8 data bits LSB-first, and 1 STOP bit. Transmission starts
 *          automatically once data is present in the FIFO (Section 3.6.1).
 * @note    Blocks while the FIFO is full. One frame takes ~87 µs at
 *          115200 Bd; call UART_Flush() to also wait for the last frame
 *          to leave the shift register.
 */
void UART_SendByte(uint8_t byte)
{
    while( ( (TSB_UART0->SR & UARTx_SR_TLVL_MASK) >> SR_TLVL_POS ) >= UART_TX_FIFO_STAGES ) { ; }
    TSB_UART0->DR = (uint32_t)byte;
}

/**
 * @brief   Transmits an unsigned 16-bit value as decimal ASCII (blocking).
 * @param   val  Value to transmit (0-65535).
 * @details Converts @p val to decimal digits with no leading zeros and
 *          sends them through UART_SendString(). For example, 1234 is
 *          sent as "1234".
 * @note    Append "\r\n" to the string if a new line is required.
 */
void UART_SendUint(uint16_t val)
{
    char buf[6];                        /* Max "65535" + NUL terminator */
    int i = (int)(sizeof(buf) - 1U);
    buf[i] = '\0';

    if ( val == 0U )
    {
        UART_SendByte('0');
        return;
    }

    /* Extract digits backwards, least significant first */
    while ( val > 0U )
    {
        buf[--i] = (char)('0' + (val % 10U));
        val /= 10U;
    }

    UART_SendString(&buf[i]);
}

/**
 * @brief   Transmits a signed value as decimal ASCII text (blocking).
 * @param   val  Value to print; a leading '-' is emitted when negative.
 * @details Emits the sign, then prints the magnitude via UART_SendUint().
 *          Takes a 32-bit argument so that negating the most-negative value
 *          cannot overflow.
 * @note    The magnitude must fit UART_SendUint()'s type — widen that formatter
 *          before printing counts beyond its range.
 */
void UART_SendInt(int32_t val)
{
    if (val < 0)
    {
        UART_SendByte('-');
        val = -val;                 /* Turn to Magnitude */
    }
    UART_SendUint((uint16_t)val);   /* Cast it to 16 bit */
}

/**
 * @brief   Transmits a NUL-terminated string (blocking).
 * @param   str  Pointer to the string to transmit.
 * @details Sends each character through UART_SendByte() up to, but not
 *          including, the NUL terminator. Does nothing if @p str is NULL.
 * @note    Append "\r\n" to the string if a new line is required.
 */
void UART_SendString(const char *str)
{
    if ( str == NULL )
    {
        return;
    }

    while ( *str != '\0' )
    {
        UART_SendByte((uint8_t)*str++);
    }
}

/**
 * @brief   Waits until all queued data has been fully transmitted (blocking).
 * @details Polls [UARTxSR]<TXRUN>: the flag stays set while data remains
 *          in the transmit FIFO or in the transmission shift register, and
 *          clears once the last STOP bit has been sent
 *          (UART-C RM, Section 4.2.8).
 * @note    Use before re-configuring the UART or entering a low-power
 *          state so the final frame is not truncated.
 */
void UART_Flush(void)
{
    while ( (TSB_UART0->SR & UARTx_SR_TXRUN_MASK) != 0U ) { ; }
}

/**
 * @brief   Transmits a CR+LF line ending (blocking).
 * @details Sends carriage return then line feed so terminals that don't
 *          auto-translate return the cursor to column 0 and advance a line.
 */
void UART_CRLF(void)
{
    UART_SendByte(ASCII_CR);
    UART_SendByte(ASCII_LF);
}

