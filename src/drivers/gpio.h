/**
 * @file        gpio.h
 * @brief       GPIO driver for TMPM4Ky microcontrollers.
 * @version     V1.0.0
 * @date        11-05-2026
 *
 * @details
 *   Pin masks, clock gating, and port init prototypes.
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#ifndef GPIO_H
#define GPIO_H

#include "TMPM4KyA.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif



/* ==========================================================================
 *   Pin Masks
 * ========================================================================== */
#define Px0_MASK    ((uint32_t)0x01 << 0U)
#define Px1_MASK    ((uint32_t)0x01 << 1U)
#define Px2_MASK    ((uint32_t)0x01 << 2U)
#define Px3_MASK    ((uint32_t)0x01 << 3U)
#define Px4_MASK    ((uint32_t)0x01 << 4U)
#define Px5_MASK    ((uint32_t)0x01 << 5U)

/* ==========================================================================
 *   Clock Gating (CG FSYSMENA)
 * ========================================================================== */
#define CG_PORTA    ((uint32_t)0x01 << 0U)
#define CG_PORTC    ((uint32_t)0x01 << 2U)
#define CG_PORTD    ((uint32_t)0x01 << 3U)
#define CG_PORTG    ((uint32_t)0x01 << 6U)
#define CG_PORTJ    ((uint32_t)0x01 << 8U)
#define CG_PORTL    ((uint32_t)0x01 << 10U)
#define CG_PORTN    ((uint32_t)0x01 << 12U)
#define CG_PORTU    ((uint32_t)0x01 << 16U)

/* ==========================================================================
 *   Function Prototypes
 * ========================================================================== */
void GPIO_Init(void);
void PORT_A_Init(void);
void PORT_C_Init(void);
void PORT_D_Init(void);
void PORT_G_Init(void);
void PORT_J_Init(void);
void PORT_L_Init(void);
void PORT_N_Init(void);
void PORT_U_Init(void);

void PA_FRn_Clear(uint32_t pin_mask);
void PC_FRn_Clear(uint32_t pin_mask);
void PD_FRn_Clear(uint32_t pin_mask);
void PG_FRn_Clear(uint32_t pin_mask);
void PN_FRn_Clear(uint32_t pin_mask);
void PU_FRn_Clear(uint32_t pin_mask);

void GPIO_G_SetData(uint8_t data);
void GPIO_U_SetData(uint8_t data);
void GPIO_G_ClrData(uint8_t data);
void GPIO_U_ClrData(uint8_t data);
void GPIO_G_ToggleData(uint8_t data);
void GPIO_U_ToggleData(uint8_t data);

#ifdef __cplusplus
}
#endif
#endif /* GPIO_H */