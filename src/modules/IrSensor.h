/**
 * @file        IrSensor.h
 * @brief       IR Sensor driver for TMPM4Ky micromouse.
 * @version     V1.0.0
 * @date        11-06-2026
 *
 * @details
 *   Provides sequenced IR emitter control and ADC sampling with ambient
 *   light cancellation. Uses DMA burst transfer from ADC Units A and C.
 *
 *   Pin Map:
 *   - Emitters (GPIO):
 *     - PU1 = Far Left IR Emitter, PU0 = Left IR Emitter  (Left side)
 *     - PG5 = Right IR Emitter, PG4 = Far Right IR Emitter (Right side)
 *   - Receivers (ADC):
 *     - Unit A: PL0 (AINA16) = Far Left, PL1 (AINA15) = Left
 *     - Unit C: PJ0 (AINC00) = Far Right, PJ1 (AINC01) = Right
 *
 *   Sampling Sequence:
 *   1. Turn all emitters OFF, wait for decay
 *   2. Trigger ADC → DMA stores ambient levels
 *   3. Turn all emitters ON, wait for propagation
 *   4. Trigger ADC → DMA stores raw (ambient + reflected)
 *   5. Subtract: reflected = raw - ambient
 *
 *   Reference Documents (Toshiba):
 *   - Product Info:  https://toshiba.semicon-storage.com/info/TXZP-PINFO-M4K(2)_en_20231225.pdf?did=70854
 *   - ADC-I RM:     https://toshiba.semicon-storage.com/info/RM-ADC-I_en_20251205.pdf?did=166835
 *   - PORT-M4K(2):  https://toshiba.semicon-storage.com/info/TXZP-PORT-M4K(2)_en_20250620.pdf?did=70850
 *
 * @note
 *   DMAC must be initialized before calling @ref IR_Init().
 *   Call @ref IR_SampleAll() from a periodic control tick (e.g., 1 ms).
 *
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#ifndef IRSENSOR_H
#define IRSENSOR_H

#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 *   Channel Enumeration
 * ========================================================================== */

/**
 * @brief  IR sensor channel indices, matching DMA buffer layout.
 */
typedef enum
{
    IR_FAR_LEFT  = 0,   /*!< AINA16 @ PL0 — Far Left receiver */
    IR_LEFT      = 1,   /*!< AINA15 @ PL1 — Left receiver */
    IR_RIGHT     = 2,   /*!< AINC01 @ PJ1 — Right receiver */
    IR_FAR_RIGHT = 3,   /*!< AINC00 @ PJ0 — Far Right receiver */
    IR_COUNT     = 4    /* Number of IR Sensor Pairs */
} ir_channel_t;


/* ==========================================================================
 *   Sensor Data Structure
 * ========================================================================== */

/**
 * @brief  Consolidated IR sensor data from one sampling cycle.
 */
typedef struct
{
    uint16_t raw[IR_COUNT];         /*!< Latest ADC reading (emitter ON) */
    uint16_t ambient[IR_COUNT];     /*!< Background light (emitter OFF) */
    uint16_t reflected[IR_COUNT];   /*!< True wall reflection (raw - ambient) */
    uint16_t filtered[IR_COUNT];    /*!< IIR-filtered reflected value */
    uint16_t distance_mm[IR_COUNT]; /*!< Distance for Each IR Sensor*/
    bool     wallDetected[IR_COUNT];
} ir_sensordata_t;


/* ==========================================================================
 *   IIR Filter Configuration
 * ========================================================================== */

/**
 * @brief  IIR filter shift value — alpha = 1 / 2^IR_FILTER_SHIFT.
 * @note   Shift = 3 → alpha = 0.125, ~8 sample time constant at 1 kHz.
 */
#define IR_FILTER_SHIFT     3U

/* ==========================================================================
 *   Distance Calibration
 * ========================================================================== */

/**
 * @brief  Number of measured calibration points.
 * @note   
 */
#define IR_CAL_POINTS       6U

/**
 * @brief  Distance sentinels (mm).
 */
#define IR_DIST_NO_WALL     255U    /*!< No wall detected / out of range */
#define IR_DIST_TOO_CLOSE   30U     /*!< Below reliable minimum */

/* ==========================================================================
 *   Wall Detection Thresholds (mm)
 * ========================================================================== */

/**
 * @brief  Wall presence threshold.
 * @note   Typical maze half-cell is ~90 mm.
 */
#define IR_WALL_THRESHOLD_MM    80U

/**
 * @brief  Hysteresis band to prevent flicker.
 */
#define IR_WALL_HYSTERESIS_MM   15U

/* ==========================================================================
 *   Function Prototypes
 * ========================================================================== */

/**
 * @brief  Initializes ADC and GPIO for IR emitters.
 * @note   DMAC must be initialized before this is called.
 */
void IR_Init(void);

/**
 * @brief  Full ON/OFF sampling cycle for all four sensors.
 * @note   Blocking — takes ~200 µs total. Call from control tick.
 */
void IR_SampleAll(void);

/* Data access */
const ir_sensordata_t* IR_GetData(void);
uint16_t IR_GetRaw(ir_channel_t ch);
uint16_t IR_GetReflected(ir_channel_t ch);
uint16_t IR_GetFiltered(ir_channel_t ch);
bool IR_IsWallDetected(ir_channel_t ch, uint16_t threshold);

/**
 * @brief  Get latest distance for a channel (mm).
 * @param  ch  Sensor channel.
 * @return uint16_t  Distance in mm, or IR_DIST_NO_WALL if none.
 */
uint16_t IR_GetDistanceMm(ir_channel_t ch);

/**
 * @brief  Check if wall is present with hysteresis.
 * @param  ch  Sensor channel.
 * @return bool  true if wall reliably detected.
 */
bool IR_IsWallPresent(ir_channel_t ch);

/* Emitter control */
void IR_AllEmittersOff(void);
void IR_AllEmittersOn(void);

/* Low-level emitter control (direct GPIO) */
void FarLeftEmitterOn(void);
void LeftEmitterOn(void);
void RightEmitterOn(void);
void FarRightEmitterOn(void);

void FarLeftEmitterOff(void);
void LeftEmitterOff(void);
void RightEmitterOff(void);
void FarRightEmitterOff(void);

void FarLeftEmitterToggle(void);
void LeftEmitterToggle(void);
void RightEmitterToggle(void);
void FarRightEmitterToggle(void);

/* IIR Filter */
uint16_t IR_FilterIIR(uint16_t prev, uint16_t curr, uint8_t shift);

#endif /* IRSENSOR_H */