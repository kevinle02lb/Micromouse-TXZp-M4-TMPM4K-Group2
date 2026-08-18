/**
 * @file        IrSensor.c
 * @brief       IR Sensor driver implementation for TMPM4Ky micromouse.
 * @version     V1.0.0
 * @date        11-06-2026
 *
 * @details
 *   Sequenced emitter control and ADC sampling with ambient cancellation.
 *   Provides a blocking one-shot cycle (@ref IR_SampleAll) and a
 *   non-blocking per-tick sampler (@ref IR_SampleStep) for the control loop.
 *
 *   Pin Map:
 *   - Emitters (GPIO):
 *     - PU1 = Far Left IR Emitter, PU0 = Left IR Emitter  (Left side)
 *     - PG5 = Right IR Emitter, PG4 = Far Right IR Emitter (Right side)
 *   - Receivers (ADC):
 *     - Unit A: PL0 (AINA16) = Far Left, PL1 (AINA15) = Left
 *     - Unit C: PJ0 (AINC00) = Far Right, PJ1 (AINC01) = Right
 *
 *   DMA Buffer Layout:
 *   - adc_a_buffer[0] = AINA16 (Far Left)
 *   - adc_a_buffer[1] = AINA15 (Left)
 *   - adc_c_buffer[0] = AINC01 (Right)
 *   - adc_c_buffer[1] = AINC00 (Far Right)
 *
 *   Reference Documents (Toshiba):
 *   - ADC-I RM:     https://toshiba.semicon-storage.com/info/RM-ADC-I_en_20251205.pdf?did=166835
 *   - DMAC-B RM:    https://toshiba.semicon-storage.com/info/RM-DMAC-B_en_20241031.pdf?did=160537
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "IrSensor.h"
#include "drivers/adc.h"
#include "drivers/gpio.h"
#include "drivers/systick.h"

#ifdef ADC_USE_DMA
    #include "drivers/dma.h"
#endif

/* ==========================================================================
 *   Private Types
 * ========================================================================== */

/**
 * @brief  IR States for ADC Readings
 */
typedef enum {
    IR_PHASE_IDLE = 0,
    IR_PHASE_AMBIENT_SETTLE,   /* emitters off — let prior IR decay        */
    IR_PHASE_AMBIENT_READ,     /* read ADC -> ambient[]                    */
    IR_PHASE_EMITTER_ON,       /* turn emitters on, begin settle wait      */
    IR_PHASE_SETTLE,           /* hold on — let receiver RC settle (ticks) */
    IR_PHASE_READ,             /* read ADC -> raw[], emitters off          */
    IR_PHASE_COMPUTE           /* ambient-cancel, filter, walls            */
} ir_phase_t;

/* Reading-to-distance curve per channel, adc descending */
static const ir_cal_point_t ir_cal[IR_COUNT][IR_CAL_POINTS] =
{
    /* ADC, mm */
    /* IR_FAR_LEFT */
    { {2383U, 30U}, {1433U, 50U}, {962U, 70U}, {592U, 90U}, {424U, 120U}, {310U, 160U} },
    /* IR_LEFT */
    { {3286U, 30U}, {2117U, 50U}, {1366U, 70U}, {1039U, 90U}, {753U, 120U}, {568U, 160U} },
    /* IR_RIGHT */
    { {3344U, 30U}, {3125U, 50U}, {2019U, 70U}, {1514U, 90U}, {1030U, 120U}, {721U, 160U} },
    /* IR_FAR_RIGHT */
    { {3287U, 30U}, {2387U, 50U}, {1502U, 70U}, {1080U, 90U}, {659U, 120U}, {442U, 160U} }
};

/* ==========================================================================
 *   Private Function Prototypes
 * ========================================================================== */

static void IR_UpdateWalls(void);
static void IR_Acquire(uint16_t out[IR_COUNT]);

/* ==========================================================================
 *   Module Data
 * ========================================================================== */

static ir_sensordata_t ir_data;


/* ==========================================================================
 *   Non-Blocking Sampling State
 * ========================================================================== */

/**
 * @brief  Control ticks to hold emitters on before the reflected read.
 * @note   1 tick @ 1 kHz = 1 ms, comfortably past the ~500 µs receiver RC.
 *         Must be >= 1. Raise if you lengthen the RC or slow the tick.
 */
#define IR_SETTLE_TICKS     1U

static ir_phase_t ir_phase      = IR_PHASE_IDLE;
static uint16_t   ir_settle_cnt = 0U;


/* ==========================================================================
 *   Wall Detection State
 * ========================================================================== */

static bool wall_state[IR_COUNT] = {false, false, false, false};

/* ==========================================================================
 *   Initialization
 * ========================================================================== */

/**
 * @brief  Initializes ADC and GPIO for IR emitters.
 * @note   
 */
void IR_Init(void)
{
    #ifdef ADC_USE_DMA
        DMAC_Init();         /* DMAC-B must init before ADC DMA requests */
    #endif
    ADC_Init();
    PORT_U_Init();       /* [1] Left emitters  */
    PORT_G_Init();       /* [2] Right emitters */
}

/* ==========================================================================
 *   Private Funcs
 * ========================================================================== */

/**
 * @brief  Read one ADC sample set from all four receivers into out[].
 * @param  out  Channel-ordered destination [IR_FAR_LEFT..IR_FAR_RIGHT].
 */
static void IR_Acquire(uint16_t out[IR_COUNT])
{
#ifdef ADC_USE_DMA
    volatile uint16_t *bufA;
    volatile uint16_t *bufC;
 
    Start_ADC();                 /* Trigger both units, DMA moves results */
    SysTick_us(5U);              /* Wait for 2 conversions + DMA burst */
 
    bufA = DMA_GetADCABuffer();
    bufC = DMA_GetADCCBuffer();
 
    /* DMA stores the raw register halfword — right-align to match polled path */
    out[IR_FAR_LEFT]  = (uint16_t)((bufA[0] & ADxREGn_ADRn) >> 4U);   /* AINA16 */
    out[IR_LEFT]      = (uint16_t)((bufA[1] & ADxREGn_ADRn) >> 4U);   /* AINA15 */
    out[IR_RIGHT]     = (uint16_t)((bufC[0] & ADxREGn_ADRn) >> 4U);   /* AINC01 */
    out[IR_FAR_RIGHT] = (uint16_t)((bufC[1] & ADxREGn_ADRn) >> 4U);   /* AINC00 */
#else
    uint16_t a0, a1;
    uint16_t c0, c1;
 
    AINA_ReadPair(&a0, &a1);     /* a0 = AINA16, a1 = AINA15 */
    AINC_ReadPair(&c0, &c1);     /* c0 = AINC01, c1 = AINC00 */
 
    out[IR_FAR_LEFT]  = a0;
    out[IR_LEFT]      = a1;
    out[IR_RIGHT]     = c0;
    out[IR_FAR_RIGHT] = c1;
#endif
}

/**
 * @brief  Ambient-cancel raw[]/ambient[] into reflected[], then IIR filter.
 * @note   Receiver reads DOWN with light, so reflected = ambient - raw.
 */
static void IR_ComputeReflected(void)
{
    int i;
 
    /* [1] Calculate reflected values (ambient cancellation) */
    for (i = 0; i < IR_COUNT; i++)
    {
        if (ir_data.ambient[i] > ir_data.raw[i])
        {
            ir_data.reflected[i] = ir_data.ambient[i] - ir_data.raw[i];
        }
        else
        {
            ir_data.reflected[i] = 0U;
        }
    }
 
    /* [2] IIR low-pass filter on reflected values */
    for (i = 0; i < IR_COUNT; i++)
    {
        ir_data.filtered[i] = IR_FilterIIR(ir_data.filtered[i],
                                           ir_data.reflected[i],
                                           IR_FILTER_SHIFT);
    }
}

/**
 * @brief  Update wallDetected[] from measured distance with hysteresis.
 * @note   Thresholding in mm rather than counts lets one limit serve all four
 *         channels, whose raw sensitivities differ by more than 2:1.
 */
static void IR_UpdateWalls(void)
{
    float distance;

    for (int i = 0; i < IR_COUNT; i++)
    {
        distance = IR_GetDistance_mm((ir_channel_t)i);

        if (wall_state[i])
            wall_state[i] = (distance < (IR_WALL_DISTANCE_MM + IR_WALL_HYSTERESIS_MM));
        else
            wall_state[i] = (distance < (IR_WALL_DISTANCE_MM - IR_WALL_HYSTERESIS_MM));

        ir_data.wallDetected[i] = wall_state[i];
    }
}


/* ==========================================================================
 *   Sampling Functions
 * ========================================================================== */

/**
 * @brief  Full ON/OFF sampling cycle for all four IR sensors.
 * @details
 *   Performs ambient cancellation to reject background IR:
 *   1. Emitters OFF -> sample ambient light
 *   2. Emitters ON  -> sample raw (ambient + reflected)
 *   3. reflected = ambient - raw
 *
 *   Blocking — each ADC pair is polled to completion. Bench / calibration
 *   use; the control loop uses @ref IR_SampleStep.
 */
void IR_SampleAll(void)
{
    /* [1] Ambient reading — all emitters OFF */
    IR_AllEmittersOff();
    SysTick_us(20U);
    IR_Acquire(ir_data.ambient);
 
    /* [2] Reflected reading — all emitters ON */
    IR_AllEmittersOn();
    SysTick_us(600U);
    IR_Acquire(ir_data.raw);
 
    /* [3] Ambient cancel + IIR filter */
    IR_ComputeReflected();
 
    /* [4] Power save — turn off emitters between samples */
    IR_AllEmittersOff();
 
    /* [5] Update wall & flags from filtered data */
    IR_UpdateWalls();
}

/**
 * @brief  Non-blocking sampler — advances one phase per control tick.
 * @details
 *   Phase order, one tick each:
 *     AMBIENT_SETTLE -> AMBIENT_READ -> EMITTER_ON -> SETTLE(IR_SETTLE_TICKS)
 *     -> READ -> COMPUTE -> (loop)
 *   Emitters are on only across EMITTER_ON..READ. The RC settle elapses in
 *   the SETTLE phase without blocking the CPU.
 */
void IR_SampleStep(void)
{
    switch (ir_phase)
    {
    case IR_PHASE_IDLE:
    default:
        IR_AllEmittersOff();
        ir_phase = IR_PHASE_AMBIENT_SETTLE;
        break;
 
    case IR_PHASE_AMBIENT_SETTLE:
        /* [1] Emitters off — prior reflection decays this tick */
        ir_phase = IR_PHASE_AMBIENT_READ;
        break;
 
    case IR_PHASE_AMBIENT_READ:
        /* [2] Ambient reading — all emitters OFF */
        IR_Acquire(ir_data.ambient);
        ir_phase = IR_PHASE_EMITTER_ON;
        break;
 
    case IR_PHASE_EMITTER_ON:
        /* [3] Emitters ON — arm settle counter */
        IR_AllEmittersOn();
        ir_settle_cnt = IR_SETTLE_TICKS;
        ir_phase      = IR_PHASE_SETTLE;
        break;
 
    case IR_PHASE_SETTLE:
        /* [4] Hold on — receiver RC settles across ticks */
        if (--ir_settle_cnt == 0U)
        {
            ir_phase = IR_PHASE_READ;
        }
        break;
 
    case IR_PHASE_READ:
        /* [5] Reflected reading, then power save (Turn off emitters) */
        IR_Acquire(ir_data.raw);
        IR_AllEmittersOff();
        ir_phase = IR_PHASE_COMPUTE;
        break;
 
    case IR_PHASE_COMPUTE:
        /* [6] Ambient cancel + filter + walls */
        IR_ComputeReflected();
        IR_UpdateWalls();
        ir_phase   = IR_PHASE_AMBIENT_SETTLE;
        break;
    }
}

/* ==========================================================================
 *   Data Accessors
 * ========================================================================== */

/**
 * @brief  Get pointer to the latest IR sensor data structure.
 * @return const IR_SensorData*  Pointer to internal ir_data.
 */
const ir_sensordata_t* IR_GetData(void)
{
    return &ir_data;
}

/**
 * @brief  Get raw ADC value (emitter ON) for a specific channel.
 * @param  ch  Sensor channel to read.
 * @return uint16_t  Raw 12-bit ADC value, right-aligned.
 */
uint16_t IR_GetRaw(ir_channel_t ch)
{
    return ir_data.raw[ch];
}

/**
 * @brief  Get ambient-corrected reflected value for a specific channel.
 * @param  ch  Sensor channel to read.
 * @return uint16_t  Reflected IR intensity (0 if raw <= ambient).
 */
uint16_t IR_GetReflected(ir_channel_t ch)
{
    return ir_data.reflected[ch];
}

/**
 * @brief  Get filtered ADC value of a channel.
 * @param  ch  Sensor channel to read.
 * @return uint16_t  Filtered 12-bit ADC value, right-aligned.
 */
uint16_t IR_GetFiltered(ir_channel_t ch)
{
    return ir_data.filtered[ch];
}

/**
 * @brief  Convert a channel's filtered reading to a distance.
 * @param  ch  Sensor channel to read.
 * @return float  Distance in mm, clamped to the ends of the table.
 * @note   Table rows must be strictly descending in adc.
 */
float IR_GetDistance_mm(ir_channel_t ch)
{
    const ir_cal_point_t *cal = ir_cal[ch];
    uint16_t value = ir_data.filtered[ch];
    uint8_t  i;
    float    frac;

    if (value >= cal[0].adc)
        return (float)cal[0].mm;

    for (i = 0U; i < (IR_CAL_POINTS - 1U); i++)
    {
        if (value >= cal[i + 1U].adc)
        {
            frac = (float)(cal[i].adc - value) /
                   (float)(cal[i].adc - cal[i + 1U].adc);

            return (float)cal[i].mm +
                   (frac * (float)(cal[i + 1U].mm - cal[i].mm));
        }
    }

    return (float)cal[IR_CAL_POINTS - 1U].mm;
}

/**
 * @brief  Check if a wall is detected based on reflected threshold.
 * @param  ch         Sensor channel to check.
 * @param  threshold  Minimum reflected value to count as wall.
 * @return bool  true if reflected value >= threshold.
 */
bool IR_IsWallDetected(ir_channel_t ch, uint16_t threshold)
{
    return (ir_data.reflected[ch] >= threshold);
}


/**
 * @brief  Check if wall is present with hysteresis.
 * @param  ch  Sensor channel.
 * @return bool  true if wall reliably detected.
 */
bool IR_IsWallPresent(ir_channel_t ch)
{
    if (ch >= IR_COUNT) 
    {
        return false;
    }
    return wall_state[ch];
}


/* ==========================================================================
 *   Emitter Control
 * ========================================================================== */

/**
 * @brief  Turn OFF all four IR emitters simultaneously.
 */
void IR_AllEmittersOff(void)
{
    FarLeftEmitterOff();
    LeftEmitterOff();
    RightEmitterOff();
    FarRightEmitterOff();
}

/**
 * @brief  Turn ON all four IR emitters simultaneously.
 */
void IR_AllEmittersOn(void)
{
    FarLeftEmitterOn();
    LeftEmitterOn();
    RightEmitterOn();
    FarRightEmitterOn();
}

/* ==========================================================================
 *   Low-Level GPIO Emitter Control
 * ========================================================================== */

/**
 * @brief  Control functions to toggle data outpins for IR emitters.
 */

void FarLeftEmitterOn(void)     { GPIO_U_SetData((uint8_t)Px1_MASK); }
void LeftEmitterOn(void)        { GPIO_U_SetData((uint8_t)Px0_MASK); }
void RightEmitterOn(void)       { GPIO_G_SetData((uint8_t)Px5_MASK); }
void FarRightEmitterOn(void)    { GPIO_G_SetData((uint8_t)Px4_MASK); }

void FarLeftEmitterOff(void)    { GPIO_U_ClrData((uint8_t)Px1_MASK); }
void LeftEmitterOff(void)       { GPIO_U_ClrData((uint8_t)Px0_MASK); }
void RightEmitterOff(void)      { GPIO_G_ClrData((uint8_t)Px5_MASK); }
void FarRightEmitterOff(void)   { GPIO_G_ClrData((uint8_t)Px4_MASK); }

void FarLeftEmitterToggle(void) { GPIO_U_ToggleData((uint8_t)Px1_MASK); }
void LeftEmitterToggle(void)    { GPIO_U_ToggleData((uint8_t)Px0_MASK); }
void RightEmitterToggle(void)   { GPIO_G_ToggleData((uint8_t)Px5_MASK); }
void FarRightEmitterToggle(void){ GPIO_G_ToggleData((uint8_t)Px4_MASK); }

/* ==========================================================================
 *   IR Filtering from Spikes
 * ========================================================================== */

/**
 * @brief  Apply IIR low-pass filter to a single ADC channel.
 * @param  prev  Previous filtered value (y[n-1]).
 * @param  curr  New raw sample (x[n]).
 * @param  shift Right-shift for alpha divisor (e.g., 3 = /8).
 * @return uint16_t  Filtered output y[n].
 * @details
 *   Exponential moving average with power-of-two alpha:
 *     y[n] = y[n-1] + alpha * (x[n] - y[n-1])
 *          = y[n-1] + (x[n] - y[n-1]) >> shift
 *
 *   Equivalent to: y[n] = ((2^shift - 1) * y[n-1] + x[n]) / 2^shift
 *
 *   Shift values:
 *   - 2: alpha = 0.25, fast response, light smoothing
 *   - 3: alpha = 0.125, balanced (default)
 *   - 4: alpha = 0.0625, slow response, heavy smoothing
 *
 *   No saturation needed — uint16_t in, uint16_t out, difference
 *   computed in unsigned arithmetic with branch on direction.
 */
uint16_t IR_FilterIIR(uint16_t prev, uint16_t curr, uint8_t shift)
{
    uint16_t delta;
    uint16_t result;

    if (curr > prev)
    {
        delta = curr - prev;
        result = prev + (delta >> shift);
    }
    else
    {
        delta = prev - curr;
        result = prev - (delta >> shift);
    }

    return result;
}