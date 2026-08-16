/**
 * @file        Encoder.c
 * @brief       Encoder control module implementation.
 * @version     V1.0.0
 * @date        17-06-2026
 *
 * @details
 *   Reads A-ENC32 hardware counters, computes delta since last tick,
 *   applies IIR filtering on speed, and maintains a signed position.
 * 
 *   Motor Datasheet:
 *      https://www.pololu.com/file/0J1487/pololu-micro-metal-gearmotors-rev-6-2.pdf
 *
 * @note 
 *   File structure and Doxygen formatting assisted by AI.
 * 
 * Copyright (c) [Kevin Le] 2026
 */

#include "Encoder.h"
#include "drivers/encoder32A.h"

/* ==========================================================================
 *   PHYSICAL CALCULATIONS & UNIT CONVERSIONS
 * ==========================================================================
 *
 *   From Pololu Micro Metal Gearmotor Datasheet (Rev 6-2, Page 2):
 *
 *   "The two-channel Hall effect encoder senses the rotation of a 6-pole
 *    magnetic disc on a rear protrusion of the motor shaft, providing a
 *    resolution of 12 counts per revolution (CPR) of the motor shaft when
 *    counting both edges of both channels."
 *
 *   "To compute the gearbox output CPR, multiply 12 by the gearbox
 *    reduction factor."
 *
 *   Gearbox ratio: 30:1 (nominal)
 *   Exact gear ratio: 29.89:1 (from Page 3 gearbox table)
 *   Wheel CPR = 12 × 29.89 = 358.68 ≈ 359 counts per wheel revolution.
 *
 *   For simplicity in code constants, use 360 counts per wheel revolution.
 *
 *   Control Loop Tick: 1 kHz (1 ms period)
 *   - Encoder_Update() is called once per tick.
 *   - delta = counts detected in the last 1 ms.
 *   - speed_raw (CPS) = delta × 1000    (1 second = 1000 ms)
 *
 *   Example:
 *   - delta = 3 counts in 1 ms → CPS = 3000
 *   - 3000 CPS / 360 counts/rev = 8.33 wheel revolutions per second.
 */

/* ==========================================================================
 *   Private Data
 * ========================================================================== */

static encoder_t enc_state[2];


/* Count direction per motor, indexed by motor_t. */
static const int32_t enc_sign[2] =
{
    MOTOR_SIGN_LEFT,
    MOTOR_SIGN_RIGHT
};

/* ==========================================================================
 *   Private Helpers
 * ========================================================================== */

/**
 * @brief  Compute speed over a multi-tick window to beat count quantization.
 * @param  enc  Pointer to the encoder state.
 * @return int32_t  Speed in counts per second.
 * @details
 *   Differences the current position against the position SPEED_WINDOW_TICKS
 *   ticks ago. Counting over a longer window yields more counts per
 *   measurement, so the result resolves in finer steps than the 1-tick delta
 *   (which only ever sees 0 or 1 count at low speed).
 *   CPS = counts_in_window x (1000 / SPEED_WINDOW_TICKS).
 */
static int32_t Encoder_WindowedSpeed(encoder_t *enc)
{
    uint8_t oldest_head;
    int32_t counts_in_window;

    enc->history_head = (uint8_t)((enc->history_head + 1U) % (SPEED_WINDOW_TICKS + 1U));
    enc->position_history[enc->history_head] = enc->position;

    oldest_head = (uint8_t)((enc->history_head + 1U) % (SPEED_WINDOW_TICKS + 1U));
    counts_in_window = enc->position - enc->position_history[oldest_head];

    return counts_in_window * (1000 / SPEED_WINDOW_TICKS);
}


/* ==========================================================================
 *   Public Functions
 * ========================================================================== */


/**
 * @brief  Initialise encoder hardware and internal state.
 * @note   Must be called once before any other function.
 */
void Encoder_Init(void)
{
    /* Initialise the hardware drivers */
    ENC32A_Init();

    /* Clear all state */
    for (uint8_t i = 0; i < 2; i++)
    {
        enc_state[i].raw_count  = 0;
        enc_state[i].prev_count = 0;
        enc_state[i].delta      = 0;
        enc_state[i].position   = 0;
        enc_state[i].speed      = 0;
 
        /* Speed window */
        enc_state[i].history_head = 0;
        for (uint8_t j = 0; j <= SPEED_WINDOW_TICKS; j++)
            enc_state[i].position_history[j] = 0;
    }

    

    /* Read the current counter values to set prev_count */
    enc_state[MOTOR_LEFT].prev_count  = ENC0_ReadCount();
    enc_state[MOTOR_RIGHT].prev_count = ENC2_ReadCount();
    enc_state[MOTOR_LEFT].raw_count   = enc_state[MOTOR_LEFT].prev_count;
    enc_state[MOTOR_RIGHT].raw_count  = enc_state[MOTOR_RIGHT].prev_count;
}



/**
 * @brief  Read both encoders, update delta, position, and filtered speed.
 * @note   Call this exactly once per control tick (1 kHz).
 */
void Encoder_Update(void)
{
    int32_t raw, delta;
 
    /* Left motor */
    raw = ENC0_ReadCount();
    delta = raw - enc_state[MOTOR_LEFT].prev_count;
    enc_state[MOTOR_LEFT].raw_count = raw;
    enc_state[MOTOR_LEFT].delta = delta;
    enc_state[MOTOR_LEFT].position += delta;
    enc_state[MOTOR_LEFT].speed = Encoder_WindowedSpeed(&enc_state[MOTOR_LEFT]);
    enc_state[MOTOR_LEFT].prev_count = raw;
 
    /* Right motor */
    raw = ENC2_ReadCount();
    delta = raw - enc_state[MOTOR_RIGHT].prev_count;
    enc_state[MOTOR_RIGHT].raw_count = raw;
    enc_state[MOTOR_RIGHT].delta = delta;
    enc_state[MOTOR_RIGHT].position += delta;
    enc_state[MOTOR_RIGHT].speed = Encoder_WindowedSpeed(&enc_state[MOTOR_RIGHT]);
    enc_state[MOTOR_RIGHT].prev_count = raw;
}

/**
 * @brief  Get the filtered speed for a motor.
 * @param  motor  MOTOR_LEFT or MOTOR_RIGHT
 * @return int32_t  Speed in counts per second (signed, positive = forward).
 */
int32_t Encoder_GetSpeed_CPS(motor_t motor)
{
    if (motor != MOTOR_LEFT && motor != MOTOR_RIGHT)
        return 0;
    return enc_sign[motor] * enc_state[motor].speed;
}

/**
 * @brief  Get the raw delta (counts since last tick) for a motor.
 * @param  motor  MOTOR_LEFT or MOTOR_RIGHT
 * @return int32_t  Signed delta counts in one tick period.
 */
int32_t Encoder_GetDelta(motor_t motor)
{
    if (motor != MOTOR_LEFT && motor != MOTOR_RIGHT)
        return 0;
    return enc_sign[motor] * enc_state[motor].delta;
}

/**
 * @brief  Get the accumulated position for a motor.
 * @param  motor  MOTOR_LEFT or MOTOR_RIGHT
 * @return int32_t  Signed position in counts (increments on forward motion).
 */
int32_t Encoder_GetPosition(motor_t motor)
{
    if (motor != MOTOR_LEFT && motor != MOTOR_RIGHT)
        return 0;
    return enc_sign[motor] * enc_state[motor].position;
}

/**
 * @brief  Reset the position counter for a motor to zero.
 * @param  motor  MOTOR_LEFT or MOTOR_RIGHT
 */
void Encoder_ResetPosition(motor_t motor)
{
    if (motor != MOTOR_LEFT && motor != MOTOR_RIGHT)
        return;
    enc_state[motor].position = 0;
}

/**
 * @brief  Get the raw hardware count for a motor.
 * @param  motor  MOTOR_LEFT or MOTOR_RIGHT
 * @return int32_t  Current counter value from the encoder peripheral.
 */
int32_t Encoder_GetRawCount(motor_t motor)
{
    if (motor == MOTOR_LEFT)
        return ENC0_ReadCount();
    else if (motor == MOTOR_RIGHT)
        return ENC2_ReadCount();
    else
        return 0;
}