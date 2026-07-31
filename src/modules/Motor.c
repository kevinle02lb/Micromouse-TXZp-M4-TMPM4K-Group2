/**
 * @file        Motor.c
 * @brief       Motor control implementation for TB67H450AFNG (slow-decay)
 * @version     V2.0.0
 * @date        26-07-2026
 *
 * @details
 *   High-level motor control. See motor.h for wiring and truth table.
 *
 *   Direction-change policy (RM-T32A-C 4.6.1):
 *   OUTCRA1 / OUTCRB1 are writable only while RUNA = 0. A direction
 *   change therefore stops the channel, rewrites output control, and
 *   restarts. Duty-only updates (RGx0) are double-buffered and may be
 *   written while the channel runs.
 *
 *     +-------------------------+-------------------------------+
 *     |  Scenario               |  Action                       |
 *     +-------------------------+-------------------------------+
 *     |  Same dir, moving       |  Live duty write only         |
 *     |  Same dir, BRAKE/STOP   |  No-op                        |
 *     |  Any direction change   |  Stop -> reconfigure -> Start |
 *     +-------------------------+-------------------------------+
 *
 *   Drive scheme: SLOW DECAY (IN1 = TimerA/OUTA, IN2 = TimerB/OUTB).
 *   The PWM off-phase is BRAKE (both inputs high), not coast. Winding
 *   current recirculates rather than collapsing each cycle, which
 *   raises low-duty torque and linearises speed-vs-duty relative to
 *   fast decay.
 *
 *   Direction labels denote robot motion. The pin mapping is inverted
 *   relative to the TB67H450 forward/reverse convention so that a
 *   FORWARD command produces forward wheel rotation for the installed
 *   motor wiring.
 *
 *     FORWARD : IN1 = PWM, IN2 = HIGH (static)
 *                 PWM low  -> IN1=L, IN2=H -> drive
 *                 PWM high -> IN1=H, IN2=H -> brake (recirculate)
 *     REVERSE : IN1 = HIGH (static), IN2 = PWM
 *                 PWM low  -> IN1=H, IN2=L -> drive
 *                 PWM high -> IN1=H, IN2=H -> brake (recirculate)
 *     BRAKE   : IN1 = HIGH, IN2 = HIGH  (both outputs low, shorted)
 *     STOP    : IN1 = LOW,  IN2 = LOW   (Hi-Z / standby)
 *
 *   Consequence of slow decay: a direction commanded at 0 % duty holds
 *   the motor in BRAKE, not coast. STOP selects the free/standby state.
 *
 *   Duty geometry (RGx0 is non-inverted in this scheme):
 *   The PWM pin uses T32A_OUTPUT_PPG = SET on CMP0 (RGx0), CLEAR on CMP1
 *   (RGx1 = period). Counting up from 0, the pin is LOW over 0..RGx0 and
 *   HIGH over RGx0..period. In slow decay the drive phase is the PWM-LOW
 *   phase, so drive time equals RGx0 counts and RGx0 maps directly to
 *   duty: RGx0 = speed% * period.
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "Motor.h"
#include "drivers/timer32A.h"

#define MAX_DUTY    100U
#define PERIOD_L    ((uint16_t)T32A_CH0_PERIOD)
#define PERIOD_R    ((uint16_t)T32A_CH3_PERIOD)

static motor_dir_t motor_left_dir  = STOP;
static motor_dir_t motor_right_dir = STOP;

/* ==========================================================================
 *   Private helper
 * ========================================================================== */

/**
 * @brief  Convert a speed/duty percentage to a timer compare value (RGx0).
 * @param  duty   Duty in percent, 0-100.
 * @param  period  Timer period in counts.
 * @return Compare value in [0, period-1]; RGx0 equals drive-time counts.
 *
 * @note   Non-inverted (slow-decay sense): a larger value yields more
 *         drive time, since the drive phase is the PWM-LOW interval
 *         (0..RGx0). A speed of 0 gives RGx0 = 0, i.e. full brake.
 */
static uint16_t Duty_ToCompare(uint8_t duty, uint16_t period)
{
    if (duty >= MAX_DUTY)
    {
        return (uint16_t)(period - 1U);     /* Near-full drive (1 count brake) */
    }

    return (uint16_t)((uint32_t)duty * (uint32_t)(period - 1U) / (uint32_t)MAX_DUTY);
}

/* ==========================================================================
 *   Initialization
 * ========================================================================== */

void Motor_Init(void)
{
    T32A0_Init(T32A_CH0_PERIOD);   /* Left motor  */
    T32A3_Init(T32A_CH3_PERIOD);   /* Right motor */

    /* Safe initial state: both inputs LOW (standby / Hi-Z) */
    T32A0_SetOutCRA1(T32A_OUTPUT_LOW);
    T32A0_SetOutCRB1(T32A_OUTPUT_LOW);
    T32A3_SetOutCRA1(T32A_OUTPUT_LOW);
    T32A3_SetOutCRB1(T32A_OUTPUT_LOW);

    motor_left_dir  = STOP;
    motor_right_dir = STOP;
}

/* ==========================================================================
 *   Start / Stop
 * ========================================================================== */

void Motor_Start(void)
{
    T32A0_Start();
    T32A3_Start();
}

void Motor_Stop(void)
{
    Motor_SetLeft(STOP, 0U);
    Motor_SetRight(STOP, 0U);

    T32A0_Stop();
    T32A3_Stop();
}

/* ==========================================================================
 *   Left motor  (T32A0: IN1 = A/OUTA, IN2 = B/OUTB)
 * ========================================================================== */

void Motor_SetLeft(motor_dir_t dir, uint8_t speed)
{
    uint16_t duty = Duty_ToCompare(speed, PERIOD_L);
 
    /* Fast path: same direction, update the PWM leg's duty only. FORWARD drives the PWM on IN1 (A); REVERSE on IN2 (B). */
    if (dir == motor_left_dir)
    {
        if (dir == FORWARD)
        {
            T32A0_SetTimerA0(duty);   /* IN1 is the PWM leg in FORWARD */
        }
        else if (dir == REVERSE)
        {
            T32A0_SetTimerB0(duty);   /* IN2 is the PWM leg in REVERSE */
        }
        /* BRAKE / STOP: no live update required */
        return;
    }
 
    /* Full path: stop, reconfigure output control, restart */
    T32A0_Stop();
 
    switch (dir)
    {
        /* Slow-decay forward: IN1 PWM, IN2 static HIGH (PWM low = drive) */
        case FORWARD:
            T32A0_SetOutCRA1(T32A_OUTPUT_PPG);       /* IN1 = PWM         */
            T32A0_SetOutCRB1(T32A_OUTPUT_HIGH);      /* IN2 = static HIGH */
            T32A0_SetTimerA0(duty);
            T32A0_SetTimerB0(0U);
            break;
 
        /* Slow-decay reverse: IN1 static HIGH, IN2 PWM (PWM low = drive) */
        case REVERSE:
            T32A0_SetOutCRA1(T32A_OUTPUT_HIGH);      /* IN1 = static HIGH */
            T32A0_SetOutCRB1(T32A_OUTPUT_PPG);       /* IN2 = PWM         */
            T32A0_SetTimerA0(0U);
            T32A0_SetTimerB0(duty);
            break;
 
        /* IN1=HIGH, IN2=HIGH -> both outputs low -> motor shorted */
        case BRAKE:
            T32A0_SetOutCRA1(T32A_OUTPUT_HIGH);
            T32A0_SetOutCRB1(T32A_OUTPUT_HIGH);
            T32A0_SetTimerA0(0U);
            T32A0_SetTimerB0(0U);
            break;
 
        /* IN1=LOW, IN2=LOW -> Hi-Z -> standby */
        case STOP:
            T32A0_SetOutCRA1(T32A_OUTPUT_LOW);
            T32A0_SetOutCRB1(T32A_OUTPUT_LOW);
            T32A0_SetTimerA0(0U);
            T32A0_SetTimerB0(0U);
            break;
    }
 
    motor_left_dir = dir;
    T32A0_Start();
}
 
/* ==========================================================================
 *   Right motor  (T32A3: IN1 = A/OUTA, IN2 = B/OUTB)
 * ========================================================================== */
 
void Motor_SetRight(motor_dir_t dir, uint8_t speed)
{
    uint16_t duty = Duty_ToCompare(speed, PERIOD_R);
 
    /* Fast path: FORWARD drives the PWM on IN1 (A); REVERSE on IN2 (B). */
    if (dir == motor_right_dir)
    {
        if (dir == FORWARD)
        {
            T32A3_SetTimerA0(duty);   /* IN1 is the PWM leg in FORWARD */
        }
        else if (dir == REVERSE)
        {
            T32A3_SetTimerB0(duty);   /* IN2 is the PWM leg in REVERSE */
        }
        return;
    }
 
    T32A3_Stop();
 
    switch (dir)
    {
        /* Slow-decay forward: IN1 PWM, IN2 static HIGH (PWM low = drive) */
        case FORWARD:
            T32A3_SetOutCRA1(T32A_OUTPUT_PPG);       /* IN1 = PWM         */
            T32A3_SetOutCRB1(T32A_OUTPUT_HIGH);      /* IN2 = static HIGH */
            T32A3_SetTimerA0(duty);
            T32A3_SetTimerB0(0U);
            break;
 
        /* Slow-decay reverse: IN1 static HIGH, IN2 PWM (PWM low = drive) */
        case REVERSE:
            T32A3_SetOutCRA1(T32A_OUTPUT_HIGH);      /* IN1 = static HIGH */
            T32A3_SetOutCRB1(T32A_OUTPUT_PPG);       /* IN2 = PWM         */
            T32A3_SetTimerA0(0U);
            T32A3_SetTimerB0(duty);
            break;
 
        /* IN1=HIGH, IN2=HIGH -> both outputs low -> motor shorted */
        case BRAKE:
            T32A3_SetOutCRA1(T32A_OUTPUT_HIGH);
            T32A3_SetOutCRB1(T32A_OUTPUT_HIGH);
            T32A3_SetTimerA0(0U);
            T32A3_SetTimerB0(0U);
            break;
 
        /* IN1=LOW, IN2=LOW -> Hi-Z -> standby */
        case STOP:
            T32A3_SetOutCRA1(T32A_OUTPUT_LOW);
            T32A3_SetOutCRB1(T32A_OUTPUT_LOW);
            T32A3_SetTimerA0(0U);
            T32A3_SetTimerB0(0U);
            break;
    }
 
    motor_right_dir = dir;
    T32A3_Start();
}

/* ==========================================================================
 *   Motor set
 * ========================================================================== */

void Motor_Set(motor_t motor, motor_dir_t dir, uint8_t speed)
{
    switch (motor)
    {
        case MOTOR_LEFT:   Motor_SetLeft(dir, speed);   break;
        case MOTOR_RIGHT:  Motor_SetRight(dir, speed);  break;
        default:                                        break;
    }
}