/**
 * @file        Motor.h
 * @brief       Motor control API for TB67H450AFNG H-bridge drivers
 * @version     V1.0.0
 * @date        08-06-2026
 *
 * @details
 *   High-level motor control. No register access — all hardware calls
 *   go through timer32A.h.
 *
 *   Wiring (from schematic):
 *     Left  motor: PA3 (TimerA / IN1), PA4= (TimerB / IN2)
 *     Right motor: PC2 (TimerA / IN1), PC3 (TimerB / IN2)
 *
 *   Datasheet truth table (TB67H450AFNG):
 *     IN1=H, IN2=L  →  Forward  (OUT1=H, OUT2=L, current OUT1→OUT2)
 *     IN1=L, IN2=H  →  Reverse  (OUT1=L, OUT2=H, current OUT2→OUT1)
 *     IN1=H, IN2=H  →  Brake    (both outputs L, motor shorted)
 *     IN1=L, IN2=L  →  Stop     (Hi-Z, enters standby after t_stby)
 *
 *
 * Copyright (c) [Kevin Le] 2026
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

typedef enum
{
    MOTOR_LEFT  = 0,   /*!< T32A0 — PA3 (TimerA/IN1), PA4 (TimerB/IN2) */
    MOTOR_RIGHT = 1    /*!< T32A3 — PC2 (TimerA/IN1), PC3 (TimerB/IN2) */
} motor_t;

typedef enum
{
    FORWARD = 0,   /*!< Forward        (IN1=PWM,  IN2=LOW) */
    REVERSE,       /*!< Reverse        (IN1=LOW,  IN2=PWM) */
    BRAKE,         /*!< Active brake   (IN1=HIGH, IN2=HIGH, motor shorted) */
    STOP           /*!< Stop/standby   (IN1=LOW,  IN2=LOW,  Hi-Z output) */
} motor_dir_t;

void Motor_Init(void);
void Motor_Start(void);
void Motor_Stop(void);

void Motor_Set(motor_t motor, motor_dir_t dir, uint8_t speed);
void Motor_SetLeft(motor_dir_t dir, uint8_t speed);
void Motor_SetRight(motor_dir_t dir, uint8_t speed);

#endif /* MOTOR_H */