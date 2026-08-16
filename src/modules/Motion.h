/**
 * @file        Motion.h
 * @brief       Speed control for the Micromouse
 * @version     V1.0.0
 * @date        27-06-2026
 *
 * @details
 *   PID-based wheel speed controller with convenience wrappers
 *   for common moves. Caller decides when to stop.
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include <stdbool.h>
#include "Motor.h"
#include "Odometry.h"

/* User Changeable Speed */
#define TURN_SPEED              450         /*!< Turn Speed in CPS */
#define MOVE_SPEED              450         /*!< Move forward Speed in CPS */

#define MOTION_DEADZONE         0.5f
#define MOTION_ROUND_OFFSET     0.5f

#define HEADING_TOLERANCE_RAD   0.02f       /*!< Turn Tolerance */

void Motion_Init(void);
void Motion_Update(void);

void Motion_SetSpeed(float left_cps, float right_cps);
void Motion_SetMoveForwardSpeed(float speed_cps);
void Motion_SetMoveBackwardSpeed(float speed_cps);
void Motion_SetTurnLeftSpeed(float speed_cps);
void Motion_SetTurnRightSpeed(float speed_cps);
void Motion_Stop(void);

float Motion_GetTarget(motor_t motor);      /*!< Commanded speed for a wheel (CPS) */
float Motion_GetOutput(motor_t motor);      /*!< Last PID output for a wheel, [-100, +100] */

void Motion_SetHeading(float target_rad);   /*!< Motion functons regarding to turning motion */
bool Motion_HeadingReached(void);

#endif /* MOTION_H */