/**
 * @file        Odometry.c
 * @brief       Wheel odometry for differential drive robot
 * @version     V1.0.0
 * @date        18-06-2026
 *
 * @details
 *   Calculates the robot's position (x, y) and orientation (theta) from
 *   left/right encoder counts. Based on the differential drive kinematic
 *   model as described in:
 *   https://automaticaddison.com/calculating-wheel-odometry-for-a-differential-drive-robot/
 * 
 *   Some Help:
 *   https://aleksandarhaber.com/clear-and-detailed-explanation-of-kinematics-equations-and-geometry-of-motion-of-differential-wheeled-robot-differential-drive-robot/#google_vignette
 *   https://www.hmc.edu/lair/ARW/ARW-Lecture01-Odometry.pdf
 * 
 *   Pololu 80x10mm wheels (diameter = 80 mm).
 *   Pololu Micro Metal Gearmotor Datasheet (Rev 6-2, Page 2):
 *   "12 CPR of the motor shaft when counting both edges of both channels"
 *   -> 4x quadrature is ALREADY included in the 12 CPR figure.
 *   -> A-ENC32 hardware does the 4x counting automatically.
 *
 *   12 CPR x 29.89:1 exact gear ratio = 358.68 -> use 360 nominal.
 *   Calibrate by spinning wheel one full turn, reading raw count.
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "Odometry.h"

odometry_t odo;

/* ==========================================================================
 *   Initialization
 * ========================================================================== */

void Odometry_Init(void)
{
    odo.x_mm = 0.0f;
    odo.y_mm = 0.0f;
    odo.heading_rad = 0.0f;
    odo.left_dist_mm = 0.0f;
    odo.right_dist_mm = 0.0f;
    odo.prev_left_pos = Encoder_GetPosition(MOTOR_LEFT);
    odo.prev_right_pos = Encoder_GetPosition(MOTOR_RIGHT);
}

void Odometry_Reset(void)
{
    Odometry_Init();
}

/* ==========================================================================
 *   Odometry Update
 * ========================================================================== */

/**
 * @brief  Update robot pose from encoder data.
 * @details
 *   This function implements the standard differential drive odometry
 *   algorithm in 4 steps:
 *
 *   Step 1: Calculate wheel displacements
 *   -------------------------------------
 *   Read encoder counts and convert to distance traveled by each wheel.
 *   D = N * (2 * pi * r) / counts_per_rev = delta_counts * MM_PER_COUNT
 *
 *   Step 2: Calculate average displacement and orientation change
 *   -------------------------------------------------------------
 *   D_avg = (D_left + D_right) / 2
 *   delta_theta = (D_right - D_left) / L
 *
 *   Step 3: Calculate changes in global position
 *   ---------------------------------------------
 *   theta_new = theta + delta_theta
 *   delta_x = D_avg * cos(theta_new)
 *   delta_y = D_avg * sin(theta_new)
 *
 *   Step 4: Update global position and orientation
 *   ---------------------------------------------
 *   x_new = x + delta_x
 *   y_new = y + delta_y
 *   theta_new = theta + delta_theta (normalized to [-pi, pi])
 *
 *   Reference:
 *   https://automaticaddison.com/calculating-wheel-odometry-for-a-differential-drive-robot/
 */
void Odometry_Update(void)
{
    // Step 1: Calculate wheel displacements
    int32_t left_pos = Encoder_GetPosition(MOTOR_LEFT);
    int32_t right_pos = Encoder_GetPosition(MOTOR_RIGHT);

    int32_t left_delta = left_pos - odo.prev_left_pos;
    int32_t right_delta = right_pos - odo.prev_right_pos;

    float D_left = (float)left_delta * MM_PER_COUNT;
    float D_right = (float)right_delta * MM_PER_COUNT;

    // Step 2: Calculate average displacement and orientation change
    float D_avg = (D_left + D_right) / AVG_DIVISOR;
    float delta_theta = (D_right - D_left) / WHEELBASE_MM;

    // Step 3: Calculate changes in global position
    float theta_new = odo.heading_rad + delta_theta;

    float delta_x = D_avg * cosf(theta_new);
    float delta_y = D_avg * sinf(theta_new);

    // Step 4: Update global position and orientation
    odo.x_mm += delta_x;
    odo.y_mm += delta_y;
    odo.heading_rad = theta_new;

    // Normalize heading to [-PI, PI]
    if (odo.heading_rad > M_PI) 
        odo.heading_rad -= M_2PI;
    else if (odo.heading_rad < -M_PI) 
        odo.heading_rad += M_2PI;

    // Store encoder positions for next tick
    odo.prev_left_pos = left_pos;
    odo.prev_right_pos = right_pos;

    // Accumulate total wheel distances (optional, for diagnostics)
    odo.left_dist_mm += D_left;
    odo.right_dist_mm += D_right;
}

/* ==========================================================================
 *   Accessors
 * ========================================================================== */

/**
 * @brief  Get global X coordinate.
 * @return float  X position in mm.  +X = forward when heading = 0.
 */
float Odometry_GetX_mm(void)
{
    return odo.x_mm;
}

/**
 * @brief  Get global Y coordinate.
 * @return float  Y position in mm.  +Y = left when heading = 0.
 */
float Odometry_GetY_mm(void)
{
    return odo.y_mm;
}

/**
 * @brief  Get heading in radians.
 * @return float  Heading angle.  0 = +X, positive = CCW (left turn).
 *               Range is [-PI, PI].
 */
float Odometry_GetHeading_rad(void)
{
    return odo.heading_rad;
}

/**
 * @brief  Get heading in degrees.
 * @return float  Heading angle.  0 = forward, +90 = left, -90 = right.
 */
float Odometry_GetHeading_deg(void)
{
    return odo.heading_rad * RAD_TO_DEG;
}