/**
 * @file        Profile.h
 * @brief       One-dimensional trapezoidal motion profile.
 * @version     V1.0.0
 * @date        15-08-2026
 *
 * @details
 *   Velocity setpoint for a fixed-length move, in mm, mm/s and mm/s^2.
 *
 *   Each tick returns the smallest of three limits:
 *   1. Ramp    - v_prev + a * dt
 *   2. Cruise  - v_max
 *   3. Brake   - sqrt(2 * a * remaining)
 *
 *   A pivot of theta radians is a segment of theta * WHEELBASE_MM / 2 per
 *   wheel, so turns and straights share one profile.
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#ifndef PROFILE_H
#define PROFILE_H

#include <stdbool.h>

#define PROFILE_DT      0.001f      /*!< Control tick period in seconds, 1 kHz */

typedef struct
{
    float distance_mm;      /*!< Total path length of the segment */
    float v_max_mm_s;       /*!< Cruise ceiling */
    float v_min_mm_s;       /*!< Setpoint floor while the segment is running */
    float accel_mm_s2;      /*!< Ramp and brake magnitude */
    float v_now_mm_s;       /*!< Setpoint carried from the previous tick */
} profile_t;

/* ==========================================================================
 *   Function Prototypes
 * ========================================================================== */

void  Profile_Begin(profile_t *p,
                    float distance_mm,
                    float v_max_mm_s,
                    float v_min_mm_s,
                    float accel_mm_s2);

float Profile_Step(profile_t *p, float traveled_mm);

bool  Profile_IsComplete(const profile_t *p, float traveled_mm);

#endif /* PROFILE_H */