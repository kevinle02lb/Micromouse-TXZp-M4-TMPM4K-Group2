/**
 * @file        Profile.c
 * @brief       One-dimensional trapezoidal motion profile implementation.
 * @version     V1.0.0
 * @date        15-08-2026
 *
 * @details
 *   Pure computation. No hardware dependency and no knowledge of which
 *   segment is running.
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "Profile.h"
#include <math.h>

/* ==========================================================================
 *   Public Functions
 * ========================================================================== */

/**
 * @brief  Arm a profile for a new segment.
 * @param  p             Profile instance.
 * @param  distance_mm   Total path length of the segment.
 * @param  v_max_mm_s    Cruise ceiling.
 * @param  v_min_mm_s    Setpoint floor while the segment is incomplete.
 * @param  accel_mm_s2   Ramp and brake magnitude.
 * @note   The setpoint starts at rest; the floor is applied by Profile_Step
 *         on the first call, so the segment begins at v_min rather than zero.
 */
void Profile_Begin(profile_t *p,
                   float distance_mm,
                   float v_max_mm_s,
                   float v_min_mm_s,
                   float accel_mm_s2)
{
    p->distance_mm = distance_mm;
    p->v_max_mm_s  = v_max_mm_s;
    p->v_min_mm_s  = v_min_mm_s;
    p->accel_mm_s2 = accel_mm_s2;
    p->v_now_mm_s  = 0.0f;
}

/**
 * @brief  Advance the profile by one control tick.
 * @param  p             Profile instance.
 * @param  traveled_mm   Path length covered since the segment started.
 * @return float  Velocity setpoint for this tick, mm/s, always positive.
 * @details
 *   Returns the smallest of the ramp, cruise and brake limits, then raises
 *   the result to v_min so the command stays above stiction. Once the target
 *   is reached the floor is withdrawn and the setpoint drops to zero.
 */
float Profile_Step(profile_t *p, float traveled_mm)
{
    float remaining = p->distance_mm - traveled_mm;
    float v_ramp;
    float v_brake;
    float v;

    if (remaining <= 0.0f)
    {
        p->v_now_mm_s = 0.0f;
        return 0.0f;
    }

    v_ramp  = p->v_now_mm_s + (p->accel_mm_s2 * PROFILE_DT);
    v_brake = sqrtf(2.0f * p->accel_mm_s2 * remaining);

    v = p->v_max_mm_s;

    if (v_ramp < v)
        v = v_ramp;

    if (v_brake < v)
        v = v_brake;

    if (v < p->v_min_mm_s)
        v = p->v_min_mm_s;

    p->v_now_mm_s = v;
    return v;
}

/**
 * @brief  Test whether the segment has covered its commanded length.
 * @param  p             Profile instance.
 * @param  traveled_mm   Path length covered since the segment started.
 * @param  tolerance_mm  Distance short of the target that still counts as done.
 * @return bool  true once the segment is complete.
 */
bool Profile_IsComplete(const profile_t *p, float traveled_mm, float tolerance_mm)
{
    return (traveled_mm >= (p->distance_mm - tolerance_mm));
}