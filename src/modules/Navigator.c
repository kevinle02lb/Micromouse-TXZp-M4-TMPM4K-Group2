/**
 * @file        Navigator.c
 * @brief       Cell-level motion sequencer for micromouse.
 * @version     V1.1.0
 * @date        03-07-2026
 *
 * @details
 *   Bridges FloodFill (planner) with Motion/Odometry (execution).
 *   Runs a 1 kHz state machine: plan -> turn -> drive -> pause -> repeat.
 *
 *   Heading is tracked internally as an exact grid index (0..3), NOT by
 *   accumulating odometry. Each turn target is a clean multiple of 90 deg,
 *   so per-turn error never compounds across the run. Odometry is used only
 *   to CHECK arrival, never to DEFINE the next target.
 *
 * @note
 *   Call Navigator_Update() exactly once per 1 kHz control tick.
 *   Call Navigator_Init() after FloodFill_Init() and Motion_Init().
 *
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "Navigator.h"
#include "FloodFill.h"
#include "Motion.h"
#include "Odometry.h"
#include "Encoder.h"
#include <math.h>

/* ==========================================================================
 *   Tuning Constants
 * ========================================================================== */

#define CELL_SIZE_MM        180.0f      // one maze cell, center to center
#define TURN_TOLERANCE_RAD  0.05f       // ~3 deg: "close enough" for a turn
#define DIST_TOLERANCE_MM   5.0f        // stop this far before full cell
#define PAUSE_TICKS         50U         // settle time between moves (50 ms @ 1 kHz)

#define HEADING_COUNT       4U          // N/E/S/W grid quantization
#define HEADING_MASK        3U          // (idx & 3) wraps 0..3 without branches

#define KP_STRAIGHT   2.0f              /* start small; tune up until straight without wobble */

/* ==========================================================================
 *   State Machine
 * ========================================================================== */

/**
 * @brief  Navigator internal states.
 */
typedef enum
{
    NAV_PLANNING,       // ask FloodFill for the next action
    NAV_TURNING,        // rotating in place until heading matches target
    NAV_DRIVING,        // driving forward until one cell is covered
    NAV_PAUSED,         // brief settle, then report move done
    NAV_FINISHED        // goal reached, motors held stopped
} nav_state_t;

/* ==========================================================================
 *   Private Data
 * ========================================================================== */

/**
 * @brief  Absolute grid heading in radians, indexed by heading_idx.
 * @note   Matches odometry convention: 0 rad forward, +CCW.
 *         idx 0 = 0 deg, 1 = +90, 2 = +180, 3 = -90.
 */
static const float heading_rad[HEADING_COUNT] =
{
    0.0f,           // idx 0
    M_PI_DIV_2,     // idx 1  (+90)
    M_PI,           // idx 2  (+180)
    -M_PI_DIV_2     // idx 3  (-90)
};

static nav_state_t nav_state;
static floodfill_t current_action;

static uint8_t heading_idx;             // exact grid heading, 0..3 (source of truth)
static float   target_heading_rad;      // heading_rad[heading_idx] for the active turn

static float drive_start_x_mm;          // odometry X captured at move start
static float drive_start_y_mm;          // odometry Y captured at move start

static int32_t drive_start_countL;      // encoder gets counter of Left
static int32_t drive_start_countR;      // encoder gets counter of Right

static uint16_t pause_tick_count;

/* ==========================================================================
 *   Private Helpers
 * ========================================================================== */

/**
 * @brief  Fold an angle into [-pi, +pi].
 * @param  angle  Input angle (radians).
 * @return float  Equivalent angle in [-pi, +pi].
 * @note   Used on the turn ERROR, so the +180/-180 wrap resolves correctly.
 */
static float NormalizeAngle(float angle)
{
    while (angle > M_PI)
        angle -= M_2PI;
    while (angle < -M_PI)
        angle += M_2PI;
    return angle;
}

/**
 * @brief  Capture current odometry position as the start of a forward move.
 */
static void BeginDrive(void)
{
    drive_start_x_mm = Odometry_GetX_mm();
    drive_start_y_mm = Odometry_GetY_mm();

    drive_start_countL = Encoder_GetPosition(MOTOR_LEFT);
    drive_start_countR = Encoder_GetPosition(MOTOR_RIGHT);  

    Motion_SetMoveForwardSpeed(MOVE_SPEED);
    nav_state = NAV_DRIVING;
}

/* ==========================================================================
 *   Public Functions
 * ========================================================================== */

/**
 * @brief  Initialize the navigator state machine.
 * @note   Assumes the robot starts aligned to grid heading index 0.
 */
void Navigator_Init(void)
{
    nav_state = NAV_PLANNING;
    current_action = FLOODFILL_STOP;

    heading_idx = 0U;
    target_heading_rad = heading_rad[heading_idx];

    drive_start_x_mm = 0.0f;
    drive_start_y_mm = 0.0f;
    pause_tick_count = 0U;
}

/**
 * @brief  Advance the state machine by one tick.
 * @note   Call exactly once per 1 kHz control tick, AFTER Motion_Update().
 */
void Navigator_Update(void)
{
    switch (nav_state)
    {
        /* ==============================================================
         *  NAV_PLANNING — get next action, set up the turn or drive
         * ============================================================== */
        case NAV_PLANNING:
        {
            current_action = FloodFill_Plan();

            switch (current_action)
            {
                case FLOODFILL_STOP:
                    Motion_Stop();
                    nav_state = NAV_FINISHED;
                    break;

                case FLOODFILL_FORWARD:
                    BeginDrive();
                    break;

                case FLOODFILL_TURN_LEFT:                       // CCW: +90 on grid
                    heading_idx = (uint8_t)((heading_idx + 1U) & HEADING_MASK);
                    target_heading_rad = heading_rad[heading_idx];
                    Motion_SetTurnLeftSpeed(TURN_SPEED);
                    nav_state = NAV_TURNING;
                    break;

                case FLOODFILL_TURN_RIGHT:                      // CW: -90 == +3 mod 4
                    heading_idx = (uint8_t)((heading_idx + 3U) & HEADING_MASK);
                    target_heading_rad = heading_rad[heading_idx];
                    Motion_SetTurnRightSpeed(TURN_SPEED);
                    nav_state = NAV_TURNING;
                    break;

                case FLOODFILL_TURN_AROUND:                     // 180 == +2 mod 4
                    heading_idx = (uint8_t)((heading_idx + 2U) & HEADING_MASK);
                    target_heading_rad = heading_rad[heading_idx];
                    Motion_SetTurnLeftSpeed(TURN_SPEED);        // spin CCW by convention
                    nav_state = NAV_TURNING;
                    break;

                default:
                    break;
            }
            break;
        }

        /* ==============================================================
         *  NAV_TURNING — hold turn until heading is within tolerance
         * ============================================================== */
        case NAV_TURNING:
        {
            float error = NormalizeAngle(target_heading_rad - Odometry_GetHeading_rad());

            if (fabsf(error) < TURN_TOLERANCE_RAD)
            {
                Motion_Stop();      // kill turn spin before driving
                BeginDrive();
            }
            break;
        }

        /* ==============================================================
         *  NAV_DRIVING — drive until one cell length is covered
         * ============================================================== */
        case NAV_DRIVING:
        {
            float dx = Odometry_GetX_mm() - drive_start_x_mm;   // delta from THIS move's start
            float dy = Odometry_GetY_mm() - drive_start_y_mm;
            float dist_traveled = sqrtf((dx * dx) + (dy * dy));

            /* Error calculation to travel of traveling stright */
            int32_t dL = Encoder_GetPosition(MOTOR_LEFT)  - drive_start_countL;
            int32_t dR = Encoder_GetPosition(MOTOR_RIGHT) - drive_start_countR;

            float err  = (float)(dR - dL);        /* >0 → right ran farther → veering LEFT */
            float corr = KP_STRAIGHT * err;

            Motion_SetSpeed(MOVE_SPEED + corr, MOVE_SPEED - corr);

            /* Distance Check */
            if (dist_traveled >= (CELL_SIZE_MM - DIST_TOLERANCE_MM))
            {
                Motion_Stop();
                pause_tick_count = 0U;
                nav_state = NAV_PAUSED;
            }
            break;
        }

        /* ==============================================================
         *  NAV_PAUSED — Pause, then tell FloodFill the move completed
         * ============================================================== */
        case NAV_PAUSED:
            pause_tick_count++;

            if (pause_tick_count >= PAUSE_TICKS)
            {
                FloodFill_ReportDone(current_action);
                nav_state = NAV_PLANNING;
            }
            break;

        /* ==============================================================
         *  NAV_FINISHED — terminal, motors stay stopped
         * ============================================================== */
        case NAV_FINISHED:
            break;

        default:
            break;
    }
}