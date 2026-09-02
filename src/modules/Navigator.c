/**
 * @file        Navigator.c
 * @brief       Cell-level motion sequencer for micromouse.
 * @version     V1.0.0
 * @date        15-08-2026
 *
 * @details
 *   Bridges FloodFill (planner) with Profile and Motion (execution).
 *   Runs a 1 kHz state machine.
 *
 *   Every move is one or two segments. A segment is a fixed length of wheel
 *   path run under one profile:
 *
 *     drive   both wheels forward   length = CELL_SIZE_MM
 *     turn    wheels opposed        length = theta * WHEELBASE_MM / 2
 *
 *   State Flow:
 *   1. NAV_PLAN    - read walls, ask FloodFill, arm a segment
 *   2. NAV_TURN    - pivot, only if the action needs one
 *   3. NAV_BACKUP  - square against the dead end wall, only after a turn
 *                    around that was planned with a wall ahead
 *   4. NAV_DRIVE   - advance one cell, stopping against a wall ahead if one
 *                    comes into range
 *   5. NAV_SETTLE  - hold at the cell centre, report the move, back to 1
 *
 * @note
 *   Call Navigator_Update() exactly once per 1 kHz control tick, BEFORE
 *   Motion_Update(), so the setpoint it produces is acted on the same tick.
 *   Call Navigator_Init() after FloodFill_Init() and Motion_Init().
 *
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "Navigator.h"
#include "FloodFill.h"
#include "IrSensor.h"
#include "Motion.h"
#include "Profile.h"
#include "Odometry.h"
#include "Encoder.h"
#include <math.h>

/* ==========================================================================
 *   Tuning Constants
 * ========================================================================== */

#define CELL_SIZE_MM            180.0f      /* one maze cell, centre to centre */

#define DRIVE_SPEED_MM_S        600.0f      /* straight cruise ceiling */
#define DRIVE_MIN_MM_S           0.0f      /* straight floor, clears stiction */
#define DRIVE_ACCEL_MM_S2       650.0f      /* straight ramp and brake */

#define TURN_SPEED_MM_S         3000.0f      /* pivot cruise ceiling, per wheel */
#define TURN_MIN_MM_S            0.0f       /* pivot floor, clears stiction */
#define TURN_ACCEL_MM_S2        4500.0f      /* pivot ramp and brake  */

#define KP_STRAIGHT               3.0f      /* damping trim, (mm/s) per mm of wheel skew */
#define KP_WALL                   0.4f      /* wall trim, (mm/s) per mm of lateral error */
#define IR_SIDE_TARGET_MM        30.0f      /* side reading when centred, measured */
#define IR_FRONT_TARGET_MM       20.0f      /* front reading at a cell centre, measured */

#define SETTLE_TICKS             60U        /* stationary hold at the cell centre */

#define IR_SIDE_OFFSET_MM        45.0f      /* axle to side beam, forward positive, MEASURE */
#define AXLE_TO_REAR_MM          55.0f      /* axle to rearmost chassis point, MEASURE */
#define EDGE_TOLERANCE_MM        40.0f      /* edge is ignored if it lands outside this band */

#define BACKUP_SPEED_MM_S       120.0f      /* reverse crawl into the dead end wall */
#define BACKUP_STALL_MM           1.0f      /* travel below this over a window means contact */
#define BACKUP_STALL_TICKS       50U        /* stall detection window */
#define BACKUP_TIMEOUT_TICKS    800U        /* abort the crawl if no wall is ever met */

/* ==========================================================================
 *   State Machine
 * ========================================================================== */

/**
 * @brief  Navigator internal states.
 */
typedef enum
{
    NAV_PLAN,       // read walls, ask FloodFill, arm the first segment
    NAV_TURN,       // pivot through the planned angle
    NAV_BACKUP,     // reverse into the dead end wall to square up
    NAV_DRIVE,      // advance one cell
    NAV_SETTLE,     // hold at the cell centre, then commit the move
    NAV_FINISHED    // goal reached, motors held stopped
} nav_state_t;

/* ==========================================================================
 *   Private Data
 * ========================================================================== */

static nav_state_t nav_state;
static floodfill_t current_action;

static profile_t   segment;             // active drive or turn profile
static int32_t     seg_start_countL;    // left encoder position at segment start
static int32_t     seg_start_countR;    // right encoder position at segment start
static float       turn_sign;           // +1 = CCW, -1 = CW
static uint16_t    settle_ticks;        // ticks elapsed in the active settle

static bool        dead_end;            // wall ahead when the turn around was planned
static uint16_t    backup_ticks;        // ticks elapsed in the active crawl
static uint16_t    backup_window;       // ticks since the last stall sample
static float       backup_mark_mm;      // travel at the last stall sample

static bool        edge_seen_left;      // side wall state on the previous tick
static bool        edge_seen_right;
static bool        edge_applied;        // one edge correction per drive segment

/* ==========================================================================
 *   Motion Interface
 * ========================================================================== */

/**
 * @brief  Command both wheel speeds in mm/s.
 * @param  left_mm_s   Left wheel speed. Positive drives the robot forward.
 * @param  right_mm_s  Right wheel speed. Positive drives the robot forward.
 * @note   Motion_SetSpeed() takes counts per second. This is the only place
 *         the segment's physical units cross into encoder units.
 */
static void CommandWheels(float left_mm_s, float right_mm_s)
{
    Motion_SetSpeed(left_mm_s / MM_PER_COUNT, right_mm_s / MM_PER_COUNT);
}

/* ==========================================================================
 *   Segment Measurement
 * ========================================================================== */

/**
 * @brief  Latch the encoder positions that segment progress is measured from.
 * @note   Measuring from a per-segment reference keeps the result free of
 *         accumulated odometry drift and of the [-pi, pi] heading wrap.
 */
static void CaptureSegmentStart(void)
{
    seg_start_countL = Encoder_GetPosition(MOTOR_LEFT);
    seg_start_countR = Encoder_GetPosition(MOTOR_RIGHT);
}

/**
 * @brief  Left wheel path since the segment started.
 * @return float  Signed distance in mm.
 */
static float SegmentLeft_mm(void)
{
    return (float)(Encoder_GetPosition(MOTOR_LEFT) - seg_start_countL) * MM_PER_COUNT;
}

/**
 * @brief  Right wheel path since the segment started.
 * @return float  Signed distance in mm.
 */
static float SegmentRight_mm(void)
{
    return (float)(Encoder_GetPosition(MOTOR_RIGHT) - seg_start_countR) * MM_PER_COUNT;
}

/**
 * @brief  Forward distance advanced since the drive segment started.
 * @return float  Mean of the two wheel paths, in mm.
 * @note   The profile brakes against this same value, so the setpoint and the
 *         completion test share one measure. Odometry displacement would be a
 *         second: on a curved path its Euclidean chord falls short of the
 *         wheel path, letting the profile reach zero speed while the
 *         completion test never fires.
 */
static float SegmentDistance_mm(void)
{
    return 0.5f * (SegmentLeft_mm() + SegmentRight_mm());
}

/**
 * @brief  Wheel arc swept since the turn segment started.
 * @return float  Arc length in mm, positive in the commanded turn direction.
 * @details
 *   In an in-place pivot the wheels travel equal and opposite arcs, so the
 *   arc each has covered is half their difference. Multiplying by turn_sign
 *   makes the result count up for either direction, which lets the profile
 *   treat a clockwise turn identically to a counter-clockwise one.
 */
static float SegmentArc_mm(void)
{
    return 0.5f * (SegmentRight_mm() - SegmentLeft_mm()) * turn_sign;
}

/**
 * @brief  Speed differential that keeps the robot centred and straight.
 * @return float  Trim in mm/s, added to the left wheel and subtracted from
 *                the right.
 * @details
 *   Position comes from the side sensors, damping from wheel skew. Lateral
 *   position sits two integrations away from the wheel differential, so a
 *   proportional term alone weaves. Skew stands in for heading and settles it.
 *
 *   A single wall gives half the displacement signal that two do, so its
 *   deviation is doubled to hold the loop gain constant.
 *
 *   With no wall in range the skew term carries alone, holding whatever
 *   heading the segment started with.
 */
static float StraightnessTrim(void)
{
    float skew  = SegmentRight_mm() - SegmentLeft_mm();
    float error = 0.0f;
    bool  left  = IR_IsWallPresent(IR_FAR_LEFT);
    bool  right = IR_IsWallPresent(IR_FAR_RIGHT);

    /* Referenced Displacement on Right. Distance from IR sensor to wall */
    if (left && right)
        error = IR_GetDistance_mm(IR_FAR_RIGHT) - IR_GetDistance_mm(IR_FAR_LEFT);
    else if (left)
        error = 2.0f * (IR_SIDE_TARGET_MM - IR_GetDistance_mm(IR_FAR_LEFT));
    else if (right)
        error = 2.0f * (IR_GetDistance_mm(IR_FAR_RIGHT) - IR_SIDE_TARGET_MM);

    return (KP_WALL * error) + (KP_STRAIGHT * skew);
}

/**
 * @brief  Pin the active drive target to the cell boundary a side wall ends on.
 * @param  traveled_mm  Distance covered so far in this segment.
 * @details
 *   Maze walls terminate on the posts that mark cell corners, so the tick a
 *   side wall leaves the beam places that beam level with a cell boundary.
 *   The axle trails the beam by IR_SIDE_OFFSET_MM, so at that instant the
 *   axle still owes half a cell plus its own offset to reach the next centre.
 *
 *   Only the trailing edge is used. A leading edge marks the same boundary but
 *   arrives while the previous cell is still being left, which places it
 *   outside this segment.
 *
 *   The edge is accepted only if it lands within EDGE_TOLERANCE_MM of where
 *   dead reckoning expected it. A wall ending behind the segment start, or a
 *   dropout from a dirty reading, falls outside that band and is discarded.
 *
 *   One correction per segment. A wall that flickers near its end would
 *   otherwise reset the target on every flicker.
 */
static void ApplySideEdgeCorrection(float traveled_mm)
{
    bool  left    = IR_IsWallPresent(IR_FAR_LEFT);
    bool  right   = IR_IsWallPresent(IR_FAR_RIGHT);
    bool  falling = (edge_seen_left && !left) || (edge_seen_right && !right);
    float expected;

    edge_seen_left  = left;
    edge_seen_right = right;

    if (edge_applied || !falling)
        return;

    expected = (CELL_SIZE_MM * 0.5f) - IR_SIDE_OFFSET_MM;

    if (fabsf(traveled_mm - expected) > EDGE_TOLERANCE_MM)
        return;

    segment.distance_mm = traveled_mm + (CELL_SIZE_MM * 0.5f) + IR_SIDE_OFFSET_MM;
    edge_applied        = true;
}

/**
 * @brief  Move the active drive target to sit at a fixed distance from a wall
 *         ahead.
 * @param  traveled_mm  Distance covered so far in this segment.
 * @details
 *   Encoder distance accumulates error with nothing to correct it. A wall
 *   ahead is a fixed reference, so once one comes into range it defines where
 *   the segment ends no matter what the count has drifted to.
 *
 *   Both front sensors are averaged. A robot sitting at an angle reads long on
 *   one and short on the other, and the mean cancels that.
 *
 *   The profile recomputes its remaining distance every tick, so moving the
 *   target mid-segment reshapes the brake curve rather than restarting it.
 *
 *   Does nothing when no wall is in range, leaving the move on dead reckoning.
 */
static void ApplyFrontWallCorrection(float traveled_mm)
{
    float front_mm;

    if (!IR_IsWallPresent(IR_LEFT) || !IR_IsWallPresent(IR_RIGHT))
        return;

    front_mm = 0.5f * (IR_GetDistance_mm(IR_LEFT) + IR_GetDistance_mm(IR_RIGHT));

    segment.distance_mm = traveled_mm + (front_mm - IR_FRONT_TARGET_MM);
}

/* ==========================================================================
 *   Segment Control
 * ========================================================================== */

/**
 * @brief  Arm a straight-line segment.
 * @param  distance_mm  Forward distance to cover.
 */
static void BeginDrive(float distance_mm)
{
    CaptureSegmentStart();

    /* Seeded from the live readings so the first tick cannot register an edge
       against an uninitialised previous state. */
    edge_seen_left  = IR_IsWallPresent(IR_FAR_LEFT);
    edge_seen_right = IR_IsWallPresent(IR_FAR_RIGHT);
    edge_applied    = false;

    Profile_Begin(&segment, distance_mm,
                  DRIVE_SPEED_MM_S, DRIVE_MIN_MM_S, DRIVE_ACCEL_MM_S2);
    nav_state = NAV_DRIVE;
}

/**
 * @brief  Arm the reverse crawl that squares the robot on a dead end wall.
 * @details
 *   Runs after the turn around, with the dead end wall now behind. Contact is
 *   detected by stall rather than by a sensor, since nothing faces backwards.
 *   Both wheels reaching the wall together leaves the chassis square to it,
 *   which removes the heading error the dead end approach accumulated.
 */
static void BeginBackup(void)
{
    CaptureSegmentStart();
    backup_ticks   = 0U;
    backup_window  = 0U;
    backup_mark_mm = 0.0f;
    nav_state      = NAV_BACKUP;
}

/**
 * @brief  Arm an in-place pivot segment.
 * @param  angle_rad  Signed rotation to perform. Positive is counter-clockwise.
 * @details
 *   The segment length is the arc each wheel traces, theta * WHEELBASE_MM / 2,
 *   so a rotation runs through the same code path as a straight move.
 */
static void BeginTurn(float angle_rad)
{
    turn_sign = (angle_rad < 0.0f) ? -1.0f : 1.0f;

    CaptureSegmentStart();
    Profile_Begin(&segment, fabsf(angle_rad) * WHEELBASE_MM * 0.5f, TURN_SPEED_MM_S, TURN_MIN_MM_S, TURN_ACCEL_MM_S2);
    nav_state = NAV_TURN;
}

/* ==========================================================================
 *   Public Functions
 * ========================================================================== */

/**
 * @brief  Initialize the navigator state machine.
 * @note   The wheels are commanded to a stop so the loop starts from a known
 *         setpoint regardless of what ran before.
 */
void Navigator_Init(void)
{
    nav_state      = NAV_PLAN;
    current_action = FLOODFILL_STOP;

    seg_start_countL = 0;
    seg_start_countR = 0;
    turn_sign        = 1.0f;
    settle_ticks     = 0U;

    dead_end         = false;
    backup_ticks     = 0U;
    backup_window    = 0U;
    backup_mark_mm   = 0.0f;

    edge_seen_left   = false;
    edge_seen_right  = false;
    edge_applied     = false;

    CommandWheels(0.0f, 0.0f);

    Profile_Begin(&segment, 0.0f, DRIVE_SPEED_MM_S, DRIVE_MIN_MM_S, DRIVE_ACCEL_MM_S2);
}

/**
 * @brief  Advance the state machine by one tick.
 * @note   Call exactly once per 1 kHz control tick, before Motion_Update().
 */
void Navigator_Update(void)
{
    switch (nav_state)
    {
        /* ==============================================================
         *  NAV_PLAN — read walls, ask FloodFill, arm the first segment
         * ============================================================== */
        case NAV_PLAN:
        {
            bool front_wall = IR_IsWallPresent(IR_LEFT) &&
                              IR_IsWallPresent(IR_RIGHT);

            CommandWheels(0.0f, 0.0f);

            FloodFill_SetWalls(front_wall,
                               IR_IsWallPresent(IR_FAR_LEFT),
                               IR_IsWallPresent(IR_FAR_RIGHT));

            current_action = FloodFill_Plan();

            switch (current_action)
            {
                case FLOODFILL_FORWARD:
                    BeginDrive(CELL_SIZE_MM);
                    break;

                case FLOODFILL_TURN_LEFT:                   // CCW
                    BeginTurn(M_PI_DIV_2);
                    break;

                case FLOODFILL_TURN_RIGHT:                  // CW
                    BeginTurn(-M_PI_DIV_2);
                    break;

                case FLOODFILL_TURN_AROUND:
                    /* A turn around away from a wall is a replan, not a dead
                       end, and has nothing behind it to square against. */
                    dead_end = front_wall;
                    BeginTurn(M_PI);
                    break;

                case FLOODFILL_STOP:
                default:
                    Motion_Stop();
                    nav_state = NAV_FINISHED;
                    break;
            }
            break;
        }

        /* ==============================================================
         *  NAV_TURN — pivot until the commanded arc is covered
         * ============================================================== */
        case NAV_TURN:
        {
            float swept = SegmentArc_mm();
            float v     = Profile_Step(&segment, swept);

            CommandWheels(-v * turn_sign, v * turn_sign);

            if (Profile_IsComplete(&segment, swept))
            {
                CommandWheels(0.0f, 0.0f);

                if (dead_end)
                    BeginBackup();
                else
                    BeginDrive(CELL_SIZE_MM);
            }
            break;
        }

        /* ==============================================================
         *  NAV_BACKUP - crawl back onto the dead end wall, then set off
         * ============================================================== */
        case NAV_BACKUP:
        {
            float traveled = SegmentDistance_mm();

            CommandWheels(-BACKUP_SPEED_MM_S, -BACKUP_SPEED_MM_S);
            backup_ticks++;
            backup_window++;

            if (backup_window >= BACKUP_STALL_TICKS)
            {
                bool stopped = fabsf(traveled - backup_mark_mm) < BACKUP_STALL_MM;

                backup_window  = 0U;
                backup_mark_mm = traveled;

                if (stopped || backup_ticks >= BACKUP_TIMEOUT_TICKS)
                {
                    CommandWheels(0.0f, 0.0f);
                    dead_end = false;

                    /* Squared on the wall the axle sits AXLE_TO_REAR_MM from
                       it, and the next cell centre is one and a half cells
                       further on. */
                    BeginDrive((CELL_SIZE_MM * 1.5f) - AXLE_TO_REAR_MM);
                }
            }
            break;
        }

        /* ==============================================================
         *  NAV_DRIVE — advance one cell under the straightness trim
         * ============================================================== */
        case NAV_DRIVE:
        {
            float traveled = SegmentDistance_mm();
            float v;
            float trim;

            /* The wall ahead is a direct measurement of what remains, so it
               is applied last and overrides the edge. */
            ApplySideEdgeCorrection(traveled);
            ApplyFrontWallCorrection(traveled);

            v    = Profile_Step(&segment, traveled);
            trim = StraightnessTrim();

            CommandWheels(v + trim, v - trim);

            if (Profile_IsComplete(&segment, traveled))
            {
                CommandWheels(0.0f, 0.0f);
                settle_ticks = 0U;
                nav_state    = NAV_SETTLE;
            }
            break;
        }

        /* ==============================================================
         *  NAV_SETTLE - hold at the cell centre, then commit the move
         * ============================================================== */
        case NAV_SETTLE:
        {
            CommandWheels(0.0f, 0.0f);
            settle_ticks++;

            if (settle_ticks >= SETTLE_TICKS)
            {
                FloodFill_ReportDone(current_action);
                nav_state = NAV_PLAN;
            }
            break;
        }

        /* ==============================================================
         *  NAV_FINISHED — terminal, bridges left in standby
         * ============================================================== */
        case NAV_FINISHED:
        default:
            break;
    }
}