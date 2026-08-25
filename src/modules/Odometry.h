/**
 * @file        Odometry.h
 * @brief       Position tracking
 * @version     V2.1.0
 * @date        15-08-2026
 *
 * @details
 *   Uses Encoder to find positioning.
 *   Tracks robot position (X, Y) and heading angle using left/right
 *   encoder counts. Uses Pololu 80x10mm wheels (diameter = 80 mm).
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
 *   Wheel CPR = 12 x 29.89 = 358.68 ~ 359 counts per wheel revolution.
 *
 *   -> 4x quadrature is ALREADY included in the 12 CPR figure.
 *   -> A-ENC32 hardware does the 4x counting automatically.
 *
 *   Motor datasheet:
 *   https://www.pololu.com/file/0J1487/pololu-micro-metal-gearmotors-rev-6-2.pdf
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "Encoder.h"
#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 *   DEFINES
 * ========================================================================== */

#undef M_PI
#define M_PI                                3.1415927f      /*!< Single precision for Float */
#define M_2PI                               6.2831855f      /*!< 2 * PI */
#define M_PI_DIV_2                          (M_PI / 2.0f)   /*!< 90 Degree - [pi/2] */

#define WHEEL_DIAMETER_MM                   82              /*!< 80mm from Datasheet */
#define WHEELBASE_MM                        100.0f           /*!< Distance between wheels (mm) */
#define WHEEL_CIRCUMFERENCE_MM              (WHEEL_DIAMETER_MM * M_PI)

/**
 * Counts per wheel revolution.
 *
 *   Pololu micro metal gearmotor encoder (Rev 6-2, pg 2):
 *   "12 CPR of the motor shaft when counting both edges of both channels"
 *   -> 4x quadrature is ALREADY included in the 12 CPR figure.
 *   -> A-ENC32 hardware does the 4x counting automatically.
 *
 *   12 CPR x 29.89:1 exact gear ratio = 358.68
 *   Calibrate by spinning wheel one full turn, reading raw count.
 */
#define COUNTS_PER_REV                      359U            /*!< Motor shaft 12 CPR x 30:1 gear */

#define MM_PER_COUNT                        (WHEEL_CIRCUMFERENCE_MM / COUNTS_PER_REV)
#define RAD_PER_COUNT                       (MM_PER_COUNT / WHEELBASE_MM)

#define AVG_DIVISOR                         2.0f            /*!< Denominator for averaging two wheel distances */
#define RAD_TO_DEG                          (180.0f / M_PI)

/* ==========================================================================
 *   ODOMETRY STRUCT
 * ========================================================================== */

typedef struct
{
    float   x_mm;                   /*!< X position in mm (forward) */
    float   y_mm;                   /*!< Y position in mm (left/right) */
    float   heading_rad;            /*!< Heading angle in radians (0 = forward) */
    float   left_dist_mm;           /*!< Total distance traveled by left wheel */
    float   right_dist_mm;          /*!< Total distance traveled by right wheel */
    int32_t prev_left_pos;          /*!< Previous left encoder position */
    int32_t prev_right_pos;         /*!< Previous right encoder position */
} odometry_t;

/* ==========================================================================
 *   Function Prototypes
 * ========================================================================== */

void  Odometry_Init(void);
void  Odometry_Reset(void);
void  Odometry_Update(void);
float Odometry_GetX_mm(void);
float Odometry_GetY_mm(void);
float Odometry_GetHeading_rad(void);
float Odometry_GetHeading_deg(void);

#ifdef __cplusplus
}
#endif
#endif /* ODOMETRY_H */