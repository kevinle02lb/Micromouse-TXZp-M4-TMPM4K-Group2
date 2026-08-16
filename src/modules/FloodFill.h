/**
 * @file        FloodFill.h
 * @brief       Flood fill path planner for a micromouse maze.
 * @version     V1.0.0
 * @date        15-08-2026
 *
 * @details
 *   Call order, once per cell:
 *   1. FloodFill_SetWalls()   - caller supplies front/left/right
 *   2. FloodFill_Plan()       - returns the action, changes no state
 *   3. FloodFill_ReportDone() - applies heading, then advances the cell
 *
 *   Every action except FLOODFILL_STOP ends one cell ahead: a turn means
 *   turn, then advance.
 *
 *   Start pose is cell (0,0) facing NORTH, with EAST 90 degrees clockwise.
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#ifndef FLOODFILL_H
#define FLOODFILL_H

#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 *   Defines
 * ========================================================================== */

#define MAZE_SIZE                   16U
#define CELL_UNVISITED              255U    /*!< Distance value for unreachable cells */

/**
 * @brief  Cardinal directions, ordered clockwise so that
 *         right = (dir + 1) & 3 and left = (dir + 3) & 3.
 */
typedef enum
{
    FLOODFILL_DIR_NORTH = 0,
    FLOODFILL_DIR_EAST  = 1,
    FLOODFILL_DIR_SOUTH = 2,
    FLOODFILL_DIR_WEST  = 3
} floodfill_dir_t;

/**
 * @brief  Wall bit positions within a cell's wall mask.
 */
typedef enum
{
    FLOODFILL_WALL_NORTH_BIT = 0x01U,
    FLOODFILL_WALL_EAST_BIT  = 0x02U,
    FLOODFILL_WALL_SOUTH_BIT = 0x04U,
    FLOODFILL_WALL_WEST_BIT  = 0x08U
} floodfill_wall_bit_t;

/**
 * @brief  Action returned by FloodFill_Plan.
 * @note   Every action other than FLOODFILL_STOP ends one cell ahead.
 */
typedef enum
{
    FLOODFILL_STOP = 0,       /*!< Inside the goal area, hold position */
    FLOODFILL_FORWARD,        /*!< Drive straight into the next cell */
    FLOODFILL_TURN_LEFT,      /*!< Turn 90 degrees CCW, then drive */
    FLOODFILL_TURN_RIGHT,     /*!< Turn 90 degrees CW, then drive */
    FLOODFILL_TURN_AROUND     /*!< Turn 180 degrees, then drive */
} floodfill_t;

/* ==========================================================================
 *   Function Prototypes
 * ========================================================================== */

void        FloodFill_Init(void);
void        FloodFill_SetWalls(bool front, bool left, bool right);
floodfill_t FloodFill_Plan(void);
void        FloodFill_ReportDone(floodfill_t action);
bool        FloodFill_IsAtGoal(void);

uint8_t     FloodFill_GetX(void);
uint8_t     FloodFill_GetY(void);

#endif /* FLOODFILL_H */