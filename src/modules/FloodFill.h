/**
 * @file        FloodFill.h
 * @brief       Flood fill path planner for a micromouse maze.
 * @version     V1.0.0
 * @date        27-06-2026
 *
 * @details
 *   
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
#define CELL_UNVISITED              255U        /*!< 2^8 - 1 = 256 - 1 = 255 */

/** 
 * @brief Cardinal directions (used for wall indexing and neighbour offsets) 
 */
typedef enum 
{
    FLOODFILL_DIR_NORTH = 0,
    FLOODFILL_DIR_EAST,
    FLOODFILL_DIR_SOUTH,
    FLOODFILL_DIR_WEST
} floodfill_dir_t;

/**
 * @brief  Bit position wall detection
 */
typedef enum 
{
    FLOODFILL_WALL_NORTH_BIT = 0x01U,   /*!< bit 0 */
    FLOODFILL_WALL_EAST_BIT  = 0x02U,   /*!< bit 1 */
    FLOODFILL_WALL_SOUTH_BIT = 0x04U,   /*!< bit 2 */
    FLOODFILL_WALL_WEST_BIT  = 0x08U    /*!< bit 3 */
} floodfill_wall_bit_t;

/**
 * @brief  Action returned by FloodFill_Plan().
 */
typedef enum 
{
    FLOODFILL_STOP = 0,       /*!< At goal, do nothing */
    FLOODFILL_WAIT,           /*!< IR cycle incomplete, stay put and retry */
    FLOODFILL_FORWARD,        /*!< Drive straight into next cell */
    FLOODFILL_TURN_LEFT,      /*!< Turn 90° CCW, then drive */
    FLOODFILL_TURN_RIGHT,     /*!< Turn 90° CW, then drive */
    FLOODFILL_TURN_AROUND     /*!< Turn 180°, then drive */
} floodfill_t;

/* ==========================================================================
 *   Function Prototypes
 * ========================================================================== */
void FloodFill_Init(void);
floodfill_t FloodFill_Plan(void);
void FloodFill_ReportDone(floodfill_t action);
bool FloodFill_IsAtGoal(void);


#endif /* FLOODFILL_H */