/**
 * @file        FloodFill.c
 * @brief       Flood fill maze solver for IEEE 16x16 micromouse.
 * @version     V1.0.0
 * @date        27-06-2026
 *
 * @details
 *   IEEE goal area: 2x2 block at maze center (cells 7-8, 7-8).
 *   BFS seeds all four goal cells simultaneously so every reachable
 *   cell stores its shortest distance to the nearest goal tile.
 *
 *   Initialization uses Manhattan distance to the nearest edge of
 *   the 2x2 goal block rather than a single point.
 *
 *   This module is a pure planner. It decides which direction to face next.
 *   The Navigator module executes motion and reports completion.
 *
 *   Key concepts:
 *
 *   BFS (Breadth-First Search):
 *   - Explores the grid level by level from the goal
 *   - Respects discovered walls
 *   - Each cell gets its shortest distance to goal
 *   - Uses a FIFO queue (first-in, first-out)
 *
 *   Queue Operations:
 *   - Enqueue: Add cell to back of queue
 *   - Dequeue: Remove cell from front of queue
 *   - This ensures level-by-level exploration
 *
 *   Algorithm Flow:
 *   1. FloodFill_Plan():
 *      a. Scan walls from IR sensors
 *      b. Run BFS from 2x2 goal area to recalculate distances
 *      c. Pick lowest-distance neighbor
 *      d. Return action: FORWARD / LEFT / RIGHT / UTURN / STOP
 *   2. Navigator executes turn/drive physically
 *   3. Navigator calls FloodFill_ReportDone() when arrived
 *   4. Repeat until goal reached
 *  
 *   Resources:
 *      https://www.geeksforgeeks.org/dsa/flood-fill-algorithm/
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "FloodFill.h"
#include "IrSensor.h"

#define GOAL_MIN_X   7U
#define GOAL_MIN_Y   7U
#define GOAL_MAX_X   8U
#define GOAL_MAX_Y   8U

/* ==========================================================================
 *   Private Data
 * ========================================================================== */

/**
 * @brief  walls[16][16] - Wall bitmask for each cell
 * @details
 *   Each cell stores 4 bits:
 *   - Bit 0 (0x01): North wall
 *   - Bit 1 (0x02): East wall
 *   - Bit 2 (0x04): South wall
 *   - Bit 3 (0x08): West wall
 */
static uint8_t walls[MAZE_SIZE][MAZE_SIZE];

/**
 * @brief  dist[16][16] - Distance to nearest goal tile for each cell
 * @details
 *   - 0 = inside 2x2 goal area
 *   - 1 = adjacent to goal
 *   - 2 = two steps from goal
 *   - ...
 *   - 255 (CELL_UNVISITED) = unreachable (blocked by walls)
 *
 *   Values are calculated by BFS from the goal.
 *   The mouse always moves to the neighbor with the lowest dist.
 */
static uint8_t dist[MAZE_SIZE][MAZE_SIZE];

/**
 * @brief  Mouse position and heading
 * @details
 *   - mouse_x, mouse_y: Current cell (0-15)
 *   - mouse_heading: Which way the mouse faces (FLOODFILL_DIR_NORTH/EAST/SOUTH/WEST)
 *
 *   These are updated by FloodFill_ReportDone() after each move.
 *   They are NOT read from Odometry to keep the module self-contained.
 */
static uint8_t mouse_x, mouse_y;
static floodfill_dir_t mouse_heading;

/* ==========================================================================
 *   Queue
 * ========================================================================== */

/**
 * @brief  Queue for BFS (Breadth-First Search)
 * @details
 *   BFS uses a FIFO queue to process cells level by level:
 *
 *   1. Start with goal cell in queue
 *   2. Dequeue a cell, check its 4 neighbors
 *   3. If a neighbor is reachable (no wall) and unvisited:
 *      - Set its distance = current distance + 1
 *      - Enqueue it
 *   4. Repeat until queue is empty
 *
 *   QUEUE_SIZE = 256 (max cells in 16x16 maze)
 *   No overflow possible if BFS is correct.
 */
#define QUEUE_SIZE  (MAZE_SIZE * MAZE_SIZE)

typedef struct
{
    uint8_t x;
    uint8_t y;
} cell_t;

static cell_t queue[QUEUE_SIZE];
static uint16_t queue_head, queue_tail;

/* ==========================================================================
 *   Queue Operations
 * ========================================================================== */

/**
 * @brief  Reset queue to empty
 * @details
 *   Sets head = tail = 0. Queue is empty when head == tail.
 */
static void Queue_Init(void)
{
    queue_head = 0;
    queue_tail = 0;
}

/**
 * @brief  Add cell to back of queue.
 * @param  x  Cell x coordinate
 * @param  y  Cell y coordinate
 * @details
 *   Enqueue = add to tail. tail increments.
 *   If tail reaches QUEUE_SIZE, queue is full (shouldn't happen).
 */
static void Queue_Enqueue(uint8_t x, uint8_t y)
{
    if (queue_tail < QUEUE_SIZE)          /* Guard: do not write past array end */
    {
        queue[queue_tail].x = x;            /* Write X into slot [tail] */
        queue[queue_tail].y = y;            /* Write Y into slot [tail] */
        queue_tail++;                       /* Move write pointer forward by 1 */
    }
}

/**
 * @brief  Remove cell from front of queue.
 * @param  cell  Pointer to store dequeued cell
 * @return bool  true if cell was dequeued, false if queue empty
 * @details
 *   Dequeue = remove from head. head increments.
 *   Returns false if head == tail (queue empty).
 */
static bool Queue_Dequeue(cell_t *cell)
{
    if (queue_head == queue_tail)           /* Empty check */
        return false;                       /* Queue Empty */

    cell->x = queue[queue_head].x;          /* Copy X out */
    cell->y = queue[queue_head].y;          /* Copy Y out */
    queue_head++;                           /* Move read pointer forward by 1 */
    return true;
}

/* ==========================================================================
 *   Wall Helpers
 * ========================================================================== */

/**
 * @brief  Get absolute wall bit for "front" relative to heading.
 * @param  heading  Current mouse heading (FLOODFILL_DIR_NORTH/EAST/SOUTH/WEST)
 * @return uint8_t  Corresponding wall bit
 *
 * @details
 *   Example: If mouse faces NORTH, front wall = NORTH wall.
 *            If mouse faces EAST,  front wall = EAST wall.
 *   This converts the mouse's relative "front" to an absolute direction.
 */
static uint8_t FloodFill_GetFrontWall(floodfill_dir_t heading)
{
    switch (heading)
    {
        case FLOODFILL_DIR_NORTH: return FLOODFILL_WALL_NORTH_BIT;
        case FLOODFILL_DIR_EAST:  return FLOODFILL_WALL_EAST_BIT;
        case FLOODFILL_DIR_SOUTH: return FLOODFILL_WALL_SOUTH_BIT;
        case FLOODFILL_DIR_WEST:  return FLOODFILL_WALL_WEST_BIT;
        default:    return FLOODFILL_WALL_NORTH_BIT;
    }
}

/**
 * @brief  Get absolute wall bit for "left" relative to heading.
 * @param  heading  Current mouse heading (FLOODFILL_DIR_NORTH/EAST/SOUTH/WEST)
 * @return uint8_t  Corresponding wall bit
 *
 * @details
 *   Example: If mouse faces NORTH, left wall = WEST wall.
 *            If mouse faces EAST,  left wall = NORTH wall.
 */
static uint8_t FloodFill_GetLeftWall(floodfill_dir_t heading)
{
    switch (heading)
    {
        case FLOODFILL_DIR_NORTH: return FLOODFILL_WALL_WEST_BIT;
        case FLOODFILL_DIR_EAST:  return FLOODFILL_WALL_NORTH_BIT;
        case FLOODFILL_DIR_SOUTH: return FLOODFILL_WALL_EAST_BIT;
        case FLOODFILL_DIR_WEST:  return FLOODFILL_WALL_SOUTH_BIT;
        default:    return FLOODFILL_WALL_WEST_BIT;
    }
}

/**
 * @brief  Get absolute wall bit for "right" relative to heading.
 * @param  heading  Current mouse heading (FLOODFILL_DIR_NORTH/EAST/SOUTH/WEST)
 * @return uint8_t  Corresponding wall bit
 *
 * @details
 *   Example: If mouse faces NORTH, right wall = EAST wall.
 *            If mouse faces EAST,  right wall = SOUTH wall.
 */
static uint8_t FloodFill_GetRightWall(floodfill_dir_t heading)
{
    switch (heading)
    {
        case FLOODFILL_DIR_NORTH: return FLOODFILL_WALL_EAST_BIT;
        case FLOODFILL_DIR_EAST:  return FLOODFILL_WALL_SOUTH_BIT;
        case FLOODFILL_DIR_SOUTH: return FLOODFILL_WALL_WEST_BIT;
        case FLOODFILL_DIR_WEST:  return FLOODFILL_WALL_NORTH_BIT;
        default:    return FLOODFILL_WALL_EAST_BIT;
    }
}

/* ==========================================================================
 *   Wall Management
 * ========================================================================== */

/**
 * @brief  Store a wall and its symmetric neighbor wall.
 * @param  x         Cell x coordinate (0-15)
 * @param  y         Cell y coordinate (0-15)
 * @param  wall_bit  FLOODFILL_WALL_NORTH_BIT, FLOODFILL_WALL_EAST_BIT, FLOODFILL_WALL_SOUTH_BIT, or FLOODFILL_WALL_WEST_BIT
 * @details
 *   Writes wall to current cell AND the adjacent cell.
 *
 *   Example: NORTH wall at (3,5) also writes SOUTH wall at (3,6).
 *   This is because walls are shared between cells:
 *   - The north edge of cell (3,5) is the south edge of cell (3,6)
 *
 *   Without this symmetry, BFS would think cells on the other side
 *   are open when they're actually blocked.
 *
 *   Bounds checks prevent writing outside the 16x16 maze.
 */
static void FloodFill_SetWall(uint8_t x, uint8_t y, floodfill_wall_bit_t wall_bit)
{
    walls[x][y] |= (uint8_t)wall_bit;

    switch(wall_bit)
    {
        case (FLOODFILL_WALL_NORTH_BIT):
            if (y + 1 < MAZE_SIZE)
                walls[x][y + 1] |= (uint8_t)FLOODFILL_WALL_SOUTH_BIT;
            break;

        case (FLOODFILL_WALL_EAST_BIT):
            if (x + 1 < MAZE_SIZE)
                walls[x + 1][y] |= (uint8_t)FLOODFILL_WALL_WEST_BIT;
            break;

        case (FLOODFILL_WALL_SOUTH_BIT):
            if (y > 0)
                walls[x][y - 1] |= (uint8_t)FLOODFILL_WALL_NORTH_BIT;
            break;

        case (FLOODFILL_WALL_WEST_BIT):
            if (x > 0)
                walls[x - 1][y] |= (uint8_t)FLOODFILL_WALL_EAST_BIT;
            break;

        default:
            break;
    }
}

/* ==========================================================================
 *   Scan Walls
 * ========================================================================== */

/**
 * @brief  Sample IR sensors and store discovered walls.
 * @details
 *   Reads all 4 IR sensors and detects walls:
 *   - FRONT wall: Both IR_LEFT and IR_RIGHT detect a wall
 *   - LEFT wall:  IR_FAR_LEFT detects a wall
 *   - RIGHT wall: IR_FAR_RIGHT detects a wall
 *
 *   Then converts relative walls (front/left/right) to absolute walls
 *   (NORTH/EAST/SOUTH/WEST) based on mouse_heading.
 *
 *   Stores walls using FloodFill_SetWall() which also writes
 *   symmetric walls to neighboring cells.
 */
static void FloodFill_ScanWalls(void)
{
    bool front, left, right;

    IR_SampleAll();

    front = IR_IsWallPresent(IR_LEFT) && IR_IsWallPresent(IR_RIGHT);
    left  = IR_IsWallPresent(IR_FAR_LEFT);
    right = IR_IsWallPresent(IR_FAR_RIGHT);

    if (front) FloodFill_SetWall(mouse_x, mouse_y, (floodfill_wall_bit_t)FloodFill_GetFrontWall(mouse_heading));
    if (left)  FloodFill_SetWall(mouse_x, mouse_y, (floodfill_wall_bit_t)FloodFill_GetLeftWall(mouse_heading));
    if (right) FloodFill_SetWall(mouse_x, mouse_y, (floodfill_wall_bit_t)FloodFill_GetRightWall(mouse_heading));
}

/* ==========================================================================
 *   BFS Update
 * ========================================================================== */

/**
 * @brief  Recalculate distances from the 2x2 goal area using BFS.
 * @details
 *   This is the core BFS (Breadth-First Search) flood fill algorithm.
 *
 *   Steps:
 *   1. Reset all distances to CELL_UNVISITED (255)
 *   2. Start with 2x2 goal cells: distance = 0
 *   3. Enqueue all four goal cells
 *   4. While queue not empty:
 *      a. Dequeue a cell
 *      b. For each of 4 neighbors:
 *         - Check if neighbor is within maze bounds
 *         - Check if wall blocks the direction
 *         - If no wall and unvisited (dist == 255):
 *             Set dist[neighbor] = dist[current] + 1
 *             Enqueue neighbor
 *
 *   Result: Each reachable cell has its shortest distance to nearest goal tile.
 *   The mouse picks the neighbor with the lowest dist value.
 */
static void FloodFill_Update(void)
{
    uint8_t x, y;
    cell_t cell;

    /* Reset all Distances */
    for (x = 0; x < MAZE_SIZE; x++)
    {
        for (y = 0; y < MAZE_SIZE; y++)
        {
            dist[x][y] = CELL_UNVISITED;
        }
    }

    /* [1] Start BFS from 2x2 goal area */
    Queue_Init();
    Queue_Enqueue(GOAL_MIN_X, GOAL_MIN_Y);  dist[GOAL_MIN_X][GOAL_MIN_Y] = 0;
    Queue_Enqueue(GOAL_MAX_X, GOAL_MIN_Y);  dist[GOAL_MAX_X][GOAL_MIN_Y] = 0;
    Queue_Enqueue(GOAL_MIN_X, GOAL_MAX_Y);  dist[GOAL_MIN_X][GOAL_MAX_Y] = 0;
    Queue_Enqueue(GOAL_MAX_X, GOAL_MAX_Y);  dist[GOAL_MAX_X][GOAL_MAX_Y] = 0;

    /* Process queue */
    while (Queue_Dequeue(&cell))            /* Pop the front cell */
    {
        x = cell.x; y = cell.y;

        /* NORTH neighbor */
        if ((y + 1 < MAZE_SIZE) && ((walls[x][y] & FLOODFILL_WALL_NORTH_BIT) == 0))
        {
            if (dist[x][y + 1] == CELL_UNVISITED)
            {
                dist[x][y + 1] = dist[x][y] + 1;
                Queue_Enqueue(x, y + 1);
            }
        }

        /* EAST neighbor */
        if ((x + 1 < MAZE_SIZE) && ((walls[x][y] & FLOODFILL_WALL_EAST_BIT) == 0))
        {
            if (dist[x + 1][y] == CELL_UNVISITED)
            {
                dist[x + 1][y] = dist[x][y] + 1;
                Queue_Enqueue(x + 1, y);
            }
        }

        /* SOUTH neighbor */
        if ((y > 0) && ((walls[x][y] & FLOODFILL_WALL_SOUTH_BIT) == 0))
        {
            if (dist[x][y - 1] == CELL_UNVISITED)
            {
                dist[x][y - 1] = dist[x][y] + 1;
                Queue_Enqueue(x, y - 1);
            }
        }

        /* WEST neighbor */
        if ((x > 0) && ((walls[x][y] & FLOODFILL_WALL_WEST_BIT) == 0))
        {
            if (dist[x - 1][y] == CELL_UNVISITED)
            {
                dist[x - 1][y] = dist[x][y] + 1;
                Queue_Enqueue(x - 1, y);
            }
        }
    }
}

/* ==========================================================================
 *   Pick Next Direction
 * ========================================================================== */

/**
 * @brief  Pick the neighbor with the lowest distance to goal.
 * @return flood_dir_t  Best direction to move (FLOODFILL_DIR_NORTH/EAST/SOUTH/WEST)
 * @details
 *   Checks all 4 neighbors of the current cell.
 *   - Must be within maze bounds (0-15)
 *   - Must not be blocked by a known wall
 *   - Picks the one with the smallest dist value
 *
 *   If multiple neighbors have the same lowest distance,
 *   the first one checked wins (FLOODFILL_DIR_NORTH, EAST, SOUTH, WEST order).
 *
 *   The mouse always moves "downhill" toward lower distances,
 *   which guarantees it's following the shortest path to goal.
 */
static floodfill_dir_t FloodFill_GetNextDir(void)
{
    uint8_t best_dist;
    floodfill_dir_t best_dir;

    /* Start with worst-case values so any valid neighbor beats them */
    best_dist = CELL_UNVISITED;   /* 255 = "unreachable" */
    best_dir  = mouse_heading;    /* If no neighbor is better, keep going forward */

    /* ------------------------------------------------------------------
     *  Check NORTH neighbor (x, y+1)
     *  - Is y+1 still inside the maze?
     *  - Is the NORTH wall bit clear in the current cell?
     *  - Is its distance lower than what we have seen so far?
     * ------------------------------------------------------------------ */
    if ((mouse_y + 1 < MAZE_SIZE) && ((walls[mouse_x][mouse_y] & FLOODFILL_WALL_NORTH_BIT) == 0))
    {
        if (dist[mouse_x][mouse_y + 1] < best_dist)
        {
            best_dist = dist[mouse_x][mouse_y + 1];
            best_dir  = FLOODFILL_DIR_NORTH;
        }
    }

    /* ------------------------------------------------------------------
     *  Check EAST neighbor (x+1, y)
     *  - Is x+1 still inside the maze?
     *  - Is the EAST wall bit clear in the current cell?
     *  - Is its distance lower than the current best?
     * ------------------------------------------------------------------ */
    if ((mouse_x + 1 < MAZE_SIZE) && ((walls[mouse_x][mouse_y] & FLOODFILL_WALL_EAST_BIT) == 0))
    {
        if (dist[mouse_x + 1][mouse_y] < best_dist)
        {
            best_dist = dist[mouse_x + 1][mouse_y];
            best_dir  = FLOODFILL_DIR_EAST;
        }
    }

    /* ------------------------------------------------------------------
     *  Check SOUTH neighbor (x, y-1)
     *  - Is y-1 >= 0?
     *  - Is the SOUTH wall bit clear in the current cell?
     *  - Is its distance lower than the current best?
     * ------------------------------------------------------------------ */
    if ((mouse_y > 0) && ((walls[mouse_x][mouse_y] & FLOODFILL_WALL_SOUTH_BIT) == 0))
    {
        if (dist[mouse_x][mouse_y - 1] < best_dist)
        {
            best_dist = dist[mouse_x][mouse_y - 1];
            best_dir  = FLOODFILL_DIR_SOUTH;
        }
    }

    /* ------------------------------------------------------------------
     *  Check WEST neighbor (x-1, y)
     *  - Is x-1 >= 0?
     *  - Is the WEST wall bit clear in the current cell?
     *  - Is its distance lower than the current best?
     * ------------------------------------------------------------------ */
    if ((mouse_x > 0) && ((walls[mouse_x][mouse_y] & FLOODFILL_WALL_WEST_BIT) == 0))
    {
        if (dist[mouse_x - 1][mouse_y] < best_dist)
        {
            best_dist = dist[mouse_x - 1][mouse_y];
            best_dir  = FLOODFILL_DIR_WEST;
        }
    }

    return best_dir;
}

/* ==========================================================================
 *   Public Functions
 * ========================================================================== */

/**
 * @brief  Initialize flood fill with Manhattan distances to nearest goal tile.
 * @details
 *   The IEEE goal is a 2x2 block at center (7,7) to (8,8).
 *   For each cell, dx = distance to nearest x-edge of the block,
 *   dy = distance to nearest y-edge. dist = dx + dy.
 *
 *   All four center cells get distance 0.
 *   Walls are cleared and mouse starts at (0,0) facing NORTH.
 */
void FloodFill_Init(void)
{
    uint8_t x, y;
    uint8_t dx, dy;

    mouse_x = 0;
    mouse_y = 0;
    mouse_heading = FLOODFILL_DIR_NORTH;

    for (x = 0; x < MAZE_SIZE; x++)
    {
        for (y = 0; y < MAZE_SIZE; y++)
        {
            walls[x][y] = 0;

            /* Distance to nearest edge of the goal block, not a single point */
            if (x < GOAL_MIN_X)      
                dx = GOAL_MIN_X - x;
            else if (x > GOAL_MAX_X) 
                dx = x - GOAL_MAX_X;
            else                     
                dx = 0;

            if (y < GOAL_MIN_Y)      
                dy = GOAL_MIN_Y - y;
            else if (y > GOAL_MAX_Y) 
                dy = y - GOAL_MAX_Y;
            else                     
                dy = 0;

            dist[x][y] = dx + dy;
        }
    }
}

/**
 * @brief  Check if the mouse is inside the 2x2 goal area.
 * @return bool  true if mouse_x is 7 or 8 AND mouse_y is 7 or 8.
 */
bool FloodFill_IsAtGoal(void)
{
    return ((mouse_x == GOAL_MIN_X || mouse_x == GOAL_MAX_X) &&
            (mouse_y == GOAL_MIN_Y || mouse_y == GOAL_MAX_Y));
}

/**
 * @brief  Plan the next move.
 * @return flood_action_t  Action for Navigator to execute.
 * @details
 *   1. FloodFill_ScanWalls():
 *      - Sample IR sensors
 *      - Detect front/left/right walls
 *      - Convert to absolute (N/E/S/W)
 *      - Store in walls array
 *
 *   2. FloodFill_Update():
 *      - Reset all distances to 255
 *      - BFS from 2x2 goal area
 *      - Fill dist[][] with shortest distances
 *
 *   3. FloodFill_GetNextDir():
 *      - Check 4 neighbors
 *      - Pick one with lowest dist
 *
 *   4. Compare next_dir to mouse_heading:
 *      - Same: FLOODFILL_FORWARD
 *      - 90° CCW: FLOODFILL_TURN_LEFT
 *      - 90° CW: FLOODFILL_TURN_RIGHT
 *      - 180°: FLOODFILL_TURN_AROUND
 *
 *   Does NOT start any motion. Navigator must execute and report done.
 */
floodfill_t FloodFill_Plan(void)
{
    floodfill_dir_t next_dir;

    if (FloodFill_IsAtGoal())
        return FLOODFILL_STOP;

    FloodFill_ScanWalls();
    FloodFill_Update();
    next_dir = FloodFill_GetNextDir();

    /* Update internal heading immediately. Position only updates on ReportDone. */
    if (next_dir == mouse_heading)
    {
        return FLOODFILL_FORWARD;
    }
    else if ((next_dir == FLOODFILL_DIR_NORTH && mouse_heading == FLOODFILL_DIR_EAST)  ||
             (next_dir == FLOODFILL_DIR_EAST  && mouse_heading == FLOODFILL_DIR_SOUTH) ||
             (next_dir == FLOODFILL_DIR_SOUTH && mouse_heading == FLOODFILL_DIR_WEST)  ||
             (next_dir == FLOODFILL_DIR_WEST  && mouse_heading == FLOODFILL_DIR_NORTH))
    {
        mouse_heading = next_dir;
        return FLOODFILL_TURN_LEFT;
    }
    else if ((next_dir == FLOODFILL_DIR_NORTH && mouse_heading == FLOODFILL_DIR_WEST)  ||
             (next_dir == FLOODFILL_DIR_WEST  && mouse_heading == FLOODFILL_DIR_SOUTH) ||
             (next_dir == FLOODFILL_DIR_SOUTH && mouse_heading == FLOODFILL_DIR_EAST)  ||
             (next_dir == FLOODFILL_DIR_EAST  && mouse_heading == FLOODFILL_DIR_NORTH))
    {
        mouse_heading = next_dir;
        return FLOODFILL_TURN_RIGHT;
    }
    else
    {
        mouse_heading = next_dir;
        return FLOODFILL_TURN_AROUND;
    }
}

/**
 * @brief  Report that the last planned action is physically complete. Updates position of the mouse.
 * @param  action  The action that was executed.
 * @details
 *   Only FLOODFILL_FORWARD changes cell position.
 *   Turns only changed heading, already done in Plan().
 */
void FloodFill_ReportDone(floodfill_t action)
{
    if (action == FLOODFILL_FORWARD)
    {
        switch (mouse_heading)
        {
            case FLOODFILL_DIR_NORTH: mouse_y++; break;
            case FLOODFILL_DIR_EAST:  mouse_x++; break;
            case FLOODFILL_DIR_SOUTH: mouse_y--; break;
            case FLOODFILL_DIR_WEST:  mouse_x--; break;
        }
    }
    /* LEFT, RIGHT, UTURN: heading already updated in Plan(), no position change */
}