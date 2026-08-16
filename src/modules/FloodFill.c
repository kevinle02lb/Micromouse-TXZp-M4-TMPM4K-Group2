/**
 * @file        FloodFill.c
 * @brief       Flood fill maze solver for an IEEE 16x16 micromouse.
 * @version     V1.0.0
 * @date        15-08-2026
 *
 * @details
 *   IEEE goal area: 2x2 block at maze center (cells 7-8, 7-8).
 *   BFS seeds all four goal cells simultaneously so every reachable cell
 *   stores its shortest distance to the nearest goal tile.
 *
 *   Pure planner. No hardware access and no motion.
 *
 *   Algorithm Flow:
 *   1. Navigator calls FloodFill_SetWalls() with front/left/right
 *   2. FloodFill_Plan():
 *      a. BFS from the 2x2 goal area to recalculate distances
 *      b. Pick lowest-distance neighbor
 *      c. Return action: FORWARD / LEFT / RIGHT / UTURN / STOP
 *   3. Navigator executes turn/drive physically
 *   4. Navigator calls FloodFill_ReportDone() when arrived
 *   5. Repeat until goal reached
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

/* ==========================================================================
 *   Goal Area
 * ========================================================================== */

#define GOAL_MIN_X   7U
#define GOAL_MIN_Y   7U
#define GOAL_MAX_X   8U
#define GOAL_MAX_Y   8U

/* ==========================================================================
 *   Private Data
 * ========================================================================== */

/**
 * @brief  Wall mask per cell, one bit per cardinal direction.
 */
static uint8_t walls[MAZE_SIZE][MAZE_SIZE];

/**
 * @brief  Distance from each cell to the nearest goal tile.
 * @note   CELL_UNVISITED marks cells BFS could not reach.
 */
static uint8_t dist[MAZE_SIZE][MAZE_SIZE];

static uint8_t         mouse_x;
static uint8_t         mouse_y;
static floodfill_dir_t mouse_heading;

/* ==========================================================================
 *   BFS Queue
 * ========================================================================== */

#define QUEUE_SIZE  (MAZE_SIZE * MAZE_SIZE)

typedef struct
{
    uint8_t x;
    uint8_t y;
} cell_t;

static cell_t   queue[QUEUE_SIZE];
static uint16_t queue_head;
static uint16_t queue_tail;

/**
 * @brief  Reset the queue to empty.
 */
static void Queue_Init(void)
{
    queue_head = 0U;
    queue_tail = 0U;
}

/**
 * @brief  Append a cell to the back of the queue.
 * @param  x  Cell x coordinate.
 * @param  y  Cell y coordinate.
 * @note   Silently drops the cell if the queue is full, which cannot occur
 *         while BFS visits each cell at most once.
 */
static void Queue_Enqueue(uint8_t x, uint8_t y)
{
    if (queue_tail < QUEUE_SIZE)
    {
        queue[queue_tail].x = x;
        queue[queue_tail].y = y;
        queue_tail++;
    }
}

/**
 * @brief  Remove a cell from the front of the queue.
 * @param  cell  Destination for the dequeued cell.
 * @return bool  false when the queue is empty.
 */
static bool Queue_Dequeue(cell_t *cell)
{
    if (queue_head == queue_tail)
        return false;

    cell->x = queue[queue_head].x;
    cell->y = queue[queue_head].y;
    queue_head++;

    return true;
}

/* ==========================================================================
 *   Direction Helpers
 *
 *   The direction enum is ordered clockwise, which reduces every rotation to
 *   modular arithmetic.
 * ========================================================================== */

/**
 * @brief  Direction 90 degrees clockwise from the argument.
 */
static floodfill_dir_t Dir_Right(floodfill_dir_t dir)
{
    return (floodfill_dir_t)(((uint8_t)dir + 1U) & 3U);
}

/**
 * @brief  Direction 180 degrees from the argument.
 */
static floodfill_dir_t Dir_Back(floodfill_dir_t dir)
{
    return (floodfill_dir_t)(((uint8_t)dir + 2U) & 3U);
}

/**
 * @brief  Direction 90 degrees counter-clockwise from the argument.
 */
static floodfill_dir_t Dir_Left(floodfill_dir_t dir)
{
    return (floodfill_dir_t)(((uint8_t)dir + 3U) & 3U);
}

/**
 * @brief  Wall bit corresponding to an absolute direction.
 * @param  dir  Absolute direction.
 * @return uint8_t  Matching wall bit.
 */
static uint8_t Dir_ToWallBit(floodfill_dir_t dir)
{
    switch (dir)
    {
        case FLOODFILL_DIR_NORTH: return (uint8_t)FLOODFILL_WALL_NORTH_BIT;
        case FLOODFILL_DIR_EAST:  return (uint8_t)FLOODFILL_WALL_EAST_BIT;
        case FLOODFILL_DIR_SOUTH: return (uint8_t)FLOODFILL_WALL_SOUTH_BIT;
        case FLOODFILL_DIR_WEST:  return (uint8_t)FLOODFILL_WALL_WEST_BIT;
        default:                  return (uint8_t)FLOODFILL_WALL_NORTH_BIT;
    }
}

/* ==========================================================================
 *   Wall Management
 * ========================================================================== */

/**
 * @brief  Record a wall in a cell and mirror it into the neighbour.
 * @param  x    Cell x coordinate.
 * @param  y    Cell y coordinate.
 * @param  dir  Absolute direction of the wall from that cell.
 * @details
 *   A wall is shared between two cells: the north edge of (3,5) is the south
 *   edge of (3,6). Without the mirrored write, BFS would treat the far side
 *   as open. Bounds checks suppress the mirror at the maze perimeter.
 */
static void FloodFill_SetWall(uint8_t x, uint8_t y, floodfill_dir_t dir)
{
    walls[x][y] |= Dir_ToWallBit(dir);

    switch (dir)
    {
        case FLOODFILL_DIR_NORTH:
            if ((y + 1U) < MAZE_SIZE)
                walls[x][y + 1U] |= (uint8_t)FLOODFILL_WALL_SOUTH_BIT;
            break;

        case FLOODFILL_DIR_EAST:
            if ((x + 1U) < MAZE_SIZE)
                walls[x + 1U][y] |= (uint8_t)FLOODFILL_WALL_WEST_BIT;
            break;

        case FLOODFILL_DIR_SOUTH:
            if (y > 0U)
                walls[x][y - 1U] |= (uint8_t)FLOODFILL_WALL_NORTH_BIT;
            break;

        case FLOODFILL_DIR_WEST:
            if (x > 0U)
                walls[x - 1U][y] |= (uint8_t)FLOODFILL_WALL_EAST_BIT;
            break;

        default:
            break;
    }
}

/* ==========================================================================
 *   BFS Distance Field
 * ========================================================================== */

/**
 * @brief  Recalculate the distance field from the 2x2 goal area.
 * @details
 *   All four goal cells are seeded at distance zero and expanded level by
 *   level. A neighbour is reached only when it lies inside the maze, is not
 *   separated by a known wall, and has not already been assigned a distance.
 *   Cells left at CELL_UNVISITED are unreachable given the walls discovered
 *   so far.
 */
static void FloodFill_Update(void)
{
    uint8_t x;
    uint8_t y;
    cell_t  cell;

    for (x = 0U; x < MAZE_SIZE; x++)
    {
        for (y = 0U; y < MAZE_SIZE; y++)
            dist[x][y] = CELL_UNVISITED;
    }

    Queue_Init();

    dist[GOAL_MIN_X][GOAL_MIN_Y] = 0U;  Queue_Enqueue(GOAL_MIN_X, GOAL_MIN_Y);
    dist[GOAL_MAX_X][GOAL_MIN_Y] = 0U;  Queue_Enqueue(GOAL_MAX_X, GOAL_MIN_Y);
    dist[GOAL_MIN_X][GOAL_MAX_Y] = 0U;  Queue_Enqueue(GOAL_MIN_X, GOAL_MAX_Y);
    dist[GOAL_MAX_X][GOAL_MAX_Y] = 0U;  Queue_Enqueue(GOAL_MAX_X, GOAL_MAX_Y);

    while (Queue_Dequeue(&cell))
    {
        x = cell.x;
        y = cell.y;

        if (((y + 1U) < MAZE_SIZE) &&
            ((walls[x][y] & FLOODFILL_WALL_NORTH_BIT) == 0U) &&
            (dist[x][y + 1U] == CELL_UNVISITED))
        {
            dist[x][y + 1U] = dist[x][y] + 1U;
            Queue_Enqueue(x, y + 1U);
        }

        if (((x + 1U) < MAZE_SIZE) &&
            ((walls[x][y] & FLOODFILL_WALL_EAST_BIT) == 0U) &&
            (dist[x + 1U][y] == CELL_UNVISITED))
        {
            dist[x + 1U][y] = dist[x][y] + 1U;
            Queue_Enqueue(x + 1U, y);
        }

        if ((y > 0U) &&
            ((walls[x][y] & FLOODFILL_WALL_SOUTH_BIT) == 0U) &&
            (dist[x][y - 1U] == CELL_UNVISITED))
        {
            dist[x][y - 1U] = dist[x][y] + 1U;
            Queue_Enqueue(x, y - 1U);
        }

        if ((x > 0U) &&
            ((walls[x][y] & FLOODFILL_WALL_WEST_BIT) == 0U) &&
            (dist[x - 1U][y] == CELL_UNVISITED))
        {
            dist[x - 1U][y] = dist[x][y] + 1U;
            Queue_Enqueue(x - 1U, y);
        }
    }
}

/**
 * @brief  Select the open neighbour with the lowest distance to goal.
 * @return floodfill_dir_t  Absolute direction of the chosen neighbour.
 * @details
 *   Neighbours are tested in the order NORTH, EAST, SOUTH, WEST and the first
 *   strictly-lower distance wins, so ties resolve to the earliest in that
 *   order. If every neighbour is walled or unreachable the current heading is
 *   returned; that state implies the mouse has sealed itself into a region
 *   and cannot occur while the maze is connected.
 */
static floodfill_dir_t FloodFill_GetNextDir(void)
{
    uint8_t         best_dist = CELL_UNVISITED;
    floodfill_dir_t best_dir  = mouse_heading;

    if (((mouse_y + 1U) < MAZE_SIZE) &&
        ((walls[mouse_x][mouse_y] & FLOODFILL_WALL_NORTH_BIT) == 0U) &&
        (dist[mouse_x][mouse_y + 1U] < best_dist))
    {
        best_dist = dist[mouse_x][mouse_y + 1U];
        best_dir  = FLOODFILL_DIR_NORTH;
    }

    if (((mouse_x + 1U) < MAZE_SIZE) &&
        ((walls[mouse_x][mouse_y] & FLOODFILL_WALL_EAST_BIT) == 0U) &&
        (dist[mouse_x + 1U][mouse_y] < best_dist))
    {
        best_dist = dist[mouse_x + 1U][mouse_y];
        best_dir  = FLOODFILL_DIR_EAST;
    }

    if ((mouse_y > 0U) &&
        ((walls[mouse_x][mouse_y] & FLOODFILL_WALL_SOUTH_BIT) == 0U) &&
        (dist[mouse_x][mouse_y - 1U] < best_dist))
    {
        best_dist = dist[mouse_x][mouse_y - 1U];
        best_dir  = FLOODFILL_DIR_SOUTH;
    }

    if ((mouse_x > 0U) &&
        ((walls[mouse_x][mouse_y] & FLOODFILL_WALL_WEST_BIT) == 0U) &&
        (dist[mouse_x - 1U][mouse_y] < best_dist))
    {
        best_dist = dist[mouse_x - 1U][mouse_y];
        best_dir  = FLOODFILL_DIR_WEST;
    }

    return best_dir;
}

/* ==========================================================================
 *   Public Functions
 * ========================================================================== */

/**
 * @brief  Clear the map and seed the distance field with Manhattan estimates.
 * @details
 *   Distances are measured to the nearest edge of the 2x2 goal block rather
 *   than to a single centre point, so all four goal cells start at zero. The
 *   estimate is replaced by the true BFS field on the first Plan call; it
 *   exists so the field is never read uninitialised.
 *
 *   The mouse is placed at (0,0) facing NORTH. See the header for the
 *   physical start pose this implies.
 */
void FloodFill_Init(void)
{
    uint8_t x;
    uint8_t y;
    uint8_t dx;
    uint8_t dy;

    mouse_x       = 0U;
    mouse_y       = 0U;
    mouse_heading = FLOODFILL_DIR_NORTH;

    for (x = 0U; x < MAZE_SIZE; x++)
    {
        for (y = 0U; y < MAZE_SIZE; y++)
        {
            walls[x][y] = 0U;

            if (x < GOAL_MIN_X)
                dx = GOAL_MIN_X - x;
            else if (x > GOAL_MAX_X)
                dx = x - GOAL_MAX_X;
            else
                dx = 0U;

            if (y < GOAL_MIN_Y)
                dy = GOAL_MIN_Y - y;
            else if (y > GOAL_MAX_Y)
                dy = y - GOAL_MAX_Y;
            else
                dy = 0U;

            dist[x][y] = dx + dy;
        }
    }
}

/**
 * @brief  Record the walls observed from the current cell.
 * @param  front  Wall directly ahead of the robot.
 * @param  left   Wall to the robot's left.
 * @param  right  Wall to the robot's right.
 * @note   Called by the Navigator once the robot is stationary and centred.
 *         Walls are additive: the absence of a wall is never recorded, so a
 *         false positive persists for the rest of the run.
 */
void FloodFill_SetWalls(bool front, bool left, bool right)
{
    if (front)
        FloodFill_SetWall(mouse_x, mouse_y, mouse_heading);

    if (left)
        FloodFill_SetWall(mouse_x, mouse_y, Dir_Left(mouse_heading));

    if (right)
        FloodFill_SetWall(mouse_x, mouse_y, Dir_Right(mouse_heading));
}

/**
 * @brief  Test whether the mouse is inside the 2x2 goal area.
 * @return bool  true when the believed cell is one of the four goal tiles.
 */
bool FloodFill_IsAtGoal(void)
{
    return (((mouse_x == GOAL_MIN_X) || (mouse_x == GOAL_MAX_X)) &&
            ((mouse_y == GOAL_MIN_Y) || (mouse_y == GOAL_MAX_Y)));
}

/**
 * @brief  Decide the next move.
 * @return floodfill_t  Action for the Navigator to execute.
 * @details
 *   Rebuilds the distance field from the walls discovered so far, selects the
 *   lowest-distance open neighbour, and expresses that direction relative to
 *   the current heading.
 *
 *   No internal state is modified. The planner remains consistent with the
 *   robot's true position until FloodFill_ReportDone confirms the move.
 */
floodfill_t FloodFill_Plan(void)
{
    floodfill_dir_t next_dir;

    if (FloodFill_IsAtGoal())
        return FLOODFILL_STOP;

    FloodFill_Update();
    next_dir = FloodFill_GetNextDir();

    if (next_dir == mouse_heading)
        return FLOODFILL_FORWARD;

    if (next_dir == Dir_Right(mouse_heading))
        return FLOODFILL_TURN_RIGHT;

    if (next_dir == Dir_Left(mouse_heading))
        return FLOODFILL_TURN_LEFT;

    return FLOODFILL_TURN_AROUND;
}

/**
 * @brief  Apply a completed move to the planner state.
 * @param  action  The action that was physically executed.
 * @details
 *   Applies the heading change implied by the action, then advances one cell
 *   in the new heading. Every action other than FLOODFILL_STOP ends one cell
 *   ahead, because the Navigator drives into the next cell after a turn.
 */
void FloodFill_ReportDone(floodfill_t action)
{
    switch (action)
    {
        case FLOODFILL_FORWARD:
            break;

        case FLOODFILL_TURN_LEFT:
            mouse_heading = Dir_Left(mouse_heading);
            break;

        case FLOODFILL_TURN_RIGHT:
            mouse_heading = Dir_Right(mouse_heading);
            break;

        case FLOODFILL_TURN_AROUND:
            mouse_heading = Dir_Back(mouse_heading);
            break;

        case FLOODFILL_STOP:
        default:
            return;
    }

    switch (mouse_heading)
    {
        case FLOODFILL_DIR_NORTH: mouse_y++; break;
        case FLOODFILL_DIR_EAST:  mouse_x++; break;
        case FLOODFILL_DIR_SOUTH: mouse_y--; break;
        case FLOODFILL_DIR_WEST:  mouse_x--; break;
        default:                             break;
    }
}

/**
 * @brief  Get the believed cell x coordinate.
 * @return uint8_t  Cell x in the range 0 to MAZE_SIZE - 1.
 */
uint8_t FloodFill_GetX(void)
{
    return mouse_x;
}

/**
 * @brief  Get the believed cell y coordinate.
 * @return uint8_t  Cell y in the range 0 to MAZE_SIZE - 1.
 */
uint8_t FloodFill_GetY(void)
{
    return mouse_y;
}