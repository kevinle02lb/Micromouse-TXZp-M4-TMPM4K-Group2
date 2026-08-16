/**
 * @file        Navigator.h
 * @brief       Robot behavior logic handler. combines FloodFill, optometry, & motion controls
 * @version     V1.0.0
 * @date        15-08-2026
 *
 * @details
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */
#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "FloodFill.h"   /* for floodfill_t */

void Navigator_Init(void);
void Navigator_Update(void);

#endif /* NAVIGATOR_H */