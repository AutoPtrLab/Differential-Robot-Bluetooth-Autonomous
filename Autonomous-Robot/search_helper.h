
// Search algorithm helper for the global navigation
#ifndef SEARCH_HELPER_H
#define SEARCH_HELPER_H
#include "occupancy_grid.h"
#include <stdint.h>
// frontier searcher , the x and y has to be scaled in 56 * 56
uint16_t BFS_frontier_search(const uint16_t grid[GRID_SIZE][GRID_SIZE], uint8_t x, uint8_t y);

uint16_t JPS_find_path(const uint16_t grid[GRID_SIZE][GRID_SIZE], uint8_t x_goal, uint8_t y_goal,
                       uint8_t x_init, uint8_t y_init);

#endif // SEARCH_HELPER_H
