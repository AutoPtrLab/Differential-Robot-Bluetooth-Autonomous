
// Search algorithm helper for the global navigation
#ifndef SEARCH_HELPER_H
#define SEARCH_HELPER_H
#include "navigation.h"
#include "occupancy_grid.h"
#include <stdint.h>
// RAM LEFT :1520 bytes of ram left
//
// The idea of the frontier searcher is first using a BFS (Breadth First Search) begginning in
// the robot  to search for the first unexplored tile of our map (the map is scaled so we
// consider the 4 tiles in the uint16_t as a unique node) . When we find the nearest
// unexplored tile, since we dont have enough memory we cannot save the parents to have the
// complete path, we will use a JPS (Jump Point Search) to obtain the path waypoints to the
// desired position since JPS only saves the Jump Points, which are more or less straight and will
// be a feasable objective for the local planner

// frontier searcher , the x and y has to be scaled in 56 * 56
uint16_t BFS_frontier_search(const uint16_t grid[GRID_SIZE][GRID_SIZE], uint8_t x, uint8_t y);

// jump points search
uint8_t JPS_find_path(const uint16_t grid[GRID_SIZE][GRID_SIZE], uint8_t x_start, uint8_t y_start,
                      uint8_t x_goal, uint8_t y_goal, uint16_t waypoints[MAX_WAYPOINTS]);

#endif // SEARCH_HELPER_H
