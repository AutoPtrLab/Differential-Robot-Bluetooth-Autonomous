// helper for the navigation module to use the spacial _grid operations
#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H
#include "math_helper.h"
#include <stdint.h>
#define GRID_SIZE 56
// this is equal to grdis size since each position in the matrix contains 4 positions
#define HALF_SIZE (GRID_SIZE)
// the start of the grid map message
#define START_MAP_BYTE 0xAB

// in the occupancy _grid,we are representing each 12.5 cm squares with half byte
//  right now we can think of this as a _grid that each of the componentes can divide in 4 more
//  squares
//
//  [ ] = [ x even, y odd ][x odd ,y odd ]
//        [x even, y even ][x odd,y even ]
//
//
//

extern uint16_t _grid[GRID_SIZE][GRID_SIZE];

extern const uint16_t NIBBLE_MASK[4];
extern const uint8_t NIBBLE_SHIFT[4];

// wrapper for the bluetooth sender,is blocking and it will last a lot, so
// it can only be done when the robot has stopped its trajectory
void send_mapgrid(void);
// initialize grid to 7
static inline void init_grid() {
    for (int8_t i = GRID_SIZE - 1; i >= 0; i--) {
        for (int8_t j = GRID_SIZE - 1; j >= 0; j--) {
            _grid[i][j] = 0x7777; // init all the values to a non deterministic one
        }
    }
}
// gets the _grid information from the bresenham information
static inline uint8_t get_grid_from_bresenham(int16_t x, int16_t y) {
    if (x < 0 || x >= (GRID_SIZE * 2) || y < 0 || y >= (GRID_SIZE * 2)) {
        return 7; // returns the default value if the matrix is out of bounds
    }
    uint8_t id = (uint8_t)((y & 1) << 1) | (x & 1); // this checks if is even or odd
    uint8_t val = (uint8_t)((_grid[y >> 1][x >> 1] & NIBBLE_MASK[id]) >> NIBBLE_SHIFT[id]);
    return val;
}
// gets the _grid information from the q16.16
static inline uint8_t get_grid_from_Vec2(Vec2 position) {
    return get_grid_from_bresenham((int16_t)(position.x >> 13) + HALF_SIZE,
                                   (int16_t)(position.y >> 13) + HALF_SIZE);
}

static inline void _update_cell(int16_t x, int16_t y, int8_t delta) {
    // out of bounds check
    if (x < 0 || x >= (GRID_SIZE * 2) || y < 0 || y >= (GRID_SIZE * 2)) {
        return;
    }

    // since this is the mid line it indicates that there is no obstacle
    uint8_t idx = ((y & 1) << 1) | (x & 1); // this checks if x and y are odd or even
    uint16_t *block = &_grid[y >> 1][x >> 1];
    int16_t val = ((*block & NIBBLE_MASK[idx]) >> NIBBLE_SHIFT[idx]) + delta;

    // saturation arithmetic
    if (val > 15)
        val = 15;
    if (val < 0)
        val = 0;
    // we return the value to its position
    *block = (*block & ~NIBBLE_MASK[idx]) | ((uint16_t)val << NIBBLE_SHIFT[idx]);
}

// obstacle raycasting using the Bresenham algorithm
inline static void obstacle_update(const Vec2 *const origin, const Vec2 *const dst) {

    // since each cell of the _grid is 12.5 cm or 0.125 m (1/8 m)  we have to scale our vectors that
    // represent meteres in the Q16.16 format to the _grid formta , and then we strip the decimals
    // from the numeber dividing by 2^16 ( num << 3 then num >> 16 = num >>13)

    // this might overflow but only if we reach 8192 meters , so we use a int16 to make the
    // operations quicker

    int16_t x1 = (int16_t)(origin->x >> 13) + HALF_SIZE;
    int16_t y1 = (int16_t)(origin->y >> 13) + HALF_SIZE;
    int16_t x2 = (int16_t)(dst->x >> 13) + HALF_SIZE;
    int16_t y2 = (int16_t)(dst->y >> 13) + HALF_SIZE;

    int16_t dx = x2 - x1;
    int16_t dy = y2 - y1;
    // now dx and dy

    int8_t inc_yi = 0;
    int8_t inc_xi = 0;
    int8_t inc_yr = 0;
    int8_t inc_xr = 0;
    if (dy >= 0) {
        inc_yi = 1;
    } else {
        dy = -dy;
        inc_yi = -1;
    }

    if (dx >= 0) {
        inc_xi = 1;
    } else {
        dx = -dx;
        inc_xi = -1;
    }

    if (dx >= dy) {
        inc_yr = 0;
        inc_xr = inc_xi;
    } else {
        inc_xr = 0;
        inc_yr = inc_yi;

        int16_t aux = dx;
        dx = dy;
        dy = aux;
    }
    int16_t avr = 2 * dy;
    int16_t av = (avr - dx);
    int16_t avi = (av - dx);
    int16_t x = x1;
    int16_t y = y1;
    while (x != x2 || y != y2) {
        _update_cell(x, y, -1);
        if (av >= 0) {
            x += inc_xi;
            y += inc_yi;
            av = (av + avi);
        } else {
            x += inc_xr;
            y += inc_yr;
            av = (av + avr);
        }
    }
    _update_cell(x, y, +3);
}

// is the same fucntion using the bresenham algorithm but does not find the obstacle
inline static void no_obstacle_update(const Vec2 *const origin, const Vec2 *const dst) {

    // since each cell of the _grid is 12.5 cm or 0.125 m (1/8 m)  we have to scale our vectors that
    // represent meteres in the Q16.16 format to the _grid formta , and then we strip the decimals
    // from the numeber dividing by 2^16 ( num << 3 then num >> 16 = num >>13)

    // this might overflow but only if we reach 8192 meters , so we use a int16 to make the
    // operations quicker
    int16_t x1 = (int16_t)(origin->x >> 13) + HALF_SIZE;
    int16_t y1 = (int16_t)(origin->y >> 13) + HALF_SIZE;
    int16_t x2 = (int16_t)(dst->x >> 13) + HALF_SIZE;
    int16_t y2 = (int16_t)(dst->y >> 13) + HALF_SIZE;

    int16_t dx = x2 - x1;
    int16_t dy = y2 - y1;
    // now dx and dy

    int8_t inc_yi = 0;
    int8_t inc_xi = 0;
    int8_t inc_yr = 0;
    int8_t inc_xr = 0;
    if (dy >= 0) {
        inc_yi = 1;
    } else {
        dy = -dy;
        inc_yi = -1;
    }

    if (dx >= 0) {
        inc_xi = 1;
    } else {
        dx = -dx;
        inc_xi = -1;
    }

    if (dx >= dy) {
        inc_yr = 0;
        inc_xr = inc_xi;
    } else {
        inc_xr = 0;
        inc_yr = inc_yi;

        int16_t aux = dx;
        dx = dy;
        dy = aux;
    }

    int16_t avr = 2 * dy;
    int16_t av = (avr - dx);
    int16_t avi = (av - dx);
    int16_t x = x1;
    int16_t y = y1;
    while (x != x2 || y != y2) {
        _update_cell(x, y, -1);
        if (av >= 0) {
            x += inc_xi;
            y += inc_yi;
            av = (av + avi);
        } else {
            x += inc_xr;
            y += inc_yr;
            av = (av + avr);
        }
    }
}
#endif // OCCUPANCY_GRID_H
