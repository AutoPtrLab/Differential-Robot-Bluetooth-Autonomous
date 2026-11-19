#include "search_helper.h"
#include "occupancy_grid.h"
#include <stdint.h>
#include <string.h>

uint16_t BFS_frontier_search(const uint16_t grid[GRID_SIZE][GRID_SIZE], uint8_t x_init,
                             uint8_t y_init) {

    // we use a bit for visited or not visited
    // automatic zero init
    static uint16_t queue[160]; // Estático para no llenar el stack

    // WARNING : be careful if the gridsize changes and 8 is not a divisor
    static uint8_t visited[(GRID_SIZE * GRID_SIZE) / 8];
    memset(visited, 0, sizeof(visited));

    uint8_t head = 0; // first value of the queue
    uint8_t tail = 0; // last value of the queue

    uint16_t init_bit = (uint16_t)y_init * GRID_SIZE + x_init;
    visited[init_bit >> 3] |= (uint8_t)(1U << (init_bit & 7));

    // queue , the first byte is x , the second one y
    queue[tail++] = (uint16_t)(((uint16_t)x_init << 8) | y_init);

    // cardinal displacemnts
    const int8_t dx[4] = {1, 0, 0, -1};
    const int8_t dy[4] = {0, 1, -1, 0};

    // the goal is to find the first value of the grid that is unknown , since we are
    // taking the 4 tiles as one we want the four of them to be unknown (init at the
    // default value)

    // while there is at least one element in the queue
    while (tail != head) {
        uint16_t curr = queue[head++];
        if (head == 160)
            head = 0;
        uint8_t x = (uint8_t)(curr >> 8);
        uint8_t y = (uint8_t)(curr & 0xFF);

        // if we reach the fisrt frontier we return the value
        if (grid[y][x] == 0x7777) {
            return curr;
        }

        // Revisar los 4 vecinos
        for (uint8_t i = 0; i < 4; i++) {
            int8_t nx = (int8_t)x + dx[i];
            int8_t ny = (int8_t)y + dy[i];

            // border check
            if (nx < 0 || nx >= GRID_SIZE || ny < 0 || ny >= GRID_SIZE) {
                continue;
            }

            uint16_t bit_idx = (uint16_t)ny * GRID_SIZE + (uint8_t)nx;
            uint16_t byte_idx = bit_idx >> 3;
            uint8_t mask = (uint8_t)(1U << (bit_idx & 7));

            // check if it was already visited
            if (visited[byte_idx] & mask) {
                continue;
            }

            // mark as visited an enqueue the possible neighbors
            visited[byte_idx] |= mask;
            queue[tail++] = (uint16_t)(((uint16_t)nx << 8) | (uint8_t)ny);
            if (tail == 160)
                tail = 0;
        }
        //  since there are no frontiers(maybe send
        //  a signal to the global planner)
    }
    return (uint16_t)0xFFFF;
}

uint16_t JPS_find_path(const uint16_t grid[GRID_SIZE][GRID_SIZE], uint8_t x_goal, uint8_t y_goal,
                       uint8_t x_init, uint8_t y_init) {
    // normally this optimization of A* uses a min heap but since the map is not to
    //  big, a linear search will sufice

    static uint16_t active_nodes[256];
    memset(active_nodes, 0xFFFF, sizeof(active_nodes));

    static uint16_t values[GRID_SIZE * GRID_SIZE];
    memset(values, 0xFF, sizeof(values));

    values[x_init + y_init * GRID_SIZE] = 0;
    active_nodes[0] = (uint16_t)((uint16_t)x_init << 8) | (uint16_t)y_init;

    while (1) {
        uint16_t min_value = 0xFF;
        uint16_t idx = 0xFFFF;
        for (int i = 0; i < 256; i++) {

            if (values[(uint8_t)active_nodes[i] * GRID_SIZE + (uint8_t)(active_nodes[i] >> 8)] <
                min_value) {
                min_value = active_nodes[i];
                idx = i;
            }
        }

        if (min_value == 0xFFFF) {
            // there is no way to that point
            return 0xFFFF;
        }

        uint16_t current = active_nodes[idx];

        if ((uint8_t)current == x_goal && (uint8_t)(current >> 8) == y_goal) {
            return current;
        }
        active_nodes[idx] = 0xFFFF;

        for ()
    }

    return 0;
}
