#include "search_helper.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define max(a, b) (((a) > (b)) ? (a) : (b))
#define MAX_JPS_NODES 128

#define NODE_FREE 0
#define NODE_OPEN 1
#define NODE_CLOSED 2

// since there is not really enough ram to mantain the data structures of both JPS and BFS we use a
// union

typedef struct {
    uint8_t x;
    uint8_t y;
    uint16_t g;     // acummulated cost,since we are using uints 10 cost per cardinal move 14
                    // per diag
    uint16_t f;     // g + ocitle
    uint8_t parent; // index of the parent (if is the initial one 0xFF)
    uint8_t state;  // NODE_FREE, NODE_OPEN, NODE_CLOSED
} JPSNode;
static union {
    struct {
        // number of active nodes
        uint16_t queue[256];
        // WARNING : be careful if the gridsize changes and 8 is not a divisor
        uint8_t visited[(GRID_SIZE * GRID_SIZE) / 8];
    } BFS;

    struct {

        // nodes list , this is the mayor improvent of using JPS, since we dont have to mantain a
        // complete grid of costs and only an array(normally is a min-heap but with so little nodes
        // is not worth) we can fit the algorithm in the 1700b of ram left
        JPSNode nodes[MAX_JPS_NODES];

    } JPS;
} searcher_memory;
static inline bool is_obstacle(const uint16_t grid[GRID_SIZE][GRID_SIZE], int16_t x, int16_t y) {
    // since we consider a obtacle when the value of the tile is greater that 7 and a tile is 4
    // bits, if the MSB is 1 its always 8 or more 0x8888 = 1000 1000 1000 1000 ,so if this isnt zero
    // is an obstacle
    if (x < 0 || x >= GRID_SIZE || y < 0 || y >= GRID_SIZE) {
        return true;
    }
    // return ((grid[y][x] & 0x8888) != 0) && ((grid[y][x] & 0x4444) != 0);
    return ((grid[y][x] & 0x8888) != 0);
}
static inline bool is_unknown(const uint16_t grid[GRID_SIZE][GRID_SIZE], uint8_t x, uint8_t y) {
    // if the four tiles are unknonw
    return (grid[y][x] == 0x7777) || (grid[y][x] == 0x6666) || (grid[y][x] == 0x5555) ||
           (grid[y][x] == 0x8888);
}
static inline bool is_walkable(const uint16_t grid[GRID_SIZE][GRID_SIZE], uint8_t x, uint8_t y) {

    // we consider walkable when the tile is 3 or less,same trick as fn is_obstacle
    //
    //  0xCCCC = 1100 1100 1100 1100 , if the mask aint zero is not walkable
    return (grid[y][x] & 0xCCCC) == 0;
}
// direction enum
typedef enum {
    DirRight = 0,
    DirLeft = 1,
    DirUp = 2,
    DirDown = 3,
    DirRightUp = 4,
    DirLeftUp = 5,
    DirRightDown = 6,
    DirLeftDown = 7
} Direction;

// direction arrays to make de Direction enum movements
static const int8_t DIR_DX[8] = {1, -1, 0, 0, 1, -1, 1, -1};
static const int8_t DIR_DY[8] = {0, 0, 1, -1, 1, 1, -1, -1};

// return the kind of movemente it was
static inline uint8_t dir_bit(int8_t dx, int8_t dy) {
    for (uint8_t i = 0; i < 8; i++) {
        if (DIR_DX[i] == dx && DIR_DY[i] == dy) {
            return (uint8_t)(1U << i);
        }
    }
    return 0;
}
static inline uint8_t prune(const uint16_t grid[GRID_SIZE][GRID_SIZE], uint8_t curr_idx) {
    // parent id
    uint8_t p_idx = searcher_memory.JPS.nodes[curr_idx].parent;

    // check if is the init node
    if (p_idx == 0xFF) {
        return 0xFF;
    }

    int16_t cx = searcher_memory.JPS.nodes[curr_idx].x;
    int16_t cy = searcher_memory.JPS.nodes[curr_idx].y;
    // check from where did the parent reached us
    int8_t dx = 0;
    int8_t dy = 0;
    if (cx > searcher_memory.JPS.nodes[p_idx].x)
        dx = 1;
    else if (cx < searcher_memory.JPS.nodes[p_idx].x)
        dx = -1;

    if (cy > searcher_memory.JPS.nodes[p_idx].y)
        dy = 1;
    else if (cy < searcher_memory.JPS.nodes[p_idx].y)
        dy = -1;

    uint8_t mask = 0;

    if (dx == 0 || dy == 0) {
        // cardinal case
        if (!is_obstacle(grid, cx + dx, cy + dy)) {
            mask |= dir_bit(dx, dy);
        }
        // forced neighbours
        if (dx != 0) {
            if (is_obstacle(grid, cx, cy + 1) && !is_obstacle(grid, cx + dx, cy + 1)) {
                mask |= dir_bit(dx, 1);
            }
            if (is_obstacle(grid, cx, cy - 1) && !is_obstacle(grid, cx + dx, cy - 1)) {
                mask |= dir_bit(dx, -1);
            }
        } else {
            if (is_obstacle(grid, cx + 1, cy) && !is_obstacle(grid, cx + 1, cy + dy)) {
                mask |= dir_bit(1, dy);
            }
            if (is_obstacle(grid, cx - 1, cy) && !is_obstacle(grid, cx - 1, cy + dy)) {
                mask |= dir_bit(-1, dy);
            }
        }
    } else {
        // diagonal case
        bool walk_x = !is_obstacle(grid, cx + dx, cy);
        bool walk_y = !is_obstacle(grid, cx, cy + dy);

        if (walk_x)
            mask |= dir_bit(dx, 0);
        if (walk_y)
            mask |= dir_bit(0, dy);

        if ((walk_x || walk_y) && !is_obstacle(grid, cx + dx, cy + dy)) {
            mask |= dir_bit(dx, dy);
        }

        // forced neighbours
        if (is_obstacle(grid, cx - dx, cy) && !is_obstacle(grid, cx - dx, cy + dy)) {
            mask |= dir_bit(-dx, dy);
        }
        if (is_obstacle(grid, cx, cy - dy) && !is_obstacle(grid, cx + dx, cy - dy)) {
            mask |= dir_bit(dx, -dy);
        }
    }

    return mask;
}

// searchs for a node based on the position
static inline uint8_t find_node_index(uint8_t x, uint8_t y, uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        if (searcher_memory.JPS.nodes[i].x == x && searcher_memory.JPS.nodes[i].y == y) {
            return i;
        }
    }
    return 0xFF;
}
static inline uint16_t octile_heuristic(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
    uint16_t dx = (x1 > x2) ? (x1 - x2) : (x2 - x1);
    uint16_t dy = (y1 > y2) ? (y1 - y2) : (y2 - y1);
    if (dx > dy) {
        return (uint16_t)(14U * dy + 10U * (dx - dy));
    } else {
        return (uint16_t)(14U * dx + 10U * (dy - dx));
    }
}

static inline uint16_t jump_cardinal(uint8_t x_goal, uint8_t y_goal, uint8_t x_node, uint8_t y_node,
                                     int8_t dx, int8_t dy,
                                     const uint16_t grid[GRID_SIZE][GRID_SIZE]) {

    int16_t n_x = x_node;
    int16_t n_y = y_node;

    while (true) {
        n_x += dx;
        n_y += dy;

        if (n_x < 0 || n_x >= GRID_SIZE || n_y < 0 || n_y >= GRID_SIZE) {
            return 0xFFFF;
        }
        if (is_obstacle(grid, n_x, n_y)) {
            return 0xFFFF;
        }

        if (n_x == x_goal && n_y == y_goal) {
            return (uint16_t)(uint16_t)(n_x << 8) | (uint8_t)n_y;
        }
        // detection of the forced neighbor
        if (is_obstacle(grid, n_x + dy, n_y + dx) &&
            !is_obstacle(grid, n_x + dy + dx, n_y + dx + dy))
            return (uint16_t)((uint16_t)n_x << 8) | (uint8_t)n_y;
        if (is_obstacle(grid, n_x - dy, n_y - dx) &&
            !is_obstacle(grid, n_x - dy + dx, n_y - dx + dy))
            return (uint16_t)((uint16_t)n_x << 8) | (uint8_t)n_y;
    }
}
static inline uint16_t jump_diag(uint8_t x_goal, uint8_t y_goal, uint8_t x_node, uint8_t y_node,
                                 int8_t dx, int8_t dy, const uint16_t grid[GRID_SIZE][GRID_SIZE]) {

    int16_t n_x = x_node;
    int16_t n_y = y_node;

    while (true) {
        if (is_obstacle(grid, n_x + dx, n_y) && is_obstacle(grid, n_x, n_y + dy)) {
            return 0xFFFF;
        }
        n_x += dx;
        n_y += dy;
        if (n_x < 0 || n_x >= GRID_SIZE || n_y < 0 || n_y >= GRID_SIZE) {
            return 0xFFFF;
        }
        if (is_obstacle(grid, n_x, n_y)) {
            return 0xFFFF;
        }
        if (n_x == x_goal && n_y == y_goal) {
            return ((uint16_t)n_x << 8) | (uint8_t)n_y;
        }

        // forced neighbours
        if ((is_obstacle(grid, n_x - dx, n_y) && !is_obstacle(grid, n_x - dx, n_y + dy)) ||
            (is_obstacle(grid, n_x, n_y - dy) && !is_obstacle(grid, n_x + dx, n_y - dy))) {
            return ((uint16_t)n_x << 8) | (uint8_t)n_y;
        }

        // call to the cardinal function
        if (jump_cardinal(x_goal, y_goal, (uint8_t)n_x, (uint8_t)n_y, dx, 0, grid) != 0xFFFF ||
            jump_cardinal(x_goal, y_goal, (uint8_t)n_x, (uint8_t)n_y, 0, dy, grid) != 0xFFFF) {
            return ((uint16_t)n_x << 8) | (uint8_t)n_y;
        }
    }
}

inline static uint16_t jump(uint8_t node_x, uint8_t node_y, uint8_t x_goal, uint8_t y_goal,
                            Direction dir, const uint16_t grid[GRID_SIZE][GRID_SIZE]) {
    int16_t n_x = node_x;
    int16_t n_y = node_y;

    int8_t dx = 0;
    int8_t dy = 0;

    switch (dir) {
    case DirRight:
        dx = 1;
        dy = 0;
        break;
    case DirLeft:
        dx = -1;
        dy = 0;
        break;
    case DirUp:
        dx = 0;
        dy = 1;
        break;
    case DirDown:
        dx = 0;
        dy = -1;
        break;
    case DirRightUp:
        dx = 1;
        dy = 1;
        break;
    case DirLeftUp:
        dx = -1;
        dy = 1;
        break;
    case DirRightDown:
        dx = 1;
        dy = -1;
        break;
    case DirLeftDown:
        dx = -1;
        dy = -1;
        break;
    default:
        return 0xFFFF;
    }
    // if is cardinal we delegated to the function
    if (dx == 0 || dy == 0) {
        return jump_cardinal(x_goal, y_goal, (uint8_t)n_x, (uint8_t)n_y, dx, dy, grid);
    } else {
        return jump_diag(x_goal, y_goal, (uint8_t)n_x, (uint8_t)n_y, dx, dy, grid);
    }
}

uint8_t JPS_find_path(const uint16_t grid[GRID_SIZE][GRID_SIZE], uint8_t x_start, uint8_t y_start,
                      uint8_t x_goal, uint8_t y_goal, uint16_t waypoints[MAX_WAYPOINTS]) {
    // check if the goal or the start is an obstacle
    if (is_obstacle(grid, x_start, y_start) || is_obstacle(grid, x_goal, y_goal)) {
        return 0;
    }
    // we dont need to restart the union mem

    //  init of the fisrt node
    searcher_memory.JPS.nodes[0].x = x_start;
    searcher_memory.JPS.nodes[0].y = y_start;
    searcher_memory.JPS.nodes[0].g = 0;
    searcher_memory.JPS.nodes[0].f = octile_heuristic(x_start, y_start, x_goal, y_goal);
    // since this node does not have a parent we use the secial value
    searcher_memory.JPS.nodes[0].parent = 0xFF;
    searcher_memory.JPS.nodes[0].state = NODE_OPEN;
    // number of nodes
    uint8_t node_count = 1;

    while (true) {
        uint8_t best_idx = 0xFF;
        uint16_t best_f = 0xFFFF;

        for (uint8_t i = 0; i < node_count; i++) {
            if (searcher_memory.JPS.nodes[i].state == NODE_OPEN &&
                searcher_memory.JPS.nodes[i].f < best_f) {
                best_f = searcher_memory.JPS.nodes[i].f;
                best_idx = i;
            }
        }

        // there is no node
        if (best_idx == 0xFF) {
            return 0;
        }

        // we reached the goal and reconstruct the waypoints
        if (searcher_memory.JPS.nodes[best_idx].x == x_goal &&
            searcher_memory.JPS.nodes[best_idx].y == y_goal) {
            // waypoint count
            uint8_t wp_count = 0;
            uint8_t curr = best_idx;

            while (curr != 0xFF && wp_count < MAX_WAYPOINTS) {
                // we pack the information
                waypoints[wp_count++] = ((uint16_t)searcher_memory.JPS.nodes[curr].x << 8) |
                                        searcher_memory.JPS.nodes[curr].y;
                curr = searcher_memory.JPS.nodes[curr].parent;
            }

            // since we want to go from the init pos to the goal we have to reverse the parent -
            // node

            for (uint8_t i = 0; i < wp_count >> 1; i++) {
                uint16_t tmp = waypoints[i];
                waypoints[i] = waypoints[wp_count - 1 - i];
                waypoints[wp_count - 1 - i] = tmp;
            }
            return wp_count;
        }

        // we "closed" the node and mark it as free
        searcher_memory.JPS.nodes[best_idx].state = NODE_CLOSED;

        uint8_t cx = searcher_memory.JPS.nodes[best_idx].x;
        uint8_t cy = searcher_memory.JPS.nodes[best_idx].y;

        uint8_t valid_dirs = prune(grid, best_idx);

        // we check only the directions that prune has given us in the bitmask
        for (uint8_t d = 0; d < 8; d++) {
            if (!(valid_dirs & (1U << d))) {
                continue;
            }

            uint16_t jp = jump(cx, cy, x_goal, y_goal, (Direction)d, grid);
            // if the jump did not lead up to anything...
            if (jp == 0xFFFF) {
                continue;
            }

            uint8_t jx = (uint8_t)(jp >> 8);
            uint8_t jy = (uint8_t)jp;

            uint16_t step_dx = (jx > cx) ? (uint16_t)(jx - cx) : (uint16_t)(cx - jx);
            uint16_t step_dy = (jy > cy) ? (uint16_t)(jy - cy) : (uint16_t)(cy - jy);
            uint16_t jump_cost = 0;
            // diagonal cost
            if (step_dx != 0 && step_dy != 0) {
                jump_cost = step_dx * 14U;

            } else {
                // cardinal cost
                jump_cost = 10 * (step_dx + step_dy);
            }

            uint16_t new_g = searcher_memory.JPS.nodes[best_idx].g + jump_cost;

            // we check the same position we have reached to check if there is an better path
            uint8_t existing = find_node_index(jx, jy, node_count);

            // the jump point is in the list
            if (existing != 0xFF) {
                if (searcher_memory.JPS.nodes[existing].state == NODE_CLOSED) {
                    continue;
                }
                // we  update if the new Jump point is better
                if (new_g < searcher_memory.JPS.nodes[existing].g) {
                    searcher_memory.JPS.nodes[existing].g = new_g;
                    searcher_memory.JPS.nodes[existing].f =
                        new_g + octile_heuristic(jx, jy, x_goal, y_goal);
                    searcher_memory.JPS.nodes[existing].parent = best_idx;
                }
                // if there was no jump point in the same position we add a new one
            } else if (node_count < MAX_JPS_NODES) {
                searcher_memory.JPS.nodes[node_count].x = jx;
                searcher_memory.JPS.nodes[node_count].y = jy;
                searcher_memory.JPS.nodes[node_count].g = new_g;
                searcher_memory.JPS.nodes[node_count].f =
                    new_g + octile_heuristic(jx, jy, x_goal, y_goal);
                searcher_memory.JPS.nodes[node_count].parent = best_idx;
                searcher_memory.JPS.nodes[node_count].state = NODE_OPEN;
                node_count++;
            }
        }
    }
}
uint16_t BFS_frontier_search(const uint16_t grid[GRID_SIZE][GRID_SIZE], uint8_t x_init,
                             uint8_t y_init) {
    // if the init values are out of the grid or are in an obstacle we return 0xFFFF
    //  (this should ideally never happen if the odometry is working properly and the robot stays in
    //  range)
    if (is_obstacle(grid, x_init, y_init)) {
        return 0xFFFF;
    }

    // since the queue does not need to be restarted to zero becuase the tail and head manage its
    // memory
    //  we only restart visited and mark all of them as not visited
    memset(searcher_memory.BFS.visited, 0, sizeof(searcher_memory.BFS.visited));
    uint8_t head = 0; // first value of the queue

    uint8_t tail = 0; // last value of the queue

    uint16_t init_bit = (uint16_t)y_init * GRID_SIZE + x_init;
    // the init bit is visited
    searcher_memory.BFS.visited[init_bit >> 3] |= (uint8_t)(1U << (init_bit & 7));

    // queue , the first byte is x , the second one y
    searcher_memory.BFS.queue[tail++] = (uint16_t)(((uint16_t)x_init << 8) | y_init);

    // cardinal displacemnts
    const int8_t dx[4] = {1, 0, 0, -1};
    const int8_t dy[4] = {0, 1, -1, 0};

    // the goal is to find the first value of the grid that is unknown , since we are
    // taking the 4 tiles as one we want the four of them to be unknown (init at the
    // default value)

    // while there is at least one element in the queue
    while (tail != head) {
        uint16_t curr = searcher_memory.BFS.queue[head++];
        // normal wrap around of a 8bit
        uint8_t x = (uint8_t)(curr >> 8);
        uint8_t y = (uint8_t)(curr & 0xFF);

        // if we reach the first frontier we return the value
        if (is_unknown(grid, x, y)) {
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
            // obstacle check
            if (is_obstacle(grid, nx, ny))
                continue;

            // getting the position , which is just a bit flag
            uint16_t bit_idx = (uint16_t)ny * GRID_SIZE + (uint8_t)nx;
            // we get the byte where this bit is
            uint16_t byte_idx = bit_idx >> 3; //( / 8)
            // we get the position of the bit
            uint8_t mask = (uint8_t)(1U << (bit_idx & 7)); // % 8

            // check if it was already visited
            if (searcher_memory.BFS.visited[byte_idx] & mask) {
                continue;
            }

            // mark as visited an enqueue the possible neighbors
            searcher_memory.BFS.visited[byte_idx] |= mask;
            searcher_memory.BFS.queue[tail++] = (uint16_t)(((uint16_t)nx << 8) | (uint8_t)ny);
            // normal wrap around
        }
        //  since there are no frontiers(maybe send
        //  a signal to the global planner)
    }
    return (uint16_t)0xFFFF;
}
