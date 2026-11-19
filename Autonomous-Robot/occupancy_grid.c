#include "occupancy_grid.h"
#include "bluetooth.h"
#include "config.h"
#include <stdint.h>

uint16_t _grid[GRID_SIZE][GRID_SIZE];
const uint16_t NIBBLE_MASK[4] = {0x000F, 0x00F0, 0x0F00, 0xF000};
const uint8_t NIBBLE_SHIFT[4] = {0, 4, 8, 12};

void send_mapgrid(void) {
    static uint8_t map_buffer[DATA_COUNT];
    // since the map despite being an array is continous in memory we can access
    // like a normal array , we only need to cast the pointer to the prper type
    const uint16_t *flat_grid = (const uint16_t *)_grid;

    const uint16_t total_elements = GRID_SIZE * GRID_SIZE; // 56 * 56
    const uint16_t elems_per_chunk =
        DATA_COUNT / 2; // since each position in the matrix is 16 bits, and we send 8bits

    for (uint16_t i = 0; i < total_elements; i += elems_per_chunk) {
        for (uint16_t j = 0; j < elems_per_chunk; j++) {
            uint16_t value = flat_grid[i + j];

            // little endian
            map_buffer[j * 2] = (uint8_t)(value);
            map_buffer[j * 2 + 1] = (uint8_t)(value >> 8);

            // waiting for the transmission flag
        }
        while (!bt_tx_is_ready())
            ;
        bt_send_data(map_buffer, DATA_COUNT, START_MAP_BYTE);
    }
}
