import math
import time

import pygame
import serial

START_BYTE = 0xAA
PACKET_SIZE = 10
# the map grid
MAP_START_BYTE = 0xAB

MAP_DIM = 56
MAP_SIZE = 6272


def main():
    ser = serial.Serial()
    ser.baudrate = 9600
    ser.port = "/dev/rfcomm0"
    ser.timeout = 0.15
    ser.open()
    time.sleep(1)
    pygame.init()
    screen = pygame.display.set_mode((1280, 720))
    clock = pygame.time.Clock()
    running = True
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    is_s_press = False
    is_a_press = False
    is_l_press = False
    is_k_press = False

    # robot position
    rx = 0.0
    ry = 0.0
    rang = 0.0

    map_buffer = bytearray()
    debug_grid = None
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_a:
                    ser.write(b"LF#")
                    is_a_press = True
                elif event.key == pygame.K_s:
                    ser.write(b"LB#")
                    is_s_press = True
                elif event.key == pygame.K_l:
                    ser.write(b"RF#")
                    is_l_press = True
                elif event.key == pygame.K_k:
                    ser.write(b"RB#")
                    is_k_press = True
                elif event.key == pygame.K_SPACE:
                    ser.write(b"RS#LS#")
                elif event.key == pygame.K_0:
                    ser.write(b"RF#")
                elif event.key == pygame.K_1:
                    ser.write(b"C1#")
                elif event.key == pygame.K_2:
                    ser.write(b"C2#")
                elif event.key == pygame.K_3:
                    ser.write(b"C3#")
                elif event.key == pygame.K_4:
                    ser.write(b"C4#")
                elif event.key == pygame.K_5:
                    ser.write(b"C5#")
                elif event.key == pygame.K_6:
                    ser.write(b"C6#")
                elif event.key == pygame.K_7:
                    ser.write(b"C7#")
                elif event.key == pygame.K_8:
                    ser.write(b"C8#")
                elif event.key == pygame.K_9:
                    ser.write(b"C9#")

            if event.type == pygame.KEYUP:
                if event.key == pygame.K_a:
                    if not is_s_press:
                        ser.write(b"LN#")
                    is_a_press = False
                elif event.key == pygame.K_s:
                    if not is_a_press:
                        ser.write(b"LN#")
                    is_s_press = False
                elif event.key == pygame.K_l:
                    if not is_k_press:
                        ser.write(b"RN#")
                    is_l_press = False
                elif event.key == pygame.K_k:
                    if not is_l_press:
                        ser.write(b"RN#")
                    is_k_press = False
                elif event.key == pygame.K_SPACE:
                    ser.write(b"RN#LN#")
                elif event.key == pygame.K_r:
                    # reset the buffer
                    map_buffer = bytearray()

        latest_pose = None
        while ser.in_waiting >= 2:
            pkt_type, payload = read_packet(ser)
            if pkt_type is None or payload is None:
                break

            if pkt_type == "pose" and len(payload) >= PACKET_SIZE:
                latest_pose = payload

            elif pkt_type == "map":
                print("map bit received")
                map_buffer.extend(payload)
                if len(map_buffer) >= MAP_SIZE:
                    CELLS = MAP_DIM * 2
                    new_grid = [[0] * CELLS for _ in range(CELLS)]
                    for r in range(MAP_DIM):
                        for c in range(MAP_DIM):
                            idx = (r * MAP_DIM + c) * 2
                            value = int.from_bytes(map_buffer[idx : idx + 2], "little")
                            for n in range(4):
                                x = c * 2 + (n & 1)
                                y = r * 2 + (n >> 1)
                                new_grid[y][x] = (value >> (4 * n)) & 0xF
                    debug_grid = new_grid
                    map_buffer.clear()
                    print("Mapa 56x56 reconstruido correctamente")

        if latest_pose is not None:
            robot_x = int.from_bytes(latest_pose[0:4], byteorder="little", signed=True)
            robot_y = int.from_bytes(latest_pose[4:8], byteorder="little", signed=True)
            robot_ang = int.from_bytes(
                latest_pose[8:10], byteorder="little", signed=False
            )
            rx = robot_x / 65536.0
            ry = robot_y / 65536.0
            rang = (robot_ang / 65536.0) * 2 * 3.14159  # radians
            print(
                f"X: {robot_x / 65536:.3f} | Y: {robot_y / 65536:.3f} | Ángulo: {(robot_ang / 65536.0) * 360.0:.1f}°"
            )
        screen.fill("grey")
        draw_debug_grid(screen, debug_grid, cell_size=6)
        tiles_per_meter = 8
        px = int((rx * tiles_per_meter + 56) * 6)
        py = int((112 - (ry * tiles_per_meter + 56)) * 6)
        pygame.draw.circle(screen, "red", (px, py), 6)
        pygame.draw.line(
            screen,
            "yellow",
            (px, py),
            (px + math.cos(-rang) * 12, py + math.sin(-rang) * 12),
            2,
        )
        pygame.display.flip()

        clock.tick(30)

    ser.close()
    pygame.quit()


def read_packet(ser):
    msg_type = "None"
    while True:
        b = ser.read(1)
        if len(b) == 0:
            return None, None
        if b[0] == START_BYTE:
            msg_type = "pose"
            break
        if b[0] == MAP_START_BYTE:
            msg_type = "map"
            break

    len_byte = ser.read(1)
    if len(len_byte) == 0:
        return None, None
    length = len_byte[0]

    payload = ser.read(length)
    if len(payload) < length:
        return None, None

    return msg_type, payload


def draw_debug_grid(
    screen,
    debug_grid,
    cell_size=12,
    offset_x=0,
    offset_y=0,
    flip_y=True,
    flip_x=False,
    transpose=False,
):
    if debug_grid is None:
        return

    n_rows = len(debug_grid)
    n_cols = len(debug_grid[0]) if n_rows else 0

    width = n_rows if transpose else n_cols
    height = n_cols if transpose else n_rows

    for r, row in enumerate(debug_grid):
        for c, val in enumerate(row):
            gx, gy = (r, c) if transpose else (c, r)

            sx = (width - 1 - gx) if flip_x else gx
            sy = (height - 1 - gy) if flip_y else gy

            level = (15 - min(max(val, 0), 15)) * 17
            color = (level, level, level)

            rect = (
                offset_x + sx * cell_size,
                offset_y + sy * cell_size,
                cell_size,
                cell_size,
            )
            pygame.draw.rect(screen, color, rect)


if __name__ == "__main__":
    main()
