import time

import pygame
import serial

START_BYTE = 0xAA
PACKET_SIZE = 12


def main():
    ser = serial.Serial()
    ser.baudrate = 9600
    ser.port = "/dev/rfcomm0"
    ser.timeout = 0.05
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

        payload = read_latest_packet(ser)
        if payload is not None and len(payload) == PACKET_SIZE:
            robot_x = int.from_bytes(payload[0:4], byteorder="little", signed=True)
            robot_y = int.from_bytes(payload[4:8], byteorder="little", signed=True)
            robot_ang = int.from_bytes(payload[8:12], byteorder="little", signed=True)

            print(
                f"X: {robot_x / 1000000:.3f} | Y: {robot_y / 1000000:.3f} | Ángulo: {(robot_ang / 1000000) * 180 / 3.141592:.3f}"
            )
        # print(f"in_waiting: {ser.in_waiting}")
        screen.fill("grey")
        pygame.display.flip()
        clock.tick(30)

    ser.close()
    pygame.quit()


def read_packet(ser):
    while True:
        b = ser.read(1)
        if len(b) == 0:
            return None
        if b[0] == START_BYTE:
            break

    len_byte = ser.read(1)
    if len(len_byte) == 0:
        return None
    length = len_byte[0]

    payload = ser.read(length)
    if len(payload) < length:
        return None

    return payload


def read_latest_packet(ser):
    latest = None
    while ser.in_waiting >= 2:
        packet = read_packet(ser)
        if packet is not None:
            latest = packet
        else:
            break
    return latest


if __name__ == "__main__":
    main()
