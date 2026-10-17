# generation script for the look up
import math

LUT_RESOLUTION = 4096  # memory of the look up table
EXIT_FORMAT = 2**16
for i in range(LUT_RESOLUTION):
    norm = i / LUT_RESOLUTION
    angle = norm * 2 * math.pi
    print(
        round(math.sin(angle) * EXIT_FORMAT)
    )  # we multiply the sin with 2^16 so the exit is in Q16.16
    print(",")
