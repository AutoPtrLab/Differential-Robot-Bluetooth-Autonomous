import time
import pygame
import serial


#this is the function that will recive the grid map from  the robot

GRID_START_BYTE = 0xABABABAB
GRID_DIM = 112 # 56 * 2 half_bytes
MAP_BYTES = 56*56 * 2

def read_grid():
    
    
