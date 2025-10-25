#!/usr/bin/env python3
"""
BusServo_Move.py — Example: move a few servos using Board helpers.
"""
import time
import Board as Board

# Initialize direct bus
Board.init_bus(port="/dev/serial0", baud=115200, dir_pins=None)

# Move IDs 1,2,3
for sid, p in [(1, 520), (2, 480), (3, 500)]:
    Board.setBusServoPulse(sid, p, 600)
time.sleep(0.7)

# Read back
for sid in (1,2,3):
    pos = Board.getBusServoPulse(sid)
    print(f"Servo {sid} pos: {pos}")
