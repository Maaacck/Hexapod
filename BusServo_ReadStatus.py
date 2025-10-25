#!/usr/bin/env python3
"""
BusServo_ReadStatus.py — Example: read positions/voltage directly from the bus.
"""
import Board as Board

bus = Board.init_bus(port="/dev/serial0", baud=115200, dir_pins=None)

for sid in (1,2,3):
    pos = bus.pos_read(sid)
    vin = bus.vin_read(sid)
    print(f"Servo {sid}: pos={pos}, vin(mV)={vin}")
