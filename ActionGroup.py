#!/usr/bin/python3
# encoding: utf-8
# Copyright 2020-2022 ians.moyes@gmail.com

import time
from BusServo import BusServo

servos = [BusServo(i) for i in range(1, 19)]

def stand_up():
    """Make the hexapod stand up."""
    for servo in servos:
        servo.move_time_write(500, 1000)
    time.sleep(1)

def sit_down():
    """Make the hexapod sit down."""
    for servo in servos:
        servo.move_time_write(100, 1000)
    time.sleep(1)

def wave_leg():
    """Wave one of the hexapod's legs."""
    servos[0].move_time_write(800, 500)
    time.sleep(0.5)
    servos[0].move_time_write(200, 500)
    time.sleep(0.5)
    servos[0].move_time_write(500, 500)
    time.sleep(0.5)
