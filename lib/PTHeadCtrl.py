#!/usr/bin/python3
# -*- coding: UTF-8 -*-

from .PWMServo import PWM_Servo

pan = 0
tilt = 1
PTHead = ()

def init_pthead(pi):
    global PTHead
    pan_servo = PWM_Servo(pi, 5, min_width=800, max_width=2200, offset=0, control_speed=True)
    tilt_servo = PWM_Servo(pi, 6, min_width=1200, max_width=2000, offset=0, control_speed=True)
    PTHead = (pan_servo, tilt_servo)

def setPWMServo(id, pos, tim=0):
    PTHead[id].setPosition(pos, tim)

def getPWMServo(id):
    return PTHead[id].Position

def setPTHpos(pan_pos, tilt_pos, tim=0):
    setPWMServo(pan, pan_pos, tim)
    setPWMServo(tilt, tilt_pos, tim)

def setPTHcentre():
    setPWMServo(pan, PTHead[pan].default_pos)
    setPWMServo(tilt, PTHead[tilt].default_pos)

def getPTHpos():
    return (PTHead[pan].Position, PTHead[tilt].Position)
