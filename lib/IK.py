#!/usr/bin/python3
# -*- coding: UTF-8 -*-

import math

THIGH_LENGTH = 44.60
CALF_LENGTH = 75.00
FOOT_LENGTH = 126.50
CALF_SQUARED = CALF_LENGTH**2
FOOT_SQUARED = FOOT_LENGTH**2
TW0_CALF_FOOT = 2 * CALF_LENGTH * FOOT_LENGTH

def switch_sides(coords):
    return (1000 - coords[0], 1000 - coords[1], 1000 - coords[2])

def remap(angle):
    return int(((math.degrees(angle) / 120) * 500) + 500)

def inverse_kin(leg, coords):
    x = float(coords[0])
    y = float(coords[1])
    z = float(coords[2])

    w = math.hypot(x, y)

    if w > THIGH_LENGTH + CALF_LENGTH + FOOT_LENGTH:
        return "Position too far"

    w -= THIGH_LENGTH

    if y == 0: y = 1
    if z == 0: z = 1
    if w == 0: w = 1

    shoulder_angle = math.atan2(x, y)
    servo_positions = [remap(shoulder_angle)]

    toedist = math.hypot(w, z)
    toedist_squared = toedist**2

    knee_acos_arg = (toedist_squared + CALF_SQUARED - FOOT_SQUARED) / (2 * toedist * CALF_LENGTH)
    knee_ang = math.acos(max(-1.0, min(1.0, knee_acos_arg)))
    knee_flex = knee_ang - math.atan2(w, z)
    servo_positions.append(remap(knee_flex))

    ankle_acos_arg = (FOOT_SQUARED + CALF_SQUARED - toedist_squared) / TW0_CALF_FOOT
    ankle_ang = math.acos(max(-1.0, min(1.0, ankle_acos_arg)))
    ankle_flex = math.pi - ankle_ang
    servo_positions.append(remap(ankle_flex))

    if leg > 2:
        return switch_sides(tuple(servo_positions))

    return tuple(servo_positions)
