#!/usr/bin/python3
# -*- coding: UTF-8 -*-

from .SerialServo import Serial_Servo

class Leg():
    default_time = 500

    def __init__(self, pi, leg):
        self.pi = pi
        self.leg = leg
        self.shoulder = Serial_Servo(self.pi, (self.leg * 3) + 1)
        self.knee = Serial_Servo(self.pi, (self.leg * 3) + 2)
        self.ankle = Serial_Servo(self.pi, (self.leg * 3) + 3)

    def set_pos(self, posn):
        if len(posn) == 2:
            self.shoulder.pos = (posn[0][0], posn[1])
            self.knee.pos = (posn[0][1], posn[1])
            self.ankle.pos = (posn[0][2], posn[1])
        else:
            self.shoulder.pos = (posn[0], Leg.default_time)
            self.knee.pos = (posn[1], Leg.default_time)
            self.ankle.pos = (posn[2], Leg.default_time)

    def get_pos(self):
        return (self.shoulder.pos, self.knee.pos, self.ankle.pos)

    pos = property(get_pos, set_pos)

    def set_load(self, mode=1):
        self.shoulder.load = mode
        self.knee.load = mode
        self.ankle.load = mode

    def get_load(self):
        return (self.shoulder.load, self.knee.load, self.ankle.load)

    load = property(get_load, set_load)
