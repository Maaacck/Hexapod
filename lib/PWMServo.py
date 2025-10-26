#!/usr/bin/python3
# -*- coding: UTF-8 -*-

import time
import threading

class PWM_Servo(object):
    servo_type = "generic"
    rotate_limits = (500, 2500)
    time_limits = (20, 5000)
    offset_limits = (-300, 300)
    PWMfreq = 50
    stepTime = 20
    default_pos = 1500

    def __init__(self, pi, pin, freq=PWMfreq, min_width=rotate_limits[0], max_width=rotate_limits[1], offset=0, control_speed=False):
        self.pi = pi
        self.SPin = pin
        self.Position = PWM_Servo.default_pos
        self.positionSet = self.Position
        self.Freq = freq
        self.Min = min_width
        self.Max = max_width
        self.Offset = offset
        self.Mintime = PWM_Servo.time_limits[0]
        self.Maxtime = PWM_Servo.time_limits[1]
        self.stepTime = PWM_Servo.stepTime
        self.positionInc = 0.0
        self.Time = 0
        self.incTimes = 1
        self.prev = time.time()
        self.speedControl = control_speed

        self.pi.set_PWM_range(self.SPin, int(1000000 / self.Freq))
        self.pi.set_PWM_frequency(self.SPin, self.Freq)
        self.pi.set_PWM_dutycycle(self.SPin, self.Position + self.Offset)

        if self.speedControl is True:
            t1 = threading.Thread(target=PWM_Servo.updatePosition, args=(self,))
            t1.setDaemon(True)
            t1.start()

    def setPosition(self, pos, tim=0):
        if pos < self.Min or pos > self.Max:
            print("Invalid position value " + str(pos))
            return "pos"
        self.positionSet = pos
        if not self.speedControl:
            tim = 0
        if tim == 0:
            self.Position = pos
            self.pi.set_PWM_dutycycle(self.SPin, self.Position + self.Offset)
        else:
            if tim < self.Mintime:
                self.Time = self.Mintime
            elif tim > self.Maxtime:
                self.Time = self.Maxtime
            else:
                self.Time = tim
        return True

    def updatePosition(self):
        while True:
            if self.Position != self.positionSet:
                self.incTimes = int(self.Time / self.stepTime)
                if self.incTimes == 0:
                    self.incTimes = 1
                self.positionInc = int((self.positionSet - self.Position) / self.incTimes)
                for i in range(self.incTimes):
                    self.prev = time.time()
                    self.Position += self.positionInc
                    self.pi.set_PWM_dutycycle(self.SPin, self.Position + self.Offset)
                    time.sleep(max(0, 0.02 - (time.time() - self.prev)))
                self.Time = self.Mintime
