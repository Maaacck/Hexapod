#!/usr/bin/python3
# -*- coding: UTF-8 -*-
# This file is a combination of multiple files from the SpiderPi-master repository,
# with optimizations for better performance and readability.

import math
import time
import serial
import pigpio
import ctypes
import threading

# ============================================================================
# RPiExpCom.py
# ============================================================================

SERVO_FRAME_HEADER = b'\x55\x55'
TRINKET_FRAME_HEADER = b'\x25\x25'

UART = serial.Serial("/dev/ttyAMA0", 115200)

def portinit(pi):
    pi.set_mode(17, pigpio.OUTPUT)
    pi.write(17, 0)
    pi.set_mode(27, pigpio.OUTPUT)
    pi.write(27, 1)

def portWrite(pi):
    pi.write(17, 0)
    pi.write(27, 1)

def portRead(pi):
    pi.write(17, 1)
    pi.write(27, 0)

def checksum(buf):
    sum = 0x00
    for b in buf:
        sum += b
    sum = ~sum
    return sum & 0xff

def serial_servo_write_cmd(pi, id, w_cmd, dat1=None, dat2=None):
    portWrite(pi)
    buf = bytearray()
    buf.append(id)
    if dat2 is not None:
        buf.append(7)
    elif dat1 is not None:
        buf.append(4)
    else:
        buf.append(3)
    buf.append(w_cmd)
    if dat2 is not None:
        buf.extend([(0xff & dat1), (0xff & (dat1 >> 8))])
        buf.extend([(0xff & dat2), (0xff & (dat2 >> 8))])
    elif dat1 is not None:
        buf.append(dat1 & 0xff)
    buf.append(checksum(buf))
    frame = bytearray(SERVO_FRAME_HEADER) + buf
    UART.write(frame)
    return True

def serial_servo_read_cmd(pi, id, r_cmd):
    prev = time.time()
    UART.flushInput()
    while time.time() < prev + 1:
        write_ok = serial_servo_write_cmd(pi, id, r_cmd)
        if write_ok == True:
            time.sleep(0.00034)
            portRead(pi)
            for i in range(20):
                results = collect_serial_servo_data(r_cmd)
                if results is not None:
                    return results
    return "Comms"

def collect_serial_servo_data(r_cmd):
    time.sleep(0.00034)
    count = UART.inWaiting()
    if count != 0:
        recv_data = UART.read(count)
        UART.flushInput()
        try:
            data_ok = recv_data[-1] == checksum(recv_data[2:-1])
            data_ok = data_ok and recv_data[4] == r_cmd
            data_ok = data_ok and recv_data[0:2] == SERVO_FRAME_HEADER
            if data_ok:
                dat_len = recv_data[3]
                if dat_len == 4:
                    return recv_data[5]
                elif dat_len == 5:
                    pos = 0xffff & (recv_data[5] | (0xff00 & (recv_data[6] << 8)))
                    return ctypes.c_int16(pos).value
                elif dat_len == 7:
                    pos1 = 0xffff & (recv_data[5] | (0xff00 & (recv_data[6] << 8)))
                    pos2 = 0xffff & (recv_data[7] | (0xff00 & (recv_data[8] << 8)))
                    return ctypes.c_int16(pos1).value, ctypes.c_int16(pos2).value
        except BaseException as e:
            print(e)
    return None

# ============================================================================
# PWMServoClass.py
# ============================================================================

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

# ============================================================================
# SerialServoClass.py
# ============================================================================

class Serial_Servo():
    MOVE_TIME_WRITE = 1
    MOVE_TIME_READ = 2
    MOVE_TIME_WAIT_WRITE = 7
    MOVE_TIME_WAIT_READ = 8
    MOVE_START = 11
    MOVE_STOP = 12
    ANGLE_OFFSET_ADJUST = 17
    ANGLE_OFFSET_WRITE = 18
    ANGLE_OFFSET_READ = 19
    ANGLE_LIMIT_WRITE = 20
    ANGLE_LIMIT_READ = 21
    VIN_LIMIT_WRITE = 22
    VIN_LIMIT_READ = 23
    TEMP_LIMIT_WRITE = 24
    TEMP_LIMIT_READ = 25
    TEMP_READ = 26
    VIN_READ = 27
    POS_READ = 28
    MOTOR_MODE_WRITE = 29
    MOTOR_MODE_READ = 30
    LOAD_MODE_WRITE = 31
    LOAD_MODE_READ = 32
    LED_CTRL_WRITE = 33
    LED_CTRL_READ = 34
    LED_ERROR_WRITE = 35
    LED_ERROR_READ = 36

    def __init__(self, pi, id):
        self.pi = pi
        self.id = id

    def set_pos(self, posn):
        return serial_servo_write_cmd(self.pi, self.id, self.MOVE_TIME_WRITE, posn[0], posn[1])

    def get_set_pos(self):
        return serial_servo_read_cmd(self.pi, self.id, self.MOVE_TIME_READ)

    def set_standby_pos(self, posn):
        return serial_servo_write_cmd(self.pi, self.id, self.MOVE_TIME_WAIT_WRITE, posn[0], posn[1])

    def trigger(self):
        return serial_servo_write_cmd(self.pi, self.id, self.MOVE_START)

    def stop(self):
        return serial_servo_write_cmd(selfpi, self.id, self.MOVE_STOP)

    def set_offset(self, offset=0):
        result = serial_servo_write_cmd(self.pi, self.id, self.ANGLE_OFFSET_ADJUST, offset)
        if type(result) == str:
            return result
        return serial_servo_write_cmd(self.pi, self.id, self.ANGLE_OFFSET_WRITE)

    def get_offset(self):
        return serial_servo_read_cmd(self.pi, self.id, self.ANGLE_OFFSET_READ)

    offset = property(get_offset, set_offset)

    def set_rotation_limits(self, limits):
        return serial_servo_write_cmd(self.pi, self.id, self.ANGLE_LIMIT_WRITE, limits[0], limits[1])

    def get_rotation_limits(self):
        return serial_servo_read_cmd(self.pi, self.id, self.ANGLE_LIMIT_READ)

    rotation_limits = property(get_rotation_limits, set_rotation_limits)

    def set_vin_limits(self, limits):
        return serial_servo_write_cmd(self.pi, self.id, self.VIN_LIMIT_WRITE, limits[0], limits[1])

    def get_vin_limits(self):
        return serial_servo_read_cmd(self.pi, self.id, self.VIN_LIMIT_READ)

    vin_limits = property(get_vin_limits, set_vin_limits)

    def set_temp_limit(self, m_temp):
        return serial_servo_write_cmd(self.pi, self.id, self.TEMP_LIMIT_WRITE, m_temp)

    def get_temp_limit(self):
        return serial_servo_read_cmd(self.pi, self.id, self.TEMP_LIMIT_READ)

    temp_limit = property(get_temp_limit, set_temp_limit)

    @property
    def temp(self):
        return serial_servo_read_cmd(self.pi, self.id, self.TEMP_READ)

    @property
    def vin(self):
        return serial_servo_read_cmd(self.pi, self.id, self.VIN_READ)

    def get_pos(self):
        return serial_servo_read_cmd(self.pi, self.id, self.POS_READ)

    pos = property(get_pos, set_pos)

    def set_load(self, mode=1):
        return serial_servo_write_cmd(self.pi, self.id, self.LOAD_MODE_WRITE, mode)

    @property
    def unload(self):
        return serial_servo_write_cmd(self.pi, self.id, self.LOAD_MODE_WRITE, 0)

    def get_load_mode(self):
        return serial_servo_read_cmd(self.pi, self.id, self.LOAD_MODE_READ)

    load = property(get_load_mode, set_load)

# ============================================================================
# PTHeadCtrl.py
# ============================================================================

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

# ============================================================================
# 2DIK.py (Optimized)
# ============================================================================

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

# ============================================================================
# LegClass.py
# ============================================================================

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

# ============================================================================
# Hexapod.py (Optimized)
# ============================================================================

pi = pigpio.pi()
SpiderPi = tuple(Leg(pi, id) for id in range(6))
init_pthead(pi)

leg_names = ("Port rear", "Port centre", "Port front", "Starboard rear", "Starboard centre", "Starboard front")
port_rear = 0
port_centre = 1
port_front = 2
sboard_rear = 3
sboard_centre = 4
sboard_front = 5

STAND_POS = (0, 100, 70)
SIT_POS = (0, 100, 20)
LIFT_POS = (0, 100, 40)
TALL_POS = (0, 100, 120)

AEP = (100, 100, 70)
PEP = (-100, 100, 70)
PSP = (-50, 100, 40)
ASP = (50, 100, 40)

def forward_step(leg, factor=1, tim=2000):
    aep = (AEP[0] * factor, AEP[1], AEP[2])
    phase = leg % 2
    if phase == 0:
        SpiderPi[leg].pos = (inverse_kin(leg, PEP), tim * 0.601)
        SpiderPi[leg].pos = (inverse_kin(leg, PSP), tim * 0.133)
        SpiderPi[leg].pos = (inverse_kin(leg, ASP), tim * 0.133)
        SpiderPi[leg].pos = (inverse_kin(leg, aep), tim * 0.133)
    else:
        SpiderPi[leg].pos = (inverse_kin(leg, ASP), tim * 0.133)
        SpiderPi[leg].pos = (inverse_kin(leg, aep), tim * 0.133)
        SpiderPi[leg].pos = (inverse_kin(leg, PEP), tim * 0.601)
        SpiderPi[leg].pos = (inverse_kin(leg, PSP), tim * 0.133)

def backward_step(leg, factor=1, tim=2000):
    pep = (PEP[0] * factor, PEP[1], PEP[2])
    phase = leg % 2
    if phase == 0:
        SpiderPi[leg].pos = (inverse_kin(leg, AEP), tim * 0.601)
        SpiderPi[leg].pos = (inverse_kin(leg, ASP), tim * 0.133)
        SpiderPi[leg].pos = (inverse_kin(leg, PSP), tim * 0.133)
        SpiderPi[leg].pos = (inverse_kin(leg, pep), tim * 0.133)
    else:
        SpiderPi[leg].pos = (inverse_kin(leg, PSP), tim * 0.133)
        SpiderPi[leg].pos = (inverse_kin(leg, pep), tim * 0.133)
        SpiderPi[leg].pos = (inverse_kin(leg, AEP), tim * 0.601)
        SpiderPi[leg].pos = (inverse_kin(leg, ASP), tim * 0.133)

def load():
    for leg in range(6):
        SpiderPi[leg].load = 1

def unload():
    for leg in range(6):
        SpiderPi[leg].unload

def trigger():
    for leg in range (6):
        SpiderPi[leg].trigger

def diag():
    print("Hexapod under test.")
    results = []
    for leg in range(6):
        print(leg_names[leg], "leg")
        result = (SpiderPi[leg].leg_state,)
        print(result)
        results.append(result)
    return tuple(results)

if __name__ == '__main__':
    portinit(pi)
    print(diag())
    # Example of how to use the functions
    # forward_step(port_front)
    # trigger()
