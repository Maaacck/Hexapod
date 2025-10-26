#!/usr/bin/python3
# -*- coding: UTF-8 -*-

from . import RPiExpCom as com

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
        return com.serial_servo_write_cmd(self.pi, self.id, self.MOVE_TIME_WRITE, posn[0], posn[1])

    def get_set_pos(self):
        return com.serial_servo_read_cmd(self.pi, self.id, self.MOVE_TIME_READ)

    def set_standby_pos(self, posn):
        return com.serial_servo_write_cmd(self.pi, self.id, self.MOVE_TIME_WAIT_WRITE, posn[0], posn[1])

    def trigger(self):
        return com.serial_servo_write_cmd(self.pi, self.id, self.MOVE_START)

    def stop(self):
        return com.serial_servo_write_cmd(selfpi, self.id, self.MOVE_STOP)

    def set_offset(self, offset=0):
        result = com.serial_servo_write_cmd(self.pi, self.id, self.ANGLE_OFFSET_ADJUST, offset)
        if type(result) == str:
            return result
        return com.serial_servo_write_cmd(self.pi, self.id, self.ANGLE_OFFSET_WRITE)

    def get_offset(self):
        return com.serial_servo_read_cmd(self.pi, self.id, self.ANGLE_OFFSET_READ)

    offset = property(get_offset, set_offset)

    def set_rotation_limits(self, limits):
        return com.serial_servo_write_cmd(self.pi, self.id, self.ANGLE_LIMIT_WRITE, limits[0], limits[1])

    def get_rotation_limits(self):
        return com.serial_servo_read_cmd(self.pi, self.id, self.ANGLE_LIMIT_READ)

    rotation_limits = property(get_rotation_limits, set_rotation_limits)

    def set_vin_limits(self, limits):
        return com.serial_servo_write_cmd(self.pi, self.id, self.VIN_LIMIT_WRITE, limits[0], limits[1])

    def get_vin_limits(self):
        return com.serial_servo_read_cmd(self.pi, self.id, self.VIN_LIMIT_READ)

    vin_limits = property(get_vin_limits, set_vin_limits)

    def set_temp_limit(self, m_temp):
        return com.serial_servo_write_cmd(self.pi, self.id, self.TEMP_LIMIT_WRITE, m_temp)

    def get_temp_limit(self):
        return com.serial_servo_read_cmd(self.pi, self.id, self.TEMP_LIMIT_READ)

    temp_limit = property(get_temp_limit, set_temp_limit)

    @property
    def temp(self):
        return com.serial_servo_read_cmd(self.pi, self.id, self.TEMP_READ)

    @property
    def vin(self):
        return com.serial_servo_read_cmd(self.pi, self.id, self.VIN_READ)

    def get_pos(self):
        return com.serial_servo_read_cmd(self.pi, self.id, self.POS_READ)

    pos = property(get_pos, set_pos)

    def set_load(self, mode=1):
        return com.serial_servo_write_cmd(self.pi, self.id, self.LOAD_MODE_WRITE, mode)

    @property
    def unload(self):
        return com.serial_servo_write_cmd(self.pi, self.id, self.LOAD_MODE_WRITE, 0)

    def get_load_mode(self):
        return com.serial_servo_read_cmd(self.pi, self.id, self.LOAD_MODE_READ)

    load = property(get_load_mode, set_load)
