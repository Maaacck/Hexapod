#!/usr/bin/python3
# encoding: utf-8
# Copyright 2020-2022 ians.moyes@gmail.com

import time
import ctypes
import Board
import config

class BusServo:
    def __init__(self, servo_id):
        self.servo_id = servo_id

    def checksum(self, buf):
        sum = 0x00
        for b in buf:
            sum += b
        sum = ~sum
        return sum & 0xff

    def write_cmd(self, w_cmd, dat1=None, dat2=None):
        Board.portWrite()
        buf = bytearray()
        buf.append(self.servo_id)

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

        buf.append(self.checksum(buf))
        frame = bytearray([Board.SERVO_FRAME_HEADER, Board.SERVO_FRAME_HEADER]) + buf
        Board.serialHandle.write(frame)
        return True

    def read_cmd(self, r_cmd):
        prev = time.time()
        Board.serialHandle.flushInput()
        while time.time() < prev + 1:
            write_ok = self.write_cmd(r_cmd)
            if write_ok:
                time.sleep(0.00034)
                Board.portRead()
                for i in range(20):
                    results = self.collect_data(r_cmd)
                    if results is not None:
                        return results
        return "Comms Error"

    def collect_data(self, r_cmd):
        time.sleep(0.00034)
        count = Board.serialHandle.inWaiting()
        if count != 0:
            recv_data = Board.serialHandle.read(count)
            Board.serialHandle.flushInput()
            try:
                data_ok = recv_data[-1] == self.checksum(recv_data[2:-1])
                data_ok = data_ok and recv_data[4] == r_cmd
                data_ok = data_ok and recv_data[0] == Board.SERVO_FRAME_HEADER and recv_data[1] == Board.SERVO_FRAME_HEADER
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

    def move_time_write(self, position, time_ms):
        return self.write_cmd(config.BS_MOVE_TIME_WRITE, position, time_ms)

    def move_time_read(self):
        return self.read_cmd(config.BS_MOVE_TIME_READ)

    def move_time_wait_write(self, position, time_ms):
        return self.write_cmd(config.BS_MOVE_TIME_WAIT_WRITE, position, time_ms)

    def move_time_wait_read(self):
        return self.read_cmd(config.BS_MOVE_TIME_WAIT_READ)

    def move_start(self):
        return self.write_cmd(config.BS_MOVE_START)

    def move_stop(self):
        return self.write_cmd(config.BS_MOVE_STOP)

    def id_write(self, servo_id):
        return self.write_cmd(config.BS_ID_WRITE, servo_id)

    def id_read(self):
        return self.read_cmd(config.BS_ID_READ)

    def angle_offset_adjust(self, offset):
        return self.write_cmd(config.BS_ANGLE_OFFSET_ADJUST, offset)

    def angle_offset_write(self):
        return self.write_cmd(config.BS_ANGLE_OFFSET_WRITE)

    def angle_offset_read(self):
        return self.read_cmd(config.BS_ANGLE_OFFSET_READ)

    def angle_limit_write(self, min_angle, max_angle):
        return self.write_cmd(config.BS_ANGLE_LIMIT_WRITE, min_angle, max_angle)

    def angle_limit_read(self):
        return self.read_cmd(config.BS_ANGLE_LIMIT_READ)

    def vin_limit_write(self, min_vin, max_vin):
        return self.write_cmd(config.BS_VIN_LIMIT_WRITE, min_vin, max_vin)

    def vin_limit_read(self):
        return self.read_cmd(config.BS_VIN_LIMIT_READ)

    def temp_max_limit_write(self, max_temp):
        return self.write_cmd(config.BS_TEMP_LIMIT_WRITE, max_temp)

    def temp_max_limit_read(self):
        return self.read_cmd(config.BS_TEMP_LIMIT_READ)

    def temp_read(self):
        return self.read_cmd(config.BS_TEMP_READ)

    def vin_read(self):
        return self.read_cmd(config.BS_VIN_READ)

    def pos_read(self):
        return self.read_cmd(config.BS_POS_READ)

    def motor_mode_write(self, mode, speed):
        return self.write_cmd(config.BS_MOTOR_MODE_WRITE, mode, speed)

    def motor_mode_read(self):
        return self.read_cmd(config.BS_MOTOR_MODE_READ)

    def load_write(self, mode):
        return self.write_cmd(config.BS_LOAD_MODE_WRITE, mode)

    def load_read(self):
        return self.read_cmd(config.BS_LOAD_MODE_READ)

    def led_ctrl_write(self, mode):
        return self.write_cmd(config.BS_LED_CTRL_WRITE, mode)

    def led_ctrl_read(self): 
        return self.read_cmd(config.BS_LED_CTRL_READ)

    def led_error_write(self, mode):
        return self.write_cmd(config.BS_LED_ERROR_WRITE, mode)

    def led_error_read(self):
        return self.read_cmd(config.BS_LED_ERROR_READ)
