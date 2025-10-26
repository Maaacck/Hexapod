#!/usr/bin/python3
# -*- coding: UTF-8 -*-

import serial
import pigpio
import time
import ctypes

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
