#!/usr/bin/env python3
"""
BusServoCmd.py — Hiwonder/LOBOT bus-servo driver for Raspberry Pi (direct bus via UART).
- Uses /dev/serial0 @ 115200 by default.
- Optional half‑duplex direction control pins: set dir_pins=(tx_en_pin, rx_en_pin) in BOARD numbering.
- Packet: 0x55 0x55 | ID | LEN | CMD | PARAMS... | CHK where CHK = low 8 bits of two's‑complement sum.
"""
import time
import serial
from typing import Optional, Tuple

# --- LOBOT command codes (subset) ---
LOBOT_SERVO_MOVE_TIME_WRITE      = 1
LOBOT_SERVO_MOVE_TIME_READ       = 2
LOBOT_SERVO_MOVE_START           = 11
LOBOT_SERVO_MOVE_STOP            = 12
LOBOT_SERVO_ID_WRITE             = 13
LOBOT_SERVO_ID_READ              = 14
LOBOT_SERVO_ANGLE_OFFSET_ADJUST  = 17
LOBOT_SERVO_ANGLE_OFFSET_WRITE   = 18
LOBOT_SERVO_ANGLE_OFFSET_READ    = 19
LOBOT_SERVO_ANGLE_LIMIT_WRITE    = 20
LOBOT_SERVO_VIN_LIMIT_WRITE      = 22
LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE = 24
LOBOT_SERVO_TEMP_READ            = 26
LOBOT_SERVO_VIN_READ             = 27
LOBOT_SERVO_POS_READ             = 28
LOBOT_SERVO_OR_MOTOR_MODE_WRITE  = 29
LOBOT_SERVO_OR_MOTOR_MODE_READ   = 30
LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE = 31
LOBOT_SERVO_LOAD_OR_UNLOAD_READ  = 32
LOBOT_SERVO_LED_CTRL_WRITE       = 33
LOBOT_SERVO_LED_CTRL_READ        = 34
LOBOT_SERVO_ANGLE_OFFSET_CLR     = 36
LOBOT_SERVO_CURRENT_READ         = 69

BROADCAST_ID = 0xFE

class BusServo:
    def __init__(self, port: str = "/dev/serial0", baud: int = 115200, timeout: float = 0.05,
                 dir_pins: Optional[Tuple[int,int]] = None):
        """
        :param port: serial device (use /dev/serial0 on Pi 4)
        :param baud: bus baud (115200 typical for Hiwonder servos)
        :param timeout: read/write timeout (seconds)
        :param dir_pins: (tx_enable_pin, rx_enable_pin) BOARD numbering; None if not used
        """
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout, write_timeout=timeout)
        self.dir_pins = dir_pins
        if self.dir_pins:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BOARD)  # using BOARD numbers
            tx_en, rx_en = self.dir_pins
            GPIO.setup(tx_en, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(rx_en, GPIO.OUT, initial=GPIO.HIGH)

    # --- low-level helpers ---
    @staticmethod
    def _checksum(frame_wo_chk: bytes) -> int:
        return ((~sum(frame_wo_chk) + 1) & 0xFF)

    def _tx_mode(self):
        if self.dir_pins:
            import RPi.GPIO as GPIO
            tx_en, rx_en = self.dir_pins
            GPIO.output(tx_en, GPIO.HIGH)
            GPIO.output(rx_en, GPIO.LOW)

    def _rx_mode(self):
        if self.dir_pins:
            import RPi.GPIO as GPIO
            tx_en, rx_en = self.dir_pins
            GPIO.output(tx_en, GPIO.LOW)
            GPIO.output(rx_en, GPIO.HIGH)

    def write_cmd(self, sid: int, cmd: int, params: bytes = b""):
        assert 0 <= sid <= 254, "servo id must be 0..254"
        length = 3 + len(params)  # LEN includes CMD + params + CHK
        core = bytes([sid & 0xFF, length & 0xFF, cmd & 0xFF]) + params
        chk = self._checksum(core)
        pkt = b"\x55\x55" + core + bytes([chk])
        self._tx_mode()
        self.ser.write(pkt)
        self.ser.flush()
        time.sleep(0.0006)
        self._rx_mode()

    def read_reply(self, expected_cmd: int, expected_min_len: int = 6):
        start = time.time()
        buf = bytearray()
        while time.time() - start < (self.ser.timeout or 0.05) + 0.05:
            b = self.ser.read(1)
            if not b:
                continue
            buf += b
            if len(buf) >= 2 and buf[-2:] == b"\x55\x55":
                rest = self.ser.read(3)
                if len(rest) < 3:
                    return None
                buf += rest
                sid = buf[-3]
                length = buf[-2]
                cmd = buf[-1]
                payload_len = length - 3
                payload = self.ser.read(payload_len)
                if len(payload) < payload_len:
                    return None
                buf += payload
                # verify checksum
                core = bytes([sid, length, cmd]) + payload[:-1]
                chk = self._checksum(core)
                if payload[-1] != chk:
                    return None
                if cmd != expected_cmd or len(buf) < expected_min_len:
                    continue
                return bytes(buf)
        return None

    # --- high-level ---
    def move_time_write(self, sid: int, pulse: int, time_ms: int):
        pulse = max(0, min(1000, int(pulse)))
        time_ms = max(0, min(30000, int(time_ms)))
        params = bytes([pulse & 0xFF, (pulse >> 8) & 0xFF, time_ms & 0xFF, (time_ms >> 8) & 0xFF])
        self.write_cmd(sid, LOBOT_SERVO_MOVE_TIME_WRITE, params)

    def move_start(self, sid: int):
        self.write_cmd(sid, LOBOT_SERVO_MOVE_START)

    def move_stop(self, sid: int):
        self.write_cmd(sid, LOBOT_SERVO_MOVE_STOP)

    def unload(self, sid: int):
        self.write_cmd(sid, LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE, bytes([0]))

    def load(self, sid: int):
        self.write_cmd(sid, LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE, bytes([1]))

    def pos_read(self, sid: int):
        self.write_cmd(sid, LOBOT_SERVO_POS_READ)
        rep = self.read_reply(LOBOT_SERVO_POS_READ, expected_min_len=8)
        if not rep:
            return None
        pos_l = rep[-3]
        pos_h = rep[-2]
        return (pos_h << 8) | pos_l

    def vin_read(self, sid: int):
        self.write_cmd(sid, LOBOT_SERVO_VIN_READ)
        rep = self.read_reply(LOBOT_SERVO_VIN_READ, expected_min_len=8)
        if not rep:
            return None
        v_l = rep[-3]
        v_h = rep[-2]
        return (v_h << 8) | v_l

    def temp_read(self, sid: int):
        self.write_cmd(sid, LOBOT_SERVO_TEMP_READ)
        rep = self.read_reply(LOBOT_SERVO_TEMP_READ, expected_min_len=7)
        if not rep:
            return None
        return rep[-2]
