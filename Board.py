#!/usr/bin/env python3
"""
Board.py — Minimal helper layer for Hiwonder bus servos (direct bus).
If you are using the USB Servo Controller, talk to it via its own serial protocol instead.
"""
from typing import Optional
from BusServoCmd import BusServo

_pwm_servo_angle = [90.0] * 6   # degrees (bookkeeping)
_pwm_servo_pulse = [1500] * 6   # us (bookkeeping)

_bus: Optional[BusServo] = None

def init_bus(port: str = "/dev/serial0", baud: int = 115200, dir_pins=None) -> BusServo:
    global _bus
    _bus = BusServo(port=port, baud=baud, dir_pins=dir_pins)
    return _bus

# --- PWM helpers (bookkeeping only) ---
def setPWMServoAngle(index: int, angle: float) -> float:
    if not (1 <= index <= 6):
        raise AttributeError(f"Invalid PWM Servo index: {index}")
    i = index - 1
    angle = max(0.0, min(180.0, float(angle)))
    _pwm_servo_angle[i] = angle
    _pwm_servo_pulse[i] = int(500 + (angle / 180.0) * 2000)
    return _pwm_servo_angle[i]

def getPWMServoAngle(index: int) -> float:
    if not (1 <= index <= 6):
        raise AttributeError(f"Invalid PWM Servo index: {index}")
    return _pwm_servo_angle[index - 1]

def getPWMServoPulse(index: int) -> int:
    if not (1 <= index <= 6):
        raise AttributeError(f"Invalid PWM Servo index: {index}")
    return _pwm_servo_pulse[index - 1]

# --- Bus-servo helpers ---
def setBusServoPulse(servo_id: int, pulse: int, time_ms: int = 500):
    if _bus is None:
        raise RuntimeError("Bus not initialized. Call init_bus() first.")
    _bus.move_time_write(servo_id, pulse, time_ms)

def getBusServoPulse(servo_id: int):
    if _bus is None:
        raise RuntimeError("Bus not initialized. Call init_bus() first.")
    return _bus.pos_read(servo_id)

def unloadBusServo(servo_id: int):
    if _bus is None:
        raise RuntimeError("Bus not initialized. Call init_bus() first.")
    _bus.unload(servo_id)

def loadBusServo(servo_id: int):
    if _bus is None:
        raise RuntimeError("Bus not initialized. Call init_bus() first.")
    _bus.load(servo_id)

def resetBusServoPulse(servo_id: int):
    setBusServoPulse(servo_id, 500, 500)
