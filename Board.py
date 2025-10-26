#!/usr/bin/python3
# encoding: utf-8
# Copyright 2020-2022 ians.moyes@gmail.com

import serial
import time
import RPi.GPIO as GPIO

SERVO_FRAME_HEADER = 0x55

serialHandle = serial.Serial("/dev/ttyAMA0", 115200)  # 初始化串口， 波特率为115200

def portInit():  # 配置单线串口为输出
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(17, GPIO.OUT, initial=False)
    GPIO.setup(27, GPIO.OUT, initial=True)

def portWrite():  # 配置单线串口为输出
    GPIO.output(17, 0)
    GPIO.output(27, 1)

def portRead():  # 配置单线串口为输入
    GPIO.output(17, 1)
    GPIO.output(27, 0)

def portReset():
    time.sleep(0.1)
    serialHandle.close()
    portWrite()
    serialHandle.open()
    time.sleep(0.1)

def portOff():
    serialHandle.close()
    GPIO.output(17, 0)
    GPIO.output(27, 0)
