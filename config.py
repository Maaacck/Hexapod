#!/usr/bin/python3
# encoding: utf-8
# Copyright 2020-2022 ians.moyes@gmail.com
#
# Environment configuration variables

# Bus Servo parameters
BS_servo_type = "LX-224HV" # Manufacturer/model of the servo
BS_num_servos = 18 # Number of servos of this type on the robot
BS_rotate_limits = (0, 1000) # Miminum & maximum values for full defection
BS_cont_speed = (-1000, 1000) # Range of continuous rotation speeds
BS_max_speed = 1250 # Maximum rotation speed in positions per second
# HiWonder LX-224HV Bus Servos have a maximum speed of rotation of 0.20sec/60°(@ VIn 11.1V).
# With a maximum rotation of 0 ~ 240° defined as a range of 1-1000.
BS_time_limits = (0, 30000) # Minimum & maximum time durations
BS_offset_limits = (-125, 125)  # Minimum & maximum offset values
# Offsets can be set between angles of -30 ° ~ 30 °
BS_Vin_limits = (9000, 12600) # Voltage limits specified in mV
# Controller allows voltage limits to be set from 4500mV to 14000mV but the operating
# voltage of the LX-224HV is 9-12.6V
BS_temp_limits = (50, 85) # Temperature alarm limit in °C.
# The limit can be set between 50 ~ 100°C.
BS_default_pos = 500 # The default position for 50% rotation

# Bus Serial Servo command codes
BS_MOVE_TIME_WRITE      =  1
BS_MOVE_TIME_READ       =  2
BS_MOVE_TIME_WAIT_WRITE =  7
BS_MOVE_TIME_WAIT_READ  =  8
BS_MOVE_START           = 11
BS_MOVE_STOP            = 12
BS_ID_WRITE             = 13
BS_ID_READ              = 14
BS_ANGLE_OFFSET_ADJUST  = 17
BS_ANGLE_OFFSET_WRITE   = 18
BS_ANGLE_OFFSET_READ    = 19
BS_ANGLE_LIMIT_WRITE    = 20
BS_ANGLE_LIMIT_READ     = 21
BS_VIN_LIMIT_WRITE      = 22
BS_VIN_LIMIT_READ       = 23
BS_TEMP_LIMIT_WRITE     = 24
BS_TEMP_LIMIT_READ      = 25
BS_TEMP_READ            = 26
BS_VIN_READ             = 27
BS_POS_READ             = 28
BS_MOTOR_MODE_WRITE     = 29
BS_MOTOR_MODE_READ      = 30
BS_LOAD_MODE_WRITE      = 31
BS_LOAD_MODE_READ       = 32
BS_LED_CTRL_WRITE       = 33
BS_LED_CTRL_READ        = 34
BS_LED_ERROR_WRITE      = 35
BS_LED_ERROR_READ       = 36
