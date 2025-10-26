#!/usr/bin/python3
# encoding: utf-8
# Copyright 2020-2022 ians.moyes@gmail.com

import Board
import ActionGroup as AG

if __name__ == '__main__':
    Board.portInit()
    AG.stand_up()
    AG.wave_leg()
    AG.sit_down()
    Board.portOff()