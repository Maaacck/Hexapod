# Hexapod Robot Control Software

This repository contains the Python software to control a hexapod robot.

## Code Structure

The code is organized into the following structure:

```
.
├── lib/
│   ├── __init__.py
│   ├── IK.py
│   ├── Leg.py
│   ├── PTHeadCtrl.py
│   ├── PWMServo.py
│   ├── RPiExpCom.py
│   └── SerialServo.py
├── main.py
├── LICENSE
└── README.md
```

- `main.py`: The main script to run the hexapod robot.
- `lib/`: This directory contains the core modules for controlling the robot.
  - `IK.py`: Inverse kinematics calculations.
  - `Leg.py`: `Leg` class to control a single leg.
  - `PTHeadCtrl.py`: Functions for controlling the pan-tilt head.
  - `PWMServo.py`: `PWM_Servo` class for controlling PWM servos.
  - `RPiExpCom.py`: Low-level communication with the Raspberry Pi expansion board.
  - `SerialServo.py`: `Serial_Servo` class for controlling serial bus servos.

## Running the Code

To run the software, execute the `main.py` script:

```bash
python3 main.py
```