import numpy as np
from math import pi
from trajectory_generation import trajectory_generator
from pylx16a.lx16a import *
import time

class hexapod:
    def __init__(self, vx, vy, vz, f_gait=1, leg_height=10, f_hard=100):
        self.phase = np.array([0, pi, 0, pi, 0, pi])
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.f_hard = f_hard
        self.f_gate = f_gait
        self.leg_height = leg_height
        self.leg1 = trajectory_generator(self.vx, self.vy, self.vz, self.leg_height, self.f_hard, self.phase[0], 1)
        self.leg2 = trajectory_generator(self.vx, self.vy, self.vz, self.leg_height, self.f_hard, self.phase[1], 2)
        self.leg3 = trajectory_generator(self.vx, self.vy, self.vz, self.leg_height, self.f_hard, self.phase[2], 3)
        self.leg4 = trajectory_generator(self.vx, self.vy, self.vz, self.leg_height, self.f_hard, self.phase[3], 4)
        self.leg5 = trajectory_generator(self.vx, self.vy, self.vz, self.leg_height, self.f_hard, self.phase[4], 5)
        self.leg6 = trajectory_generator(self.vx, self.vy, self.vz, self.leg_height, self.f_hard, self.phase[5], 6)

    def get_loop_points(self, vx, vy):
        self.leg1.update_vx_vy(vx, vy)
        self.leg2.update_vx_vy(vx, vy)
        self.leg3.update_vx_vy(vx, vy)
        self.leg4.update_vx_vy(vx, vy)
        self.leg5.update_vx_vy(vx, vy)
        self.leg6.update_vx_vy(vx, vy)
        loop_points = []
        Lpoints = []
        for i in range(5 * self.f_hard):
            p1 = self.leg1.anglist()
            p2 = self.leg2.anglist()
            p3 = self.leg3.anglist()
            p4 = self.leg4.anglist()
            p5 = self.leg5.anglist()
            p6 = self.leg6.anglist()
            loop_points.append(p1 + p2 + p3 + p4 + p5 + p6)
        return loop_points

if __name__=="__main__":
    # This is the port for the servo controller.
    # On Linux, it's most likely "/dev/ttyUSB0"
    # On Windows, try "COM3" or similar
    LX16A.initialize("COM3")

    servos = []
    for i in range(1, 19):
        try:
            servo = LX16A(i)
            servo.set_angle_limits(0, 240)
            servos.append(servo)
        except ServoTimeoutError as e:
            print(f"Servo {e.id_} is not responding. Exiting...")
            quit()

    h1 = hexapod(vx=0, vy=0, vz=0)
    loop_points = h1.get_loop_points(vx=2, vy=2)
    loop_points = np.array(loop_points)

    while True:
        for angles in loop_points:
            for i in range(18):
                servos[i].move(angles[i])
            time.sleep(0.02)
