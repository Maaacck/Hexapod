from math import sin, pi
import matplotlib.pyplot as plt
import numpy as np
from ik import kinematics

class trajectory_generator:
    def __init__(self, vx, vy, vz, leg_height, f_hard, phase_angle, leg_id):
        self.f_gait = 1
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.leg_height = leg_height
        self.f_hard = f_hard
        self.phase_angle = phase_angle
        self.leg_id = leg_id
        self.kin = kinematics()

        if self.leg_id > 3:
            self.x0 = -3
            self.y0 = 0
        else:
            self.x0 = 3
            self.y0 = 0
        self.z0 = 0

        self.x1 = self.x0 - self.vx / (2 * self.f_gait)
        self.y1 = self.y0 - self.vy / (2 * self.f_gait)

        self.n = int(self.f_hard * self.phase_angle / (2 * pi * self.f_gait))
        self.flag = 1

    def get_next(self):
        if self.n < self.f_hard / (2 * self.f_gait):
            if self.flag:
                x = self.x0 + 2 * (self.x1 - self.x0) * self.f_gait * self.n / self.f_hard
                y = self.y0 + 2 * (self.y1 - self.y0) * self.f_gait * self.n / self.f_hard
                z = self.leg_height * sin(2 * pi * self.f_gait * self.n / self.f_hard)
            else:
                x = self.x1 + 2 * (self.x0 - self.x1) * self.f_gait * self.n / self.f_hard
                y = self.y1 + 2 * (self.y0 - self.y1) * self.f_gait * self.n / self.f_hard
                z = 0
        else:
            self.n = 0
            self.flag = not self.flag
            if self.flag:
                x = self.x0
                y = self.y0
                z = 0
                self.swap()
                self.update_vx_vy(self.vx, self.vy)
            else:
                x = self.x1
                y = self.y1
                z = 0
        self.n += 1
        return x, y, z

    def swap(self):
        self.x0, self.x1 = self.x1, self.x0
        self.y0, self.y1 = self.y1, self.y0

    def update_vx_vy(self, vx, vy):
        self.vx = vx
        self.vy = vy
        self.x1 = self.x0 - self.vx / (2 * self.f_gait)
        self.y1 = self.y0 - self.vy / (2 * self.f_gait)

    def anglist(self):
        x, y, z = self.get_next()
        pitch, yaw, roll = self.kin.in_kin(x, y, z, self.leg_id)
        return [np.rad2deg(pitch), np.rad2deg(yaw), np.rad2deg(roll)]

if __name__ == "__main__":
    t1 = trajectory_generator(vx=2, vy=2, vz=0, leg_height=10, f_hard=100, phase_angle=pi, leg_id=4)
    kin = kinematics()
    points = []
    angle = []
    for i in range(103):
        x, y, z = t1.get_next()
        points.append([x, y, z])
        pitch, yaw, roll = kin.in_kin(x, y, z, 4)
        angle.append([pitch, yaw, roll])
    points = np.array(points)
    angle = np.array(angle)
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    pitch = angle[:, 0]
    yaw = angle[:, 1]
    roll = angle[:, 2]
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.plot(x, y, z)
    plt.show()
    plt.plot(x, z)
    plt.show()
    plt.plot(pitch)
    plt.plot(yaw)
    plt.plot(roll)
    plt.show()
