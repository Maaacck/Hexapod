#! usr/bin/env python3
from math import *
from matplotlib import pyplot as plt
import sys
sys.path.append('/hexapod/hexapod_core')
from ik import in_kin as kinematics
from fk_s3r import f_kin
import numpy as np

# The `trajectory_generator` class generates a continuous trajectory for leg motion based on input
# parameters such as velocity, leg height, and phase angle.


class trajectory_generator:
    def __init__(self, vx, vy, vz, leg_height, f_hard, phase_angle, leg_id):
        # This is the initialization of the `trajectory_generator` class. It sets the values of the input
        # parameters `vx`, `vy`, `vz`, `leg_height`, and `f_hard` as instance variables. It also sets the
        # value of `f_gate` to 2 and calculates the value of `n` based on the `phase_angle` input. It
        # initializes the `x0` and `y0` coordinates to 0.0 and 0.10 respectively, and calculates the values of
        # `x1` and `y1` based on `vx`, `vy`, and `f_gate`. It sets the initial coordinates as `[x0, y0, 0]`
        # and sets the `flag` variable to 1.
        self.vx = vx
        self.vz = vz
        self.vy = vy
        self.leg_height = leg_height
        self.f_hard = f_hard
        self.f_gate = 2
        self.leg_id = leg_id
        self.leg_kin = kinematics()
        self.n = int(phase_angle * self.f_hard / (2 * np.pi * self.f_gate))
        if self.leg_id == 1 or self.leg_id == 2 or self.leg_id == 3:
            self.x0, self.y0 = 0.0, -0.10
        if self.leg_id == 4 or self.leg_id == 5 or self.leg_id == 6:
            self.x0, self.y0 = 0.0, 0.10
        self.x1 = self.x0 + self.vx * 2 / self.f_gate
        self.y1 = self.y0 + self.vy * 2 / self.f_gate
        self.cord = [self.x0, self.y0, 0]
        self.flag = 1


        # This block of code is checking if the current value of `self.n` (which is calculated based on the
        # `phase_angle` input) exceeds the maximum value of `n_max` that can be achieved with the given values
        # of `self.f_hard` and `self.f_gate`. If it does, then it calculates the remainder `r` when `self.n`
        # is divided by `n_max`, and subtracts `r * n_max` from `self.n`. If `r` is odd, it sets the value of
        # `self.flag` to `-1`, updates the `x` coordinate of the current position to `self.vx / (2 *
        # self.f_gate)`, and swaps the values of `self.x0` and `self.x1` using the `swap()` method. This is
        # done to ensure that the trajectory of the leg motion is continuous and does not have any sudden
        # jumps or discontinuities.
        n_max = int((self.f_hard / (2 * self.f_gate)))
        if n_max <= self.n:
            r = int(self.n / n_max)
            self.n = self.n - r * n_max
            if r % 2 != 0:
                self.flag = self.flag * -1
                self.cord[0] = self.vx / (2 * self.f_gate)
                self.update_vx_vy()
                self.swap()

    def get_next(self):
        """
        The function calculates the next position of an object based on its current position, velocity, and
        frequency.
        :return: The function `get_next` returns the updated `self.cord` list.
        """
        time_left = (2 / self.f_gate) - (self.n / self.f_hard)
        # print(time_left)
        if time_left > 0:
            self.cord[0] = (
                self.cord[0] + ((self.x1 - self.cord[0]) / time_left) / self.f_hard
            )
            self.cord[1] = (
                self.cord[1] + ((self.y1 - self.cord[1]) / time_left) / self.f_hard
            )
            self.cord[2] = (
                0.5
                * (self.flag + 1)
                * (self.vz / self.f_gate)
                * sin((pi * self.n * self.f_gate) / (2 * self.f_hard))
            )
            self.cord[2] = -self.leg_height + self.cord[2]
            # print( self.cord)

        if abs(time_left) <= (1 / self.f_hard):
            self.flag = self.flag * -1
            if self.flag == -1:
                self.update_vx_vy()
            self.swap()
            self.n = 0
        else:
            self.n += 1

        return self.cord

    def swap(self):
        """
        The "swap" function swaps the values of four variables.
        """
        self.x1, self.y1, self.x0, self.y0 = self.x0, self.y0, self.x1, self.y1

    def update_vx_vy(self):
        """
        This function updates the values of x1 and y1 based on the current values of x0, y0, vx, vy, and
        f_gate.
        """
        self.x1 = self.x0 + self.vx * 2 / self.f_gate
        self.y1 = self.y0 + self.vy * 2 / self.f_gate

    def change_vx_vy(self,vx,vy):
        if vx != 0 :
            self.ang1_shift = atan(abs(vy)/abs(vx))
        else:
            self.ang1_shift =  pi/2
        self.vx = sqrt(vx**2 + vy**2)

    def anglist(self):
        x, y, z = self.get_next()
        p, q, r = self.leg_kin.ik(x,y,z)

        return p,q,r, x, y, z
    
    def pointlist(self):
        x, y, z = self.get_next()
        
        return x,y,z


# This code block is the main program that creates an instance of the `trajectory_generator` class
# with specific input parameters, generates a trajectory for leg motion using the `get_next()` method
# of the `trajectory_generator` class, calculates the inverse kinematics angles using the `ik()`
# method of the `kinematics` class, calculates the forward kinematics coordinates using the `fk()`
# method of the `f_kin` class, and plots the trajectory and angles using the `matplotlib` library.
if __name__ == "__main__":
    tg = trajectory_generator(0.03, 0, 0.04, 0.132, 100, np.pi, 2)

    x_cor, y_cor, z_cor = [], [], []
    ang_1, ang_2, ang_3 = [], [], []
    # x_f ,y_f , z_f = [] , [] , []

    leg = kinematics()

    for _ in range(50):
        x, y, z = tg.get_next()
        x_cor.append(x)
        y_cor.append(y)
        z_cor.append(z)

        t1, t2, t3 = leg.ik(x, y, z)
        t1, t2, t3 = np.rad2deg(t1), np.rad2deg(t2), np.rad2deg(t3)
        ang_1.append(t1)
        ang_2.append(t2)
        ang_3.append(t3)

    fkin = f_kin()
    xf, yf, zf = [], [], []
    for i in range(50):
        xa, ya, za = fkin.fk(ang_1[i], ang_2[i], ang_3[i])
        xf.append(xa)
        yf.append(ya)
        zf.append(za)

    # import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    plt.title("traj x vs traj z")
    ax.plot3D(x_cor, y_cor, z_cor)
    plt.show()
    plt.plot(x_cor, z_cor)
    plt.show()

    plt.plot(ang_1)
    plt.plot(ang_2)
    plt.plot(ang_3)
    plt.show()

    # plt.plot(xf, zf)
    # plt.show()
