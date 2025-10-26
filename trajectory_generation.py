#! usr/bin/env python3
from math import *
from matplotlib import pyplot as plt
from ik import in_kin as kinematics
import numpy as np

# The `trajectory_generator` class generates a continuous trajectory for leg motion based on input
# parameters such as velocity, leg height, and phase angle.


class trajectory_generator:
    def __init__(self, vx, vy, vz, leg_height, f_hard, phase_angle, leg_id):
        self.vx, self.vy, self.vz = vx,vy, vz
        self.n = 0
        self.f_hard = f_hard
        self.f_gait = 1
        self.leg_height = leg_height
        self.x0, self.y0, self.z0  = 0,0,0
        self.x1 = self.x0 + self.vx/(2*self.f_gait)
        self.y1 = self.y0 + self.vy/(2*self.f_gait)
        self.leg_kin = kinematics()
        self.leg_id = leg_id
        self.n = int(phase_angle * self.f_hard / (2 * np.pi * self.f_gait))
        if self.leg_id == 1 or self.leg_id == 2 or self.leg_id == 3:
            self.x0, self.y0 = 0.0, 0.10
        if self.leg_id == 4 or self.leg_id == 5 or self.leg_id == 6:
            self.x0, self.y0 = 0.0, -0.10
        self.cord = [self.x0, self.y0, 0] 
        self.flag = 1

        n_max = int((self.f_hard / (2 * self.f_gait)))
        if n_max <= self.n:
            r = int(self.n / n_max)
            self.n = self.n - r * n_max
            if r % 2 != 0:
                self.flag = self.flag * -1
                self.cord[0] = self.vx / (2 * self.f_gait)
                self.update_vx_vy()
                self.swap()

    def get_next(self):

        time_left = 1/(2*self.f_gait) - self.n/(self.f_hard)
        # print(time_left)
        if time_left>0:

            self.cord[0] = self.x0 + self.flag*self.vx*self.n/self.f_hard
            self.cord[1] = self.y0 + self.flag*self.vy*self.n/self.f_hard
            

            self.cord[2] = 0.5* (self.flag + 1)*(self.vz/(1*self.f_gait))*sin((2*pi*self.n*self.f_gait)/self.f_hard)
            self.cord[2] = -self.leg_height + self.cord[2]
            # print(self.cord[2])

        
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
        self.x1 = self.x0 + self.vx / (2*self.f_gait)
        self.y1 = self.y0 + self.vy / (2*self.f_gait)

    def anglist(self):
        x, y, z = self.get_next()
        p, q, r = self.leg_kin.ik(x,y,z)
        p, q , r = np.rad2deg(p), np.rad2deg(q), np.rad2deg(r)
        return p,q,r


if __name__ == "__main__":
    tg = trajectory_generator(0.03, 0.0, 0.04, 0.1, 100,1*pi, 4)

    leg = kinematics()
    x_cor, y_cor, z_cor = [], [], []
    ang_1, ang_2, ang_3 = [], [], []

#103
    for i in range(103):
        x, y, z = tg.get_next()
        x_cor.append(x)
        y_cor.append(y)
        z_cor.append(z)

        t1, t2, t3 = leg.ik(x, y, z)
        t1, t2, t3 = np.rad2deg(t1), np.rad2deg(t2), np.rad2deg(t3)
        ang_1.append(t1)
        ang_2.append(t2)
        ang_3.append(t3)


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

    

            






         

    
