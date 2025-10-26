#! usr/bin/env python3
from math import *
from matplotlib import pyplot as plt
import sys
import os
sys.path.append('/hexapod/hexapod_core')
# sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../../')
# from s3r_kin import kinematics
from trajectory_generation import trajectory_generator
# from mpl_toolkits import mplot3d


class hexapod:

    def __init__(self, vx, vy, vz):

        self.legs = {1:0, 2:pi, 3:0, 4:pi, 5:0, 6:pi}
        # self.phase_angs = [0, pi, 0, pi, 0, pi]
        self.phase_angs = list(self.legs.values())
        self.leg_ids = list(self.legs.keys())

        self.vx, self.vy, self.vz = vx, vy, vz
        self.f_hard = 100
        self.f_gate= 1
        self.leg_height = 0.1

        leg_traj = []
        leg = trajectory_generator(vx=self.vx, vy=self.vy, vz=self.vz, leg_height=self.leg_height,
                                     f_hard=self.f_hard, phase_angle=self.phase_angs[0], leg_id = self.leg_ids[0])
        leg_traj.append(leg)
        leg = trajectory_generator(vx=self.vx, vy=self.vy, vz=self.vz, leg_height=self.leg_height,
                                     f_hard=self.f_hard, phase_angle=self.phase_angs[3], leg_id = self.leg_ids[3])
        leg_traj.append(leg)
        self.leg_traj = leg_traj

    def traj_phase_shift(self, loop_points):
        leg1 = leg3 = [loop_points[0:99,0], loop_points[0:99,1], loop_points[0:99,2]]
        leg2 = [loop_points[50:149,0], loop_points[50:149,1], loop_points[50:149,2]]

        leg4 = leg6 = [loop_points[50:149,3], loop_points[50:149,4], loop_points[50:149,5]]
        leg5 = [loop_points[0:99,3], loop_points[0:99,4], loop_points[0:99,5]]
        legs = [leg1, leg2, leg3, leg4, leg5, leg6]

        legs = []
        for n in range(6):
            leg = []
            n = int(self.phase_angs[n] * self.f_hard / (2 * np.pi * self.f_gate))
            leg.extend(loop_points[2*n:2*n+self.f_hard-1,0])
            leg.extend(loop_points[2*n:2*n+self.f_hard-1,1])
            leg.extend(loop_points[2*n:2*n+self.f_hard-1,2])
            legs.append(leg)
        return legs

    def get_loop_points(self, vx, vy):
        self.vx, self.vy = vx, vy
        loop_points = []
        Lpoints = []
        # for i in range(2):
        #     self.leg_traj[i].change_vx_vy(vx, vy)
        for i in range(5*self.f_hard):
            points = []
            p = []
            for j in range(2):
                points.extend(self.leg_traj[j].anglist())
                # else:
                #     key=0
                #     points.extend(self.leg_traj[j].anglist(key))
            loop_points.append(points)

        return loop_points 
    


if __name__=="__main__":
    quad = hexapod( 0.1, 0, 0.05)
    loop_points = quad.get_loop_points(0.1, 0)
    # print( np.array(loop_points))
    import numpy as np
    loop_points = np.array(loop_points)

    legs = quad.traj_phase_shift(loop_points)
    loop_points = np.array(legs)


    # print(len(loop_points[:,0]))
    ang1 = loop_points[0,:99]
    ang2 = loop_points[0,100:198]
    ang3 = loop_points[0,199:297]
    ang4 = loop_points[1,:99]
    ang5 = loop_points[1,100:198]
    ang6 = loop_points[1,199:297]
    ang7 = loop_points[2,:99]
    ang8 = loop_points[2,100:198]
    ang9 = loop_points[2,199:297]
    ang10 = loop_points[3,:99]
    ang11 = loop_points[3,100:198]
    ang12 = loop_points[3,199:297]
    ang13 = loop_points[4,:99]
    ang14 = loop_points[4,100:198]
    ang15 = loop_points[4,199:297] 
    ang16 = loop_points[5,:99]
    ang17 = loop_points[5,100:198]
    ang18 = loop_points[5,199:297]
    
    
    

    plt.plot(ang1)
    plt.plot(ang2)
    plt.plot(ang3)
    plt.show()
    # plt.plot(ang4)
    # plt.plot(ang5)
    # plt.plot(ang6)
    # plt.show()
    # plt.plot(ang7)
    # plt.plot(ang8)
    # plt.plot(ang9)
    # plt.show()
    # plt.plot(ang10)
    # plt.plot(ang11)
    # plt.plot(ang12)
    # plt.show()
    # plt.plot(ang13)
    # plt.plot(ang14)
    # plt.plot(ang15)
    # plt.show()
    # plt.plot(ang16)
    # plt.plot(ang17)
    # plt.plot(ang18)
    # plt.show()

    # x = []
    # y = []
    # z = []
    # for i in range(18):
    #     x.append(Lpoints) 
    # fig = plt.figure()
    # ax = plt.axes(projection="3d")
    # plt.title("traj x vs traj z")
    # # ax.plot3D(Lpoints[:,0], Lpoints[:,1], Lpoints[:,2])
    # # ax.plot3D(Lpoints[:,3], Lpoints[:,4], Lpoints[:,5])
    # # ax.plot3D(Lpoints[:,6], Lpoints[:,7], Lpoints[:,8])
    # # ax.plot3D(Lpoints[:,9], Lpoints[:,10], Lpoints[:,11])
    # # ax.plot3D(Lpoints[:,12], Lpoints[:,13], Lpoints[:,13])
    # # ax.plot3D(Lpoints[:,15], Lpoints[:,16], Lpoints[:,17])
    # # plt.show()

    # plt.plot(Lpoints[:,2])
    # plt.plot(Lpoints[:,5])
    # # plt.plot(Lpoints[:,8])
    # # plt.plot(Lpoints[:,11])
    # # plt.plot(Lpoints[:,13])
    # # plt.plot(Lpoints[:,17])
    # plt.show()