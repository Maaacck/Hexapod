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
        self.f_gate= 2
        self.leg_height = 0.1

        leg_traj = []
        for i in range(6):
            leg = trajectory_generator(vx=self.vx, vy=self.vy, vz=self.vz, leg_height=self.leg_height,
                                     f_hard=self.f_hard, phase_angle=self.phase_angs[i], leg_id = self.leg_ids[i])
            leg_traj.append(leg)
        self.leg_traj = leg_traj


    def get_loop_points(self, vx, vy):
        self.vx, self.vy = vx, vy
        loop_points = []
        Lpoints = []
        # for i in range(6):
        #     self.leg_traj[i].change_vx_vy(vx, vy)
        for i in range(5*self.f_hard):
            points = []
            p = []
            for j in range(6):
                # t1,t2,t3,x,y,z=self.leg_traj[j].anglist()
                # points.extend([t1,t2,t3])
                # p.extend([x,y,z])
                points.extend(self.leg_traj[j].anglist())
                
                # else:
                #     key=0
                #     points.extend(self.leg_traj[j].anglist(key))
            loop_points.append(points)
            Lpoints.append(p)
        return loop_points, Lpoints  
    

if __name__=="__main__":
    quad = hexapod( 0.04, 0, 0.05)
    loop_points, Lpoints = quad.get_loop_points(0.1, 0)
    # print( np.array(loop_points))
    import numpy as np
    loop_points = np.array(loop_points)
    print(len(loop_points[:,0]))
    from matplotlib import pyplot as plt
    ang1 = loop_points[:,0]
    ang2 = loop_points[:,1]
    ang3 = loop_points[:,2]
    ang4 = loop_points[:,3]
    ang5 = loop_points[:,4]
    ang6 = loop_points[:,5]
    ang7 = loop_points[:,6]
    ang8 = loop_points[:,7]
    ang9 = loop_points[:,8]
    ang10 = loop_points[:,9]
    ang11 = loop_points[:,10]
    ang12 = loop_points[:,11]
    ang13 = loop_points[:,12]
    ang14 = loop_points[:,13]
    ang15 = loop_points[:,14]
    ang16 = loop_points[:,15]
    ang17 = loop_points[:,16]
    ang18 = loop_points[:,17]
    

    plt.plot(ang1)
    plt.plot(ang2)
    plt.plot(ang3)
    plt.show()
    plt.plot(ang4)
    plt.plot(ang5)
    plt.plot(ang6)
    plt.show()
    plt.plot(ang7)
    plt.plot(ang8)
    plt.plot(ang9)
    plt.show()
    plt.plot(ang10)
    plt.plot(ang11)
    plt.plot(ang12)
    plt.show()
    plt.plot(ang13)
    plt.plot(ang14)
    plt.plot(ang15)
    plt.show()
    plt.plot(ang16)
    plt.plot(ang17)
    plt.plot(ang18)
    plt.show()

    # x = []
    # y = []
    # z = []
    # for i in range(18):
    #     x.append(Lpoints) 
    # fig = plt.figure()
    # ax = plt.axes(projection="3d")
    # plt.title("traj x vs traj z")
    # ax.plot3D(Lpoints[:,0], Lpoints[:,1], Lpoints[:,2])
    # ax.plot3D(Lpoints[:,3], Lpoints[:,4], Lpoints[:,5])
    # ax.plot3D(Lpoints[:,6], Lpoints[:,7], Lpoints[:,8])
    # ax.plot3D(Lpoints[:,9], Lpoints[:,10], Lpoints[:,11])
    # ax.plot3D(Lpoints[:,12], Lpoints[:,13], Lpoints[:,13])
    # ax.plot3D(Lpoints[:,15], Lpoints[:,16], Lpoints[:,17])
    # plt.show()

    # plt.plot(Lpoints[:,2])
    # plt.plot(Lpoints[:,5])
    # plt.plot(Lpoints[:,8])
    # plt.plot(Lpoints[:,11])
    # plt.plot(Lpoints[:,14])
    # plt.plot(Lpoints[:,17])
    plt.show()
