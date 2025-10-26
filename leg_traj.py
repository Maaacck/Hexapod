#! usr/bin/env python3
from math import *
from matplotlib import pyplot as plt
import sys
import os
sys.path.append('/hexapod/hexapod_core')
# sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../../')
# from s3r_kin import kinematics
from online_trajectory import trajectory_generator
from ik_s3r import in_kin
# from mpl_toolkits import mplot3d


class leg_traj :

    def __init__(self, vx, vy, vz):

        self.vx, self.vy, self.vz = vx, vy, vz
        self.f_hard = 100
        self.f_gate= 2
        self.leg_height = 0.1

        self.leg_traj = trajectory_generator(vx=self.vx, vy=self.vy, vz=self.vz, leg_height=self.leg_height,
                                        f_hard=self.f_hard, phase_angle=0, leg_id = 1)
        
    
    def get_loop_points(self, vx, vy):
        self.vx, self.vy = vx, vy
        loop_points = []
        Lpoints = []
        
        self.leg_traj.change_vx_vy(vx, vy)

        for i in range(self.f_hard):
            points = []
            p = []
            
            points.extend(self.leg_traj.anglist())
            p.extend(self.leg_traj.pointlist())
                # else:
                #     key=0
                #     points.extend(self.leg_traj[j].anglist(key))
            loop_points.append(points)
            Lpoints.append(p)
        return loop_points, Lpoints  
        

if __name__=="__main__":
    
    # test above code
    quad = leg_traj( 0.1, 0, 0.05)
    loop_points, Lpoints = quad.get_loop_points(0.1, 0)

    # use matplotlib to plot the trajectory
    import numpy as np
    import matplotlib.pyplot as plt
   
    plt.plot( loop_points)
    plt.show()
    # plt.plot( Lpoints)
    # plt.show()