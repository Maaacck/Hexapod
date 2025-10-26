import math as m
import numpy as np

class in_kin:#inverse kinematics
    def __init__(self, link_lengths = [0.05, 0.15, 0.15]):
        self.l1, self.l2, self.l3 = link_lengths

    def ik(self, p, q, r):#Changes cartesian cords for polar
        x, y, z = p,q,r

        theta_1 = m.atan2(y,x)

        r = m.sqrt(x**2 + y**2 + z**2)

        theta_2 = m.acos((-self.l3**2 + self.l2**2 + r**2)/(2*self.l2*r)) + m.atan2(z,m.sqrt(x**2+y**2))
        theta_3 = -m.acos((r**2-self.l2**2-self.l3**2)/(2*self.l2*self.l3))

        return theta_1, theta_2, theta_3 #Class returns the angle necessary for position
    