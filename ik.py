import math as m

class kinematics:
    def __init__(self, link_lengths = [0.05, 0.15, 0.15]):
        self.l1 = link_lengths[0]
        self.l2 = link_lengths[1]
        self.l3 = link_lengths[2]

    def in_kin(self, x, y, z, leg_id):
        theta_1 = m.atan2(y,x)
        
        dist = m.sqrt(x**2 + y**2 + z**2)

        theta_2 = m.acos((-self.l3**2 + self.l2**2 + dist**2)/(2*self.l2*dist)) + m.atan2(z,m.sqrt(x**2+y**2))

        theta_3 = -m.acos((dist**2-self.l2**2-self.l3**2)/(2*self.l2*self.l3))

        return theta_1, theta_2, theta_3
