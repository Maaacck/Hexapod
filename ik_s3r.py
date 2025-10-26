# from cmath import acos
import math as m
import numpy as np


# class in_kin():
#     def __init__(self, link_lengths_ = [0.055,0.07,0.043]):
#         self.l1, self.l2, self.l3 = link_lengths_

    # def ik(self,p,q,r):
    #     x, y, z = p, q, r
    #     theta_1 = m.atan2(y, x)

    #     r = m.sqrt((x)**2  + (y)**2 + (z)**2)
    #     theta_2 = m.acos((self.l2**2 + r**2 - self.l3**2)/(2*self.l2*r))+m.atan(x/z)
    #     theta_3 = -(m.pi - m.acos((self.l3**2 +  self.l2**2 - r**2)/(2*self.l3* self.l2)))

    #     # if(key==0):
    #     #     theta_2 = -m.acos((self.l2**2 + r**2 - self.l3**2)/(2*self.l2*r))+m.atan(x/z)
    #     #     theta_3 = (m.pi - m.acos((self.l3**2 +  self.l2**2 - r**2)/(2*self.l3* self.l2)))
    #     return theta_1,theta_2,theta_3

class in_kin:
    def __init__(self, link_lengths_=[0.0508, 0.104, 0.185]):
        self.l1, self.l2, self.l3 = link_lengths_

    def ik(self, p, q, r):
        x, y, z = p, q, r

        theta_1 = m.atan2(y, x)

        q1 = theta_1

        T01_inv = np.array(
            [
                [m.cos(q1), m.sin(q1), 0, -self.l1],
                [0, 0, 1, 0],
                [m.sin(q1), -m.cos(q1), 0, 0],
                [0, 0, 0, 1],
            ]
        )
        P = np.dot(T01_inv, np.array([[x], [y], [z], [1]]))

        r = m.sqrt(P[0] ** 2 + P[1] ** 2)

        theta_2 = m.acos(
            (self.l2**2 + r**2 - self.l3**2) / (2 * self.l2 * r)
        ) + m.atan2(P[1], P[0])
        theta_3 = -(
            m.pi
            - m.acos((self.l3**2 + self.l2**2 - r**2) / (2 * self.l3 * self.l2))
        )

        a2 = P[0] ** 2 + P[1] ** 2
        a = m.sqrt(a2)

        D = (a2 - self.l2**2 - self.l3**2) / (2 * self.l3 * self.l2)
        # print(D)
        theta_3 = m.atan2(-m.sqrt(1 - D**2), D)
        theta_2 = m.atan2(P[1], P[0]) - m.atan2(
            self.l3 * m.sin(theta_3), self.l2 + self.l3 * m.cos(theta_3)
        )

        # if key==0:
        #     theta_3 = m.atan2(-m.sqrt(1-D**2),D)
        #     theta_2 = -m.atan2(P[1],P[0]) - m.atan2(self.l3*m.sin(theta_3),self.l2+self.l3*m.cos(theta_3))

        return theta_1, theta_2, theta_3


if __name__ == "__main__":
    leg = in_kin()
    print(leg.ik(0.122, 0, -0.15))
