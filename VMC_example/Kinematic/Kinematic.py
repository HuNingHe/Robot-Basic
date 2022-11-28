"""
################# Kinematic functions ##################
joint frames is similar to miniCheetah
"""
import math
from typing import List, Sequence


class Kinematic:

    def __init__(self, link_length: List):
        self._LinkLength = link_length  # Length of Hip Thigh Leg

    # 正向运动学
    def forward(self, theta: Sequence):
        s1 = math.sin(theta[0])
        c1 = math.cos(theta[0])
        c2 = math.cos(theta[1])
        s2 = math.sin(theta[1])
        c23 = math.cos(theta[1] + theta[2])
        s23 = math.sin(theta[1] + theta[2])
        foot_position = [0, 0, 0]

        foot_position[0] = -self._LinkLength[2] * s23 - self._LinkLength[1] * s2
        foot_position[1] = self._LinkLength[2] * c23 * s1 + self._LinkLength[1] * c2 * s1 + self._LinkLength[0] * s1
        foot_position[2] = -self._LinkLength[2] * c23 * c1 - self._LinkLength[1] * c2 * c1 - self._LinkLength[0] * c1
        return foot_position

    def Jacobi(self, theta: Sequence):
        s1 = math.sin(theta[0])
        c1 = math.cos(theta[0])
        c2 = math.cos(theta[1])
        s2 = math.sin(theta[1])
        c23 = math.cos(theta[1] + theta[2])
        s23 = math.sin(theta[1] + theta[2])

        J = []
        for _ in range(3):
            J.append([])

        J[0].append(0)
        J[0].append(-self._LinkLength[2] * c23 - self._LinkLength[1] * c2)
        J[0].append(-self._LinkLength[2] * c23)

        J[1].append(self._LinkLength[2] * c23 * c1 + self._LinkLength[1] * c2 * c1 + self._LinkLength[0] * c1)
        J[1].append(-self._LinkLength[2] * s23 * s1 - self._LinkLength[1] * s2 * s1)
        J[1].append(-self._LinkLength[2] * s23 * s1)

        J[2].append(self._LinkLength[2] * c23 * s1 + self._LinkLength[1] * c2 * s1 + self._LinkLength[0] * s1)
        J[2].append(self._LinkLength[2] * s23 * c1 + self._LinkLength[1] * s2 * c1)
        J[2].append(self._LinkLength[2] * s23 * c1)

        return J


if __name__ == '__main__':
    kine = Kinematic([0, 0.21, 0.21])
    theta1 = [0, 0, -1.1]
    print(kine.forward(theta1))
