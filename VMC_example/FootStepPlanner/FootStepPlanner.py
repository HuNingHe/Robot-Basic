from UserParameters.UserParameters import UserParameters
from StateEstimation.StateEstimation import StateEstimation
import numpy as np
from cvxopt import matrix, solvers, spdiag
import math
import time as ttm


def mIn3(a1: float, a2: float, a3: float, is_max: int):
    if is_max == 0:
        if a1 >= a3 and a1 >= a2:
            return a1
        elif a2 >= a1 and a2 >= a3:
            return a2
        else:
            return a3
    else:
        if a1 <= a3 and a1 <= a2:
            return a1
        elif a2 <= a1 and a2 <= a3:
            return a2
        else:
            return a3


class StepPlanner:
    def __init__(self, user_parameters: UserParameters, state: StateEstimation):
        self.userParameters = user_parameters
        self.state = state
        self.Kv = [0.1, 0.1]  # the parameter for controlling desire foothold

        #  parameters for rhythm control
        self.controlWeights = [1.0, 5.0, 1000.0]
        self.boundT = [0.15, 0.35]  # boundary of swing period : min and max
        self.boundStep = [[-0.4, 0.4],
                          [-0.3, 0.3]]

    def raibert(self, time: float, init_pos: np, index: int):
        foothold_des = [0, 0]

        foothold_des[0] = 0.5 * self.state.com_local_vel[0] * self.userParameters.Ts + self.Kv[0] * \
                          (self.state.com_local_vel[0] - self.userParameters.desire_vel[0]) + \
                          self.userParameters.desire_height[index] * math.tan(self.state.euler[1])  # x

        foothold_des[1] = -0.5 * self.state.com_local_vel[2] * self.userParameters.Ts + self.Kv[1] * \
                          (-self.state.com_local_vel[2] - self.userParameters.desire_vel[1])
        # + self.desire_height[index] * math.tan(self.euler[0])  # y

        c_fi = math.cos(2 * math.pi * time / self.userParameters.Tw)
        s_fi = math.sin(2 * math.pi * time / self.userParameters.Tw)

        ft = np.zeros(3, dtype=float)
        dft = np.zeros(3, dtype=float)

        if time <= self.userParameters.Tw:
            ft[2] = self.userParameters.stepHeight * 0.5 * (1 - c_fi) + init_pos[2]
            ft[1] = (foothold_des[1] - init_pos[1]) * (time / self.userParameters.Tw - s_fi / (2 * math.pi)) + init_pos[1]
            ft[0] = (foothold_des[0] - init_pos[0]) * (time / self.userParameters.Tw - s_fi / (2 * math.pi)) + init_pos[0]

            dft[2] = (math.pi * self.userParameters.stepHeight / self.userParameters.Tw) * s_fi
            dft[1] = (foothold_des[1] - init_pos[1]) * (1 - c_fi) / self.userParameters.Tw
            dft[0] = (foothold_des[0] - init_pos[0]) * (1 - c_fi) / self.userParameters.Tw

        return [ft, dft]

    def rhythmControl(self, time: float, init_pos: np, index: int):
        # first stage
        # t_start = ttm.time()
        abs_des_vel = [math.fabs(self.userParameters.desire_vel[0]), math.fabs(self.userParameters.desire_vel[1])]
        w = math.sqrt(9.81 / math.fabs(self.userParameters.desire_height[index]))  # 这里到时候可以切换为腿的实际高度试试
        norm_bound = [0.0, 0.0]
        t_norm = 0.0

        if abs_des_vel[0] == 0 and abs_des_vel[1] != 0:
            norm_bound[0] = self.boundStep[1][0] / abs_des_vel[1] if self.boundStep[1][0] / abs_des_vel[1] > self.boundT[0] else self.boundT[0]
            norm_bound[1] = self.boundStep[1][1] / abs_des_vel[1] if self.boundStep[1][1] / abs_des_vel[1] < self.boundT[1] else self.boundT[1]
        elif abs_des_vel[0] != 0 and abs_des_vel[1] == 0:
            norm_bound[0] = self.boundStep[0][0] / abs_des_vel[0] if self.boundStep[0][0] / abs_des_vel[0] > self.boundT[0] else self.boundT[0]
            norm_bound[1] = self.boundStep[0][1] / abs_des_vel[0] if self.boundStep[0][1] / abs_des_vel[0] < self.boundT[1] else self.boundT[1]
        elif abs_des_vel[0] == 0 and abs_des_vel[1] == 0:
            norm_bound[0] = self.boundT[0]
            norm_bound[1] = self.boundT[1]
        else:
            for i in range(2):
                norm_bound[i] = mIn3(self.boundStep[0][i] / abs_des_vel[0], self.boundStep[1][i] / abs_des_vel[1], self.boundT[i], i)
        t_norm = 0.5 * norm_bound[0] + 0.5 * norm_bound[1]

        step_norm = [- self.userParameters.desire_vel[0] * t_norm, self.userParameters.desire_vel[1] * t_norm]  # L and W
        b_norm = [step_norm[0] / (math.exp(w * t_norm) - 1), step_norm[1] / (math.exp(w * t_norm) - 1)]
        b_bound = [[-self.boundStep[0][1] / (math.exp(w * self.boundT[0]) - 1), self.boundStep[0][1] / (math.exp(w * self.boundT[0]) - 1)],
                   [-self.boundStep[1][1] / (math.exp(w * self.boundT[0]) - 1), self.boundStep[1][1] / (math.exp(w * self.boundT[0]) - 1)]]

        # second stage
        est_dcm = np.array([self.state.com_local_vel[0] / w, -self.state.com_local_vel[2] / w])

        p = [0.5 * self.controlWeights[0], 0.5 * self.controlWeights[0], 0.5 * self.controlWeights[1],
             0.5 * self.controlWeights[2], 0.5 * self.controlWeights[2]]
        P = spdiag(p)

        q = matrix([-self.controlWeights[0] * step_norm[0], -self.controlWeights[0] * step_norm[1],
                    -self.controlWeights[1] * math.exp(w * t_norm), -self.controlWeights[2] * b_norm[0],
                    -self.controlWeights[2] * b_norm[1]])

        g1 = np.diag([1.0, 1.0, 1.0, 1.0, 1.0])
        g2 = -g1
        g = np.vstack((g1, g2))
        G = matrix(g)

        h = matrix([self.boundStep[0][1], self.boundStep[1][1], math.exp(w * self.boundT[1]), b_bound[0][1], b_bound[1][1],
                    -self.boundStep[0][0], -self.boundStep[1][0], -math.exp(w * self.boundT[0]), -b_bound[0][0], -b_bound[1][0]])

        a = np.array([[1.0, 0.0, -(est_dcm[0] - init_pos[0]) * math.exp(-w * time), 1.0, 0.0],
                      [0.0, 1.0, -(est_dcm[1] - init_pos[1]) * math.exp(-w * time), 0.0, 1.0]])
        A = matrix(a)
        b = matrix([0.0, 0.0])

        solvers.options['show_progress'] = False
        sol = solvers.qp(P, q, G, h, A, b)
        # print(est_dcm[1])
        # t_end = ttm.time()
        # print(t_end - t_start)
        if sol['x'][2] < 5.0:
            print(sol['x'][2])
        # print(sol['primal objective'])


if __name__ == '__main__':
    # P = matrix(np.diag([1, 0]), tc='d')
    # q = matrix(np.array([3, 4]), tc='d')
    # G = matrix(np.array([[-1, 0], [0, -1], [-1, -3], [2, 5], [3, 4]]), tc='d')
    # h = matrix(np.array([0, 0, -15, 100, 80]), tc='d')
    # sol = solvers.qp(P, q, G, h)
    # sol = solvers.qp(P,q,G,h,A,b)
    # print(sol['x'])
    # print(sol['primal objective'])
    b = np.array([[1, 1],
                  [1, 2]])
    lb = matrix(b)
    print(lb)
