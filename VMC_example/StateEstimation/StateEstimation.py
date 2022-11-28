from WebotsDevices.WebotsDevices import WebotsDevices
from Kinematic.Kinematic import Kinematic
import numpy as np
import math
import copy


class StateEstimation:
    def __init__(self, devices: WebotsDevices, kin: Kinematic, time_step: int, init_theta: np):

        self.Devices = devices
        self.timeStep = time_step
        self.kine = kin

        self.is_touching = [0, 0, 0, 0]                                         # Touching state
        self.euler = np.array([0.0, 0.0, 0.0])                                  # Roll Pitch Yaw
        self.pre_euler = np.array([0.0, 0.0, 0.0])
        self.dot_euler = np.array([0.0, 0.0, 0.0])
        self.pre_dot_euler = np.array([0.0, 0.0, 0.0])

        self.pre_global_vel = np.array([0.0, 0.0, 0.0])
        self.pre_global_pos = np.array([0.0, 0.0, 0.0])
        self.com_local_vel = np.array([0.0, 0.0, 0.0])                          # Velocity Represent in the IMU frame
        self.com_global_pos = np.array([0.0] * 3)

        self.joint_theta = np.zeros((4, 3), dtype=float)
        self.foot_pos = np.zeros((4, 3), dtype=float)

        for leg in range(4):
            self.foot_pos[leg] = np.array(self.kine.forward(init_theta))         # Initial foot position

        self.pre_foot_pos = np.zeros((4, 3), dtype=float)
        self.dot_foot = np.zeros((4, 3), dtype=float)
        self.pre_dot_foot = np.zeros((4, 3), dtype=float)

    def update_euler(self):
        self.euler = np.array(self.Devices.IMU.getRollPitchYaw())
        self.dot_euler = (self.euler - self.pre_euler) / (0.001 * self.timeStep)  # Differential of euler angle
        # 1-order low pass filter, not suggest to change(0.3 0.7)
        self.dot_euler = 0.3 * self.dot_euler + 0.7 * self.pre_dot_euler

        # 防止虚拟力突变
        if -0.05 < self.euler[2] + math.pi < 0.05 or -0.05 < self.euler[2] - math.pi < 0.05:
            self.dot_euler[2] = self.pre_dot_euler[2]
        self.pre_dot_euler = copy.copy(self.dot_euler)
        self.pre_euler = copy.copy(self.euler)

    def com_velocity_est(self):
        # roll pitch yaw
        s_r = math.sin(self.euler[0])
        c_r = math.cos(self.euler[0])
        s_p = math.sin(self.euler[1])
        c_p = math.cos(self.euler[1])
        s_y = math.sin(self.euler[2])
        c_y = math.cos(self.euler[2])

        self.com_global_pos = np.array(self.Devices.GPS.getValues())
        # 1-order low pass filter, need small alpha to smooth the velocity
        com_global_vel = 0.4 * (self.com_global_pos - self.pre_global_pos) / (0.001 * self.timeStep) + 0.6 * self.pre_global_vel
        self.pre_global_vel = copy.deepcopy(com_global_vel)
        self.pre_global_pos = copy.deepcopy(self.com_global_pos)

        rotate = np.zeros((3, 3), dtype=float)
        rotate[0, 0] = c_y * c_p
        rotate[0, 1] = s_r * s_y - c_y * c_r * s_p
        rotate[0, 2] = c_y * s_r * s_p + s_y * c_r
        rotate[1, 0] = s_p
        rotate[1, 1] = c_p * c_r
        rotate[1, 2] = -c_p * s_r
        rotate[2, 0] = -s_y * c_p
        rotate[2, 1] = c_r * s_y * s_p + s_r * c_y
        rotate[2, 2] = -s_y * s_r * s_p + c_r * c_y
        self.com_local_vel = rotate.T @ com_global_vel

    # Foot velocity Update
    def foot_velocity_est(self):
        self.dot_foot = 0.9 * (self.foot_pos - self.pre_foot_pos) / (0.001 * self.timeStep) + 0.1 * self.pre_dot_foot
        self.pre_foot_pos = copy.deepcopy(self.foot_pos)
        self.pre_dot_foot = copy.deepcopy(self.dot_foot)

    def update_robot_state(self):
        # (1) Body State Estimation
        self.update_euler()
        self.com_velocity_est()

        # (2) Foot Velocity Estimation and Position Update
        self.joint_theta = np.array(self.Devices.get_theta())
        for leg in range(4):
            self.foot_pos[leg] = np.array(self.kine.forward(self.joint_theta[leg]))
        self.foot_velocity_est()

        # (3) Touching State
        self.is_touching = self.Devices.foot_touch_state()
