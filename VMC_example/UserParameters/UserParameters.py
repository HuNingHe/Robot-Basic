import numpy as np
from WebotsDevices.WebotsDevices import WebotsDevices


class UserParameters:
    def __init__(self, devices: WebotsDevices, time_step: int):
        # you can add your parameters here
        self.Devices = devices
        self.timeStep = time_step
        self.Mg = 8.6 * 9.81  # gravity of body
        # self.LinkLength = np.array([0, 0.4, 0.4])       # length of hip thigh knee
        self.Tw = 0.2  # swing duration
        self.Ts = 0.2  # stance duration
        self.stop = 0
        self.K_extend = 1.2  # the parameter of extending trajectory
        self.Kst = 0.5  # the parameter of stance duration adjustment

        self.pelvis = 0.0

        self.desire_height = np.array([-0.32, -0.32, -0.32, -0.32])  # LF RF RB LB
        self.desire_vel = np.array([0, 0, 0])
        self.stepHeight = 0.06  # the parameter of gait's height, which depends on your Hip and foot coordinate 0.12

        self.yaw_des = np.array([0, 0], dtype=float)  # Desire Yaw angle and desire Yaw velocity
        self.roll_des = np.array([0, 0], dtype=float)  # Desire Roll angle and desire Roll velocity
        self.pitch_des = np.array([0, 0], dtype=float)  # Desire Pitch angle and desire Pitch angular velocity

    # 'U' and 'L' control the robot height
    # DOWN and UP control the Pitch
    # LEFT and RIGHT control the Roll
    # 'W' 'S' 'A' 'D' 'Q' 'E' control the moving direction
    # "P" used to stop
    def key_control(self):
        key = self.Devices.KeyBoard.getKey()
        if key == self.Devices.KeyBoard.UP:
            self.pitch_des[0] += 0.002 * self.timeStep

        elif key == self.Devices.KeyBoard.DOWN:
            self.pitch_des[0] -= 0.002 * self.timeStep

        elif key == self.Devices.KeyBoard.LEFT:
            self.roll_des[0] -= 0.002 * self.timeStep

        elif key == self.Devices.KeyBoard.RIGHT:
            self.roll_des[0] += 0.002 * self.timeStep

        elif key == ord('P'):
            self.stop = 1
            self.pitch_des[0] = 0
            self.roll_des[0] = 0

        elif key == ord('Q'):
            self.stop = 0
            self.desire_vel = np.array([-0.3, 0.2, 0])

        elif key == ord('E'):
            self.stop = 0
            self.desire_vel = np.array([-0.3, -0.2, 0])

        elif key == ord('W'):
            self.stop = 0
            self.desire_vel = np.array([-0.3, 0, 0])

        elif key == ord('A'):
            # self.desire_vel = np.array([-0.1, 0, 0])
            self.yaw_des[1] = 0.8
            self.stop = 0

        elif key == ord('D'):
            # self.desire_vel = np.array([-0.1, 0, 0])
            self.yaw_des[1] = -0.8
            self.stop = 0

        elif key == ord('S'):
            self.stop = 0
            self.desire_vel = np.array([0.3, 0, 0])

        elif key == ord('U'):
            self.desire_height -= 0.0005 * self.timeStep

        elif key == ord('L'):
            self.desire_height += 0.0005 * self.timeStep

        else:
            self.desire_vel = np.array([0.0, 0.0, 0.0])
            self.yaw_des[1] = 0
