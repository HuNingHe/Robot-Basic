from StateEstimation.StateEstimation import StateEstimation
from WebotsDevices.WebotsDevices import WebotsDevices
from UserParameters.UserParameters import UserParameters
from FootStepPlanner.FootStepPlanner import StepPlanner
from Kinematic import Kinematic
import numpy as np
import math


class VMC:
    def __init__(self, devices: WebotsDevices, user_parameter: UserParameters, state: StateEstimation, kine: Kinematic, sw_traj: StepPlanner,
                 time_step: int):

        # (1) Parameters for desire locomotion
        self.devices = devices
        self.timeStep = time_step
        self.userParameter = user_parameter
        self.state = state
        self.kine = kine
        self.swTraj = sw_traj
        self.time = 0  # gait time

        # (2) Parameters for virtual forces
        self.K_st = np.array([[0, 0, 0],  # K specify the spring factor, B specify the damper factor
                              [0, 0, 0],  # you should only change the diagonal elements
                              [0, 0, 3800]],  # 0, 0, 3800
                             dtype=float)

        self.B_st = np.array([[100, 0, 0],  # 140
                              [0, 200, 0],  # 140
                              [0, 0, 220]],  # 200
                             dtype=float)

        self.K_sw = np.array([[100, 0, 0],
                              [0, 100, 0],
                              [0, 0, 200]],
                             dtype=float)

        self.B_sw = np.array([[50, 0, 0],  # 100
                              [0, 50, 0],  # 100
                              [0, 0, 100]],  # 120
                             dtype=float)

        # if you want to set yaw to a fixed value, you can tune self.KB_yaw[0] and self.yaw_des[0]
        # in order to turn, we don't set the yaw to a fixed value
        self.KB_yaw = np.array([0, 200])  # virtual spring and damper factor for controlling yaw when move 120 20
        self.KB_roll_mov = np.array([1200, 120])  # 3000 200

        # if you want to set pitch to a fixed value, you can set self.KB_pitch_mov=np.array([0.1, 0.05])
        # in order to up and down slope, we don't set the pitch to a fixed value
        self.KB_pitch_mov = np.array([0, 0])  # virtual spring and damper factor for controlling pitch when move

        self.KB_roll = np.array([0.5, 0.1])  # virtual spring and damper factor for controlling roll
        self.KB_pitch = np.array([1.2, 0.3])  # virtual spring and damper factor for controlling pitch

        # (3) Parameters for phase swapping
        self.standFlag = 2  # 0: LF-RH stand;     1: RF-LH stand;    2: fly;    3: all legs stand
        self.fly_time = 0  # record the flying time, its convenient for extending this algorithm

        # (4) init foot position and footpath
        self.init_pos = np.zeros((4, 3), dtype=float)
        self.path = np.zeros((2, 3), dtype=float)
        self.path_dot = np.zeros((2, 3), dtype=float)

    def swap_phase_trot(self):
        # in case that when the robot get the start command, it wouldn't swap phase, because all legs stand when start
        if self.time > self.userParameter.Tw / 6:
            if self.standFlag == 0 and all(self.state.is_touching == np.array([1, 1, 1, 1])):
                self.standFlag = 1
                self.time = 0
                # print("L-->R")

            elif self.standFlag == 1 and all(self.state.is_touching == np.array([1, 1, 1, 1])):
                self.standFlag = 0
                self.time = 0
                # print("R-->L")

            elif all(self.state.is_touching == np.array([1, 1, 1, 1])) and self.standFlag == 2:
                self.standFlag = 3
                self.userParameter.stop = 1
                self.time = 0
                # print("fly-->all legs stand")

            elif self.standFlag == 3 and all(self.state.is_touching == np.array([1, 1, 1, 1])) and self.userParameter.stop == 0:
                self.standFlag = 1
                self.time = 0
                # print("all legs stand-->move forward")

        if all(self.state.is_touching == np.array([1, 1, 1, 1])) and self.userParameter.stop == 1:
            self.standFlag = 3
            self.time = 0
            # print("any state-->all legs stand")

        if all(self.state.is_touching == np.array([0, 0, 0, 0])):
            self.fly_time += 0.001 * self.timeStep
            if self.fly_time > 0.05:
                self.standFlag = 2
                self.userParameter.stop = 1
                self.fly_time = 0
                self.time = 0
                # print("any state--> fly")

    def vmc_trot(self):

        # if you want to develop your own gait like(pace/gallop), you should change almost all the method:  run()
        self.userParameter.key_control()
        self.state.update_robot_state()
        self.swap_phase_trot()
        self.time += self.timeStep * 0.001
        # stance duration adjustment
        self.userParameter.Ts *= math.cos(self.userParameter.Kst * self.state.euler[1])

        #        ------------------------------
        #      /                             /
        #     /                             /
        #     -----------------------------
        #       |  |                  |  |
        #       |  |                  |  |
        #       |  | "1:RF"           |  | "2:RB"
        #       | "0:LF"              | "3:LB"

        # ------------ diagonal legs i.e trot gait control ---------------
        # -------------------- diagonal stand legs -----------------------
        st = [0, 0, 0]  # the index of stand leg. when the third value is not zero, all legs stand
        sw = [0, 0, 0]  # the index of swing leg. when the third value is not zero, all legs swing

        if self.standFlag == 0:
            sw[0] = 1
            sw[1] = 3

            st[0] = 0
            st[1] = 2
        elif self.standFlag == 1:
            sw[0] = 0
            sw[1] = 2

            st[0] = 1
            st[1] = 3
        elif self.standFlag == 2:
            sw[2] = 100
        else:
            st[2] = 300
        # trot gait control
        if sw[2] == 0 and st[2] == 0:
            # -------------------- diagonal stand legs -----------------------
            pitch_height = self.KB_pitch_mov.dot(self.userParameter.pitch_des - np.array([self.state.euler[1], self.state.dot_euler[1]]))
            # front side legs and hind side legs need different compensations in height to balance pitch disturbance
            delta_height_pitch = [-pitch_height, pitch_height]

            err_height = np.zeros((2, 3), dtype=float)  # errors for legs in pitch
            err_vel = np.zeros((2, 3), dtype=float)

            err_roll = self.userParameter.roll_des - np.array([self.state.euler[0], self.state.dot_euler[0]])
            torque_roll = np.array([self.KB_roll_mov.dot(err_roll), 0, 0])

            err_yaw = self.userParameter.yaw_des - np.array([self.state.euler[2], self.state.dot_euler[2]])
            f_yaw = np.zeros(2, dtype=float)
            if st[0] == 0:
                f_yaw[0] = -self.KB_yaw.dot(err_yaw)
                f_yaw[1] = self.KB_yaw.dot(err_yaw)
            else:
                f_yaw[0] = self.KB_yaw.dot(err_yaw)
                f_yaw[1] = -self.KB_yaw.dot(err_yaw)

            st_force = np.zeros((2, 3), dtype=float)  # stand leg virtual forces
            st_torque = np.zeros((2, 3), dtype=float)  # stand leg torques

            for i in range(2):
                self.userParameter.desire_height[st[i]] += delta_height_pitch[i]
                err_height[i] = np.array([0, 0, self.userParameter.desire_height[st[i]]]) - np.array([0, 0, self.state.foot_pos[st[i]][2]])
                err_vel[i] = self.userParameter.desire_vel - self.state.dot_foot[st[i]]
                # print(self.state.dot_foot[st[1]])
                # 为了确保机器人运行时更加稳定，需要保证横滚角稳定为0，这不利于横向坡面运动，因此
                # 横向坡面运动时，将下面的np.array([])部分替换成# np.array([0.6 * math.sin(self.euler[1]) * self.Mg + left_f_yaw, 0.6 * math.sin(self.euler[0])*self.Mg, 0])
                # else下面的替换成np.array([0.6 * math.sin(self.euler[1]) * self.Mg + right_f_yaw, 0.6 * math.sin(self.euler[0])*self.Mg, 0])
                st_force[i] = self.K_st @ err_height[i] + self.B_st @ err_vel[i] - np.array(
                    [0.6 * math.sin(self.state.euler[1]) * self.userParameter.Mg + f_yaw[i], 0,
                     self.userParameter.Mg / 2])  # 0.6 * math.sin(self.state.euler[1]) * self.userParameter.Mg +

                st_torque[i] = np.array(self.kine.Jacobi(self.state.joint_theta[st[i]])).T @ st_force[i] - torque_roll
                # mmmm = np.linalg.pinv(np.array(self.kine.Jacobi(self.state.joint_theta[st[i]])).T) @ np.array([15, 15, 20])
                # print(mmmm)
                self.devices.set_motor_torque(st[i], st_torque[i])
            # -------------------- diagonal swing legs -----------------------
            # you can extend the swing trajectory when the leg is not touching the floor
            # you can also stop the swing when the swinging leg touch the floor before the instant moment Tw

            err_path = np.zeros((2, 3), dtype=float)
            err_dot_path = np.zeros((2, 3), dtype=float)
            sw_force = np.zeros((2, 3), dtype=float)
            sw_torque = np.zeros((2, 3), dtype=float)

            for j in range(2):
                if self.time == 0.001 * self.timeStep:
                    self.init_pos[j] = self.state.foot_pos[sw[j]]
                    self.init_pos[j + 2] = self.state.foot_pos[st[j]]
                # 提前触地
                if self.state.is_touching[sw[j]] == 1 and self.userParameter.Tw / 6 < self.time : # < self.userParameter.Tw:
                    self.path[j], self.path_dot[j] = self.state.foot_pos[sw[j]], np.array([0, 0, 0])
                    # self.userParameter.desire_height[sw[j]] = self.state.foot_pos[sw[j]][2]  # 适应地形,效果不行

                elif self.state.is_touching[sw[j]] == 0 and self.time > self.userParameter.Tw and self.state.foot_pos[sw[j]][2] > -0.38:
                    self.path[j] = np.array([self.state.foot_pos[sw[j]][0], self.state.foot_pos[sw[j]][1],
                                             self.state.foot_pos[sw[j]][2] - self.userParameter.K_extend * (self.time - self.userParameter.Tw)])
                    self.path_dot[j] = np.array([self.state.dot_foot[sw[j]][0], self.state.dot_foot[sw[j]][1], - self.userParameter.K_extend])
                    print("extend leg: {0}".format(sw[j]))

                else:
                    self.path[j], self.path_dot[j] = self.swTraj.raibert(self.time, self.init_pos[j], sw[j])
                    self.swTraj.rhythmControl(self.time, 0.5 * (self.init_pos[2] + self.init_pos[3]), sw[j])

                    if self.time > self.userParameter.Tw:  # and self.is_touching[sw[j]] == 1:
                        self.path[j], self.path_dot[j] = self.state.foot_pos[sw[j]], np.array([0, 0, 0])
                        print("swap-phase may meet some errors!")

                err_path[j] = self.path[j] - self.state.foot_pos[sw[j]]
                err_dot_path[j] = self.path_dot[j] - self.state.dot_foot[sw[j]]

                sw_force[j] = self.K_sw @ err_path[j] + self.B_sw @ err_dot_path[j]
                sw_torque[j] = np.array(self.kine.Jacobi(self.state.joint_theta[sw[j]])).T @ sw_force[j]

                self.devices.set_motor_torque(sw[j], sw_torque[j])

        # When fly and all legs stand, we only need to balance the robot's pitch and roll disturbance
        # When all legs stand
        elif st[2] == 300:
            pitch_height = self.KB_pitch.dot(self.userParameter.pitch_des - np.array([self.state.euler[1], self.state.dot_euler[1]]))
            torque_roll = self.KB_roll.dot(self.userParameter.roll_des - np.array([self.state.euler[0], self.state.dot_euler[0]]))

            # left side legs and right side legs need different compensations in height
            delta_height_roll = [-torque_roll, torque_roll, torque_roll, -torque_roll]
            # front side legs and hind side legs need different compensations in height
            delta_height_pitch = [-pitch_height, -pitch_height, pitch_height, pitch_height]

            pos_des = np.zeros((4, 3), dtype=float)
            vel_des = np.zeros(3, dtype=float)

            err_p = np.zeros((4, 3), dtype=float)
            err_v = np.zeros((4, 3), dtype=float)

            f = np.zeros((4, 3), dtype=float)
            torque = np.zeros((4, 3), dtype=float)

            for j in range(4):
                pos_des[j][2] = self.userParameter.desire_height[j] + delta_height_pitch[j] + delta_height_roll[j]
                err_p[j] = pos_des[j] - self.state.foot_pos[j]

                err_v[j] = vel_des - self.state.dot_foot[j]
                f[j] = self.K_st @ err_p[j] + self.B_st @ err_v[j] - np.array([0, 0, self.userParameter.Mg / 4])

                torque[j] = np.array(self.kine.Jacobi(self.state.joint_theta[j])).T @ f[j]  # + np.array([torque_roll, 0, 0])
                self.devices.set_motor_torque(j, torque[j])

        # when all legs swing or fly state
        else:
            pos_des = np.zeros((4, 3), dtype=float)
            vel_des = np.zeros(3, dtype=float)

            err_p = np.zeros((4, 3), dtype=float)
            err_v = np.zeros((4, 3), dtype=float)

            f = np.zeros((4, 3), dtype=float)
            torque = np.zeros((4, 3), dtype=float)

            for i in range(4):
                pos_des[i][2] = self.userParameter.desire_height[i]
                err_p[i] = pos_des[i] - self.state.foot_pos[i]

                err_v[i] = vel_des - self.state.dot_foot[i]
                f[i] = self.K_sw @ err_p[i] + self.B_sw @ err_v[i] - np.array([0, 0, self.userParameter.Mg / 4])

                torque[i] = np.array(self.kine.Jacobi(self.state.joint_theta[i])).T @ f[i]
                self.devices.set_motor_torque(i, torque[i])

    def swap_phase_es(self):
        # in case that when the robot get the start command, it wouldn't swap phase, because all legs stand when start
        if self.time > self.userParameter.Tw / 6:
            if self.standFlag == 0 and all(self.state.is_touching == np.array([1, 1, 1, 1])):
                self.standFlag = 1
                self.time = 0
                # print("L-->R")

            elif self.standFlag == 1 and all(self.state.is_touching == np.array([1, 1, 1, 1])):
                self.standFlag = 0
                self.time = 0
                # print("R-->L")

            elif all(self.state.is_touching == np.array([1, 1, 1, 1])) and self.standFlag == 2:
                self.standFlag = 3
                self.userParameter.stop = 1
                self.time = 0
                # print("fly-->all legs stand")

            elif self.standFlag == 3 and all(self.state.is_touching == np.array([1, 1, 1, 1])) and self.userParameter.stop == 0:
                self.standFlag = 1
                self.time = 0
                # print("all legs stand-->move forward")

        if all(self.state.is_touching == np.array([1, 1, 1, 1])) and self.userParameter.stop == 1:
            self.standFlag = 3
            self.time = 0
            # print("any state-->all legs stand")

        if all(self.state.is_touching == np.array([0, 0, 0, 0])):
            self.fly_time += 0.001 * self.timeStep
            if self.fly_time > 0.05:
                self.standFlag = 2
                self.userParameter.stop = 1
                self.fly_time = 0
                self.time = 0
                # print("any state--> fly")

    def run_es(self):
        # if you want to develop your own gait like(pace/gallop), you should change almost all the method:  run()
        self.userParameter.key_control()
        self.state.update_robot_state()
        self.swap_phase_vmc()
        self.time += self.timeStep * 0.001
        # stance duration adjustment
        # self.userParameter.Ts *= math.cos(self.userParameter.Kst * self.state.euler[1])

        #        ------------------------------
        #      /                             /
        #     /                             /
        #     -----------------------------
        #       |  |                  |  |
        #       |  |                  |  |
        #       |  | "1:RF"           |  | "2:RB"
        #       | "0:LF"              | "3:LB"

        # ------------ diagonal legs i.e trot gait control ---------------
        # -------------------- diagonal stand legs -----------------------
        st = [0, 0, 0]  # the index of stand leg. when the third value is not zero, all legs stand
        sw = [0, 0, 0]  # the index of swing leg. when the third value is not zero, all legs swing

        if self.standFlag == 0:
            sw[0] = 1
            sw[1] = 3

            st[0] = 0
            st[1] = 2
        elif self.standFlag == 1:
            sw[0] = 0
            sw[1] = 2

            st[0] = 1
            st[1] = 3
        elif self.standFlag == 2:
            sw[2] = 100
        else:
            st[2] = 300
        # trot gait control
        if sw[2] == 0 and st[2] == 0:
            # -------------------- diagonal stand legs -----------------------
            pitch_height = self.KB_pitch_mov.dot(self.userParameter.pitch_des - np.array([self.state.euler[1], self.state.dot_euler[1]]))
            # front side legs and hind side legs need different compensations in height to balance pitch disturbance
            delta_height_pitch = [-pitch_height, pitch_height]

            err_height = np.zeros((2, 3), dtype=float)  # errors for legs in pitch
            err_vel = np.zeros((2, 3), dtype=float)

            err_roll = self.userParameter.roll_des - np.array([self.state.euler[0], self.state.dot_euler[0]])
            torque_roll = np.array([self.KB_roll_mov.dot(err_roll), 0, 0])

            err_yaw = self.userParameter.yaw_des - np.array([self.state.euler[2], self.state.dot_euler[2]])
            left_f_yaw = -self.KB_yaw.dot(err_yaw)
            right_f_yaw = self.KB_yaw.dot(err_yaw)

            st_force = np.zeros((2, 3), dtype=float)  # stand leg virtual forces
            st_torque = np.zeros((2, 3), dtype=float)  # stand leg torques

            for i in range(2):
                self.userParameter.desire_height[st[i]] += delta_height_pitch[i]
                err_height[i] = np.array([0, 0, self.userParameter.desire_height[st[i]]]) - np.array([0, 0, self.state.foot_pos[st[i]][2]])
                err_vel[i] = self.userParameter.desire_vel - self.state.dot_foot[st[i]]
                # 为了确保机器人运行时更加稳定，需要保证横滚角稳定为0，这不利于横向坡面运动，因此
                # 横向坡面运动时，将下面的np.array([])部分替换成# np.array([0.6 * math.sin(self.euler[1]) * self.Mg + left_f_yaw, 0.6 * math.sin(self.euler[0])*self.Mg, 0])
                # else下面的替换成np.array([0.6 * math.sin(self.euler[1]) * self.Mg + right_f_yaw, 0.6 * math.sin(self.euler[0])*self.Mg, 0])
            if st[0] == 0:
                st_force[0] = self.K_st @ err_height[0] + self.B_st @ err_vel[0] + np.array(
                    [0.6 * math.sin(self.state.euler[1]) * self.userParameter.Mg + left_f_yaw, 0, 0])
                st_force[1] = self.K_st @ err_height[1] + self.B_st @ err_vel[1] + np.array(
                    [0.6 * math.sin(self.state.euler[1]) * self.userParameter.Mg + right_f_yaw, 0, 0])

            else:
                st_force[0] = self.K_st @ err_height[0] + self.B_st @ err_vel[0] + np.array(
                    [0.6 * math.sin(self.state.euler[1]) * self.userParameter.Mg + right_f_yaw, 0, 0])
                st_force[1] = self.K_st @ err_height[1] + self.B_st @ err_vel[1] + np.array(
                    [0.6 * math.sin(self.state.euler[1]) * self.userParameter.Mg + left_f_yaw, 0, 0])

            for k in range(2):
                st_torque[k] = -np.array(self.kine.Jacobi(self.state.joint_theta[st[k]])).T @ st_force[k] - torque_roll
                self.devices.set_motor_torque(st[k], st_torque[k])
            # -------------------- diagonal swing legs -----------------------
            # you can extend the swing trajectory when the leg is not touching the floor
            # you can also stop the swing when the swinging leg touch the floor before the instant moment Tw

            err_path = np.zeros((2, 3), dtype=float)
            err_dot_path = np.zeros((2, 3), dtype=float)
            sw_force = np.zeros((2, 3), dtype=float)
            sw_torque = np.zeros((2, 3), dtype=float)

            for j in range(2):
                if self.time == 0.001 * self.timeStep:
                    self.init_pos[j] = self.state.foot_pos[sw[j]]
                    self.init_pos[j + 2] = self.state.foot_pos[st[j]]
                # 提前触地
                if self.state.is_touching[sw[j]] == 1 and self.time > self.userParameter.Tw / 6:
                    self.path[j], self.path_dot[j] = self.state.foot_pos[sw[j]], np.array([0, 0, 0])
                    # self.userParameter.desire_height[sw[j]] = self.state.foot_pos[sw[j]][2]  # 适应地形,效果不行

                elif self.state.is_touching[sw[j]] == 0 and self.time > self.userParameter.Tw and self.state.foot_pos[sw[j]][2] > -0.7:
                    self.path[j] = np.array([self.state.foot_pos[sw[j]][0], self.state.foot_pos[sw[j]][1],
                                             self.state.foot_pos[sw[j]][2] - self.userParameter.K_extend * (self.time - self.userParameter.Tw)])
                    self.path_dot[j] = np.array([self.state.dot_foot[sw[j]][0], self.state.dot_foot[sw[j]][1], - self.userParameter.K_extend])
                    print("extend leg: {0}".format(sw[j]))

                else:  # self.is_touching[sw[j]] == 0 and self.time < self.Tw:
                    self.path[j], self.path_dot[j] = self.swTraj.raibert(self.time, self.init_pos[j], sw[j])
                    # self.swTraj.rhythmControl(self.time, 0.5 * (self.init_pos[2] + self.init_pos[3]), sw[j])

                    if self.time > self.userParameter.Tw:  # and self.is_touching[sw[j]] == 1:
                        self.path[j], self.path_dot[j] = self.state.foot_pos[sw[j]], np.array([0, 0, 0])
                        print("swap-phase may meet some errors!")

                err_path[j] = self.path[j] - self.state.foot_pos[sw[j]]
                err_dot_path[j] = self.path_dot[j] - self.state.dot_foot[sw[j]]

                sw_force[j] = self.K_sw @ err_path[j] + self.B_sw @ err_dot_path[j]
                sw_torque[j] = np.array(self.kine.Jacobi(self.state.joint_theta[sw[j]])).T @ sw_force[j]

                self.devices.set_motor_torque(sw[j], sw_torque[j])

        # When fly and all legs stand, we only need to balance the robot's pitch and roll disturbance
        # When all legs stand
        elif st[2] == 300:
            pitch_height = self.KB_pitch.dot(self.userParameter.pitch_des - np.array([self.state.euler[1], self.state.dot_euler[1]]))
            roll_height = self.KB_roll.dot(self.userParameter.roll_des - np.array([self.state.euler[0], self.state.dot_euler[0]]))

            # left side legs and right side legs need different compensations in height
            delta_height_roll = [-roll_height, roll_height, roll_height, -roll_height]
            # front side legs and hind side legs need different compensations in height
            delta_height_pitch = [-pitch_height, -pitch_height, pitch_height, pitch_height]

            pos_des = np.zeros((4, 3), dtype=float)
            vel_des = np.zeros(3, dtype=float)

            err_p = np.zeros((4, 3), dtype=float)
            err_v = np.zeros((4, 3), dtype=float)

            f = np.zeros((4, 3), dtype=float)
            torque = np.zeros((4, 3), dtype=float)

            for j in range(4):
                pos_des[j][2] = self.userParameter.desire_height[j] + delta_height_roll[j] + delta_height_pitch[j]
                err_p[j] = pos_des[j] - self.state.foot_pos[j]

                err_v[j] = vel_des - self.state.dot_foot[j]
                f[j] = self.K_st @ err_p[j] + self.B_st @ err_v[j]  # - np.array([0, 0, self.userParameter.Mg / 4])

                torque[j] = np.array(self.kine.Jacobi(self.state.joint_theta[j])).T @ f[j]
                self.devices.set_motor_torque(j, torque[j])

        # when all legs swing or fly state
        else:
            pos_des = np.zeros((4, 3), dtype=float)
            vel_des = np.zeros(3, dtype=float)

            err_p = np.zeros((4, 3), dtype=float)
            err_v = np.zeros((4, 3), dtype=float)

            f = np.zeros((4, 3), dtype=float)
            torque = np.zeros((4, 3), dtype=float)

            for i in range(4):
                pos_des[i][2] = self.userParameter.desire_height[i]
                err_p[i] = pos_des[i] - self.state.foot_pos[i]

                err_v[i] = vel_des - self.state.dot_foot[i]
                f[i] = self.K_sw @ err_p[i] + self.B_sw @ err_v[i]  # + np.array([0, 0, self.userParameter.Mg / 4])

                torque[i] = np.array(self.kine.Jacobi(self.state.joint_theta[i])).T @ f[i]
                self.devices.set_motor_torque(i, torque[i])
