from VMC.VMC import VMC
from StateEstimation.StateEstimation import StateEstimation
from Kinematic.Kinematic import Kinematic
from UserParameters.UserParameters import UserParameters
from FootStepPlanner.FootStepPlanner import StepPlanner
from WebotsDevices.WebotsDevices import WebotsDevices
from controller import Robot
import numpy as np

robot_link_length = [0, 0.21, 0.21]  # hip thigh knee lengths
init_joint_theta = np.array([0, 0.55, -1.1])  # ab/ad hip knee joint theta angle

robot = Robot()
timeStep = int(1)
webots_devices = WebotsDevices(robot)
webots_devices.devices_create()
webots_devices.devices_init(timeStep)

user_parameters = UserParameters(webots_devices, timeStep)
kinematic = Kinematic(robot_link_length)
state_estimation = StateEstimation(webots_devices, kinematic, timeStep, init_joint_theta)
stepPlanner = StepPlanner(user_parameters, state_estimation)
vmc = VMC(webots_devices, user_parameters, state_estimation, kinematic, stepPlanner, timeStep)

while robot.step(timeStep) != -1:
    tmp_str = str()
    for i in range(4):
        tmp_str += str(webots_devices.TC_3D[i].getValues()[0]) + ' ' + str(webots_devices.TC_3D[i].getValues()[1]) + \
                   ' ' + str(webots_devices.TC_3D[i].getValues()[2]) + ' '
    with open("body_vel.txt", mode='a+') as f1:
        f1.write(str(state_estimation.com_local_vel[1]) + '\t' + str(state_estimation.euler[0]) + '\n')
        # f1.write(str(webots_devices.get_theta()[0][1]*57.3) + '\t' +str(webots_devices.get_theta()[0][2]*57.3) + '\n')#str(webots_devices.get_theta()[0][0] *57.3)+'\t' +
    # if webots_devices.TC_3D[0].getValues()[1] > 900:
    #     print(tmp_str)
    # print(state_estimation.com_local_pos)
    forces = bytes(tmp_str, encoding='UTF-8')
    webots_devices.emitter.send(forces)
    vmc.vmc_trot()
