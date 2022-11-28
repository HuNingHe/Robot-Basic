from typing import Any, List


class WebotsDevices:
    def __init__(self, robot: Any):
        self.robot = robot

        self.motor = []
        self.motor_torque_limits = [14, 14, 21]
        for _ in range(4):
            self.motor.append([])

        self.pos_sensor = []  # LF RF RB LB
        for _ in range(4):
            self.pos_sensor.append([])

        # TouchSensor
        self.touch_sensor = [self.robot.createTouchSensor("LF_touch sensor"),
                             self.robot.createTouchSensor("RF_touch sensor"),
                             self.robot.createTouchSensor("RH_touch sensor"),
                             self.robot.createTouchSensor("LH_touch sensor")]

        self.TC_3D = [self.robot.createTouchSensor("LF_3D_TC"), self.robot.createTouchSensor("RF_3D_TC"),
                      self.robot.createTouchSensor("RH_3D_TC"), self.robot.createTouchSensor("LH_3D_TC")]

        # Emit message to Supervisor Robot
        self.emitter = self.robot.createEmitter("emitter")

        # Inertial_unit & ACC & GPS & keyboard
        self.IMU = self.robot.createInertialUnit("inertial unit")
        self.GPS = self.robot.createGPS("GPS")
        self.KeyBoard = self.robot.getKeyboard()

    def devices_create(self):

        self.motor[0].append(self.robot.createMotor("LFL0_rotational motor"))
        self.motor[0].append(self.robot.createMotor("LFL1_rotational motor"))
        self.motor[0].append(self.robot.createMotor("LFL2_rotational motor"))

        self.motor[1].append(self.robot.createMotor("RFL0_rotational motor"))
        self.motor[1].append(self.robot.createMotor("RFL1_rotational motor"))
        self.motor[1].append(self.robot.createMotor("RFL2_rotational motor"))

        self.motor[2].append(self.robot.createMotor("RHL0_rotational motor"))
        self.motor[2].append(self.robot.createMotor("RHL1_rotational motor"))
        self.motor[2].append(self.robot.createMotor("RHL2_rotational motor"))

        self.motor[3].append(self.robot.createMotor("LHL0_rotational motor"))
        self.motor[3].append(self.robot.createMotor("LHL1_rotational motor"))
        self.motor[3].append(self.robot.createMotor("LHL2_rotational motor"))

        self.pos_sensor[0].append(self.robot.createPositionSensor("LFL0_position sensor"))
        self.pos_sensor[0].append(self.robot.createPositionSensor("LFL1_position sensor"))
        self.pos_sensor[0].append(self.robot.createPositionSensor("LFL2_position sensor"))

        self.pos_sensor[1].append(self.robot.createPositionSensor("RFL0_position sensor"))
        self.pos_sensor[1].append(self.robot.createPositionSensor("RFL1_position sensor"))
        self.pos_sensor[1].append(self.robot.createPositionSensor("RFL2_position sensor"))

        self.pos_sensor[2].append(self.robot.createPositionSensor("RHL0_position sensor"))
        self.pos_sensor[2].append(self.robot.createPositionSensor("RHL1_position sensor"))
        self.pos_sensor[2].append(self.robot.createPositionSensor("RHL2_position sensor"))

        self.pos_sensor[3].append(self.robot.createPositionSensor("LHL0_position sensor"))
        self.pos_sensor[3].append(self.robot.createPositionSensor("LHL1_position sensor"))
        self.pos_sensor[3].append(self.robot.createPositionSensor("LHL2_position sensor"))

    def devices_init(self, time_step: float):
        self.KeyBoard.enable(time_step)
        self.GPS.enable(time_step)
        self.IMU.enable(time_step)
        self.emitter.setChannel(100)

        for leg in range(4):
            # init touch_sensor
            self.touch_sensor[leg].enable(time_step)
            self.TC_3D[leg].enable(time_step)
            # init position sensor
            for joint in range(3):
                self.pos_sensor[leg][joint].enable(time_step)
                self.motor[leg][joint].enableTorqueFeedback(time_step)

    # set torque for one leg's joint
    def set_motor_torque(self, leg: int, torque: List):
        for joint in range(3):
            if torque[joint] >= self.motor_torque_limits[joint]:
                torque[joint] = self.motor_torque_limits[joint]
            elif torque[joint] <= -self.motor_torque_limits[joint]:
                torque[joint] = -self.motor_torque_limits[joint]
            else:
                torque[joint] = torque[joint]
            self.motor[leg][joint].setTorque(torque[joint])

    def foot_touch_state(self):
        touch = [0, 0, 0, 0]
        for leg in range(4):
            touch[leg] = self.touch_sensor[leg].getValue()
        return touch

    def get_theta(self):
        angle = []
        for _ in range(4):
            angle.append([])

        for leg in range(4):
            for joint in range(3):
                angle[leg].append(self.pos_sensor[leg][joint].getValue())
        return angle
