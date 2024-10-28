import numpy as np
from dataclasses import dataclass

class RobotState:
    def __init__(self):
        self.yaw_rate = 0.0
        self.target_height = 16
        self.pitch = 0.0
        self.roll = 0.0

        self.foot_locations = np.zeros((3, 4))
        self.joint_angles = np.zeros((3, 4))

state = RobotState()

class RobotConfiguration:
     def __init__(self):
        self.upper_leg_length = 63
        self.lower_leg_length = 82
        self.pangolin_height = 125

        self.leg_length = 100

        self.dt = 0.01

        self.leg_motor_direction = np.array([1, -1, 1, -1]) # 1-2
                                                            # 3-4

        self.leg_center_position = np.array([   [0,     0, 0, 0],
                                                [2880, 1230, 1230, 2880],
                                                [2250 ,1820, 1820, 2250]])

        self.move_forward = 15
        self.move_backward = -15
        self.turn_forward = 25
        self.turn_backward = -25

        self.max_linear_vel = self.leg_length * 0.001 *0.001 * np.sin(np.deg2rad(self.move_forward)) * 2/3 / self.dt #0.00216 m/s
        
class RobotDynamixel:
    def __init__(self):
        self.DEVICE_NAME = "/dev/ttyUSB0"
        self.B_RATE      = 57600
        self.LED_ADDR_LEN = (65,1)
        self.LED_ON = 1
        self.LED_OFF = 0

# Leg Linkage for the purpose of hardware interfacing
class Leg_linkage:
    def __init__(self):
        self.a = 65
        self.b = 18 #mm
        self.c = 65 #mm
        self.d = 18 #mm

@dataclass
class Vector3:
    x: float
    y: float
    z: float

@dataclass
class Vel:
    linear: Vector3
    angular: Vector3

if __name__ == "__main__":
    import traceback
    robot_config = RobotConfiguration()

    command_dict = {
        "spine":robot_config.max_linear_vel,
    }


    while True:
        try:
            cmd = input("CMD : ")
            if cmd in command_dict:
                command_dict[cmd]()
            elif cmd == "exit":
                break
        except Exception as e:
            traceback.print_exc()
            break