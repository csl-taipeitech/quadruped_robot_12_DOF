import numpy as np

class RobotConfiguration:
     def __init__(self):
        self.upper_leg_length = 65
        self.lower_leg_length = 90
        self.pangolin_height = 125

        self.dt = 0.1

        self.leg_motor_direction = np.array([1, -1, 1, -1]) # 1-2
                                                            # 3-4

        self.leg_center_position = np.array([   [2048, 2048, 2048, 2048],
                                                [2048, 2048, 2048, 2088],
                                                [2048 ,2048, 2048, 2048]])

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
        self.a = 62
        self.b = 17.5 #mm
        self.c = 62 #mm
        self.d = 17.5 #mm

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