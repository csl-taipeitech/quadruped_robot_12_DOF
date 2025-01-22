import numpy as np
#from dataclasses import dataclass

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
        #################### OUTPUT　MOTOR #######################
        self.goal_change_threshold = 1.0

        #################### MOVEMENT PARAMS ####################
        self.z_time_constant = 0.02

        ######################## GEOMETRY ######################
        self.upper_leg_length = 0.065
        self.lower_leg_length = 0.065
        self.leg_front_back = 0.09  # front-back distance from center line to leg axis
        self.leg_left_right = 0.048  # left-right distance from center line to leg plane
        self.pangolin_height = 0.16
        self.ABDUCTION_OFFSET = 0.0365

        #self.leg_motor_direction = np.array([1, -1, 1, -1]) # 1-2
                                                            # 3-4

        self.leg_center_position = np.array([   [2048, 2048, 2048, 2048],
                                                [2048, 2048, 2048, 2048],
                                                [2048 ,2048, 2048, 2048]])
        self.LEG_ORIGINS = np.array(
            [   [self.leg_front_back, self.leg_front_back, -self.leg_front_back, -self.leg_front_back],
                [-self.leg_left_right, self.leg_left_right, -self.leg_left_right, self.leg_left_right],
                [0, 0, 0, 0],
            ]
        )

        self.ABDUCTION_OFFSETS = np.array(
            [
                -self.ABDUCTION_OFFSET,
                self.ABDUCTION_OFFSET,
                -self.ABDUCTION_OFFSET,
                self.ABDUCTION_OFFSET,
            ]
        )
        #################### STANCE ####################
        self.delta_x = 0.09067
        self.delta_y = 0.085
        self.x_shift = 0.0
        self.x_shift_back = -0.0
        self.default_z_ref = -0.10

        #################### SWING ######################
        self.z_coeffs = None
        self.z_clearance = 0.03
        self.alpha = (
            1  # Ratio between touchdown distance and total horizontal stance movement
        )
        self.beta = (
            0.5  # Ratio between touchdown distance and total horizontal stance movement
        )

        #################### GAIT #######################
        self.dt = 0.02
        self.num_phases = 4
        self.contact_phases = np.array(
            [[1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 1], [1, 1, 1, 0]]
        )
        self.overlap_time = (
            0.15  # duration of the phase where all four feet are on the ground
        )
        self.swing_time = (
            0.2  # duration of the phase when only two feet are on the ground
        )
        
    @property
    def default_stance(self):
        return np.array(
            [
                [
                    self.delta_x + self.x_shift,
                    self.delta_x + self.x_shift,
                    -self.delta_x + self.x_shift,
                    -self.delta_x + self.x_shift,
                ],
                [-self.delta_y, self.delta_y, -self.delta_y, self.delta_y],
                [self.default_z_ref, self.default_z_ref, self.default_z_ref, self.default_z_ref],
            ]
        )

    ################## SWING ###########################
    @property
    def z_clearance(self):
        return self.__z_clearance

    @z_clearance.setter
    def z_clearance(self, z):
        self.__z_clearance = z

    ########################### GAIT ####################
    @property
    def overlap_ticks(self):
        return int(self.overlap_time / self.dt)

    @property
    def swing_ticks(self):
        return int(self.swing_time / self.dt)

    @property
    def stance_ticks(self):
        return 2 * self.overlap_ticks + self.swing_ticks

    @property
    def phase_ticks(self):
        return np.array(
            [self.overlap_ticks, self.swing_ticks, self.overlap_ticks, self.swing_ticks]
        )

    @property
    def phase_length(self):
        return 2 * self.overlap_ticks + 2 * self.swing_ticks

class RobotDynamixel:
    def __init__(self):
        self.DEVICE_NAME = "/dev/ttyUSB0"
        self.B_RATE      = 57600
        self.LED_ADDR_LEN = (65,1)
        self.LED_ON = 1
        self.LED_OFF = 0