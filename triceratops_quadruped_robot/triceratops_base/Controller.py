import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

from Triceratops_Config import RobotConfiguration, RobotDynamixel

from DXL_motor_control import DXL_Communication
from Gait_controller import GaitController
from Swing_controller import SwingController
from Stance_controller import StanceController
from Triceratops_IK import InverseKinematics

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from math import pi
import threading
import queue
import traceback
import atexit
import time

import threading
import concurrent.futures
import numpy as np

DEGREE_TO_SERVO = 4095/360

class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0

        self.angular_x = 0.0
        self.angular_y = 0.0
        self.angular_z = 0.0

        self.get_logger().info('Vel Subscriber has been started')

    def listener_callback(self, msg):
        self.linear_x = msg.linear.x
        self.linear_y = msg.linear.y
        self.linear_z = msg.linear.z

        self.angular_x = msg.angular.x
        self.angular_y = msg.angular.y
        self.angular_z = msg.angular.z

class RobotState:
    def __init__(self):
        self.config = RobotConfiguration()
        self.ticks = 0                    
        self.height = -0.10
        self.foot_locations = self.config.default_stance
        self.joint_angles = np.zeros((3, 4))
        self.last_goal = np.zeros(4) 

class Command:
    def __init__(self):
        self.horizontal_velocity = np.array([0.0, 0.0])
        self.yaw_rate = 0
        self.height = -0.10

class RobotControl:
    def __init__(self):
        self.cmd_vel = CmdVelSubscriber()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.cmd_vel)

        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self.config = RobotConfiguration()
        self.gait_controller = GaitController(self.config)
        self.swing_controller = SwingController(self.config)
        self.stance_controller = StanceController(self.config)
        self.inverse_kinematics = InverseKinematics(self.config)

        self.state = RobotState()
        self.command = Command()
        self.control_cmd = ControlCmd()

    def get_vel_data(self):
        self.command.horizontal_velocity = np.array([self.cmd_vel.linear_x, self.cmd_vel.linear_y])
        self.command.yaw_rate = self.cmd_vel.angular_z

    def __del__(self):
        pass

    def cleanup(self):
        self.executor.shutdown()

    def step_gait(self, state, command):
        contact_modes = self.gait_controller.contacts(state.ticks)
        new_foot_locations = np.zeros((3, 4))

        for leg_index in range(4):
            contact_mode = contact_modes[leg_index]
            # foot_location = state.foot_locations[:, leg_index]
            if contact_mode == 1:  
                new_location = self.stance_controller.next_foot_location(leg_index, state, command)
            else: 
                swing_proportion = (
                    self.gait_controller.subphase_ticks(state.ticks) / self.config.swing_ticks
                )
                new_location = self.swing_controller.next_foot_location(
                    swing_proportion, leg_index, state, command
                )
            new_foot_locations[:, leg_index] = new_location

        return new_foot_locations, contact_modes

    def puppy_move(self):
        while True:
            self.get_vel_data()

            # Asynchronously calculate foot placements
            with concurrent.futures.ThreadPoolExecutor() as executor:
                future_foot_locations = executor.submit(self.step_gait, self.state, self.command)

            self.state.joint_angles = self.inverse_kinematics.four_legs_inverse_kinematics(
                self.state.foot_locations
            )

            # Wait for foot placements to be ready
            self.state.foot_locations, _ = future_foot_locations.result()

            goal = (
                self.state.joint_angles * 180 / 3.14 * DEGREE_TO_SERVO
                + self.config.leg_center_position
            )

            # Only update motor positions if the goal has significantly changed
            if np.linalg.norm(goal - self.state.last_goal) > self.config.goal_change_threshold:
                self.control_cmd.motor_position_control(goal)
                self.state.last_goal = goal

            self.state.ticks += 1    

class ControlCmd:
    """Low-level control of Dynamixel motors."""
    def __init__(self):
        self.setup_dynamixel()
        self.setup_motors()
        self.initialize_motor_states()
        self.walking_freq = 2000  # Hz

    def setup_dynamixel(self):
        self.robot_dynamixel = RobotDynamixel()
        self.dynamixel = DXL_Communication(self.robot_dynamixel.DEVICE_NAME, self.robot_dynamixel.B_RATE)
        self.dynamixel.activateDXLConnection()
        

    def setup_motors(self):
        motor_names = ['FR_higher', 'FR_lower', 'FR_hip',  
                        'FL_higher', 'FL_lower', 'FL_hip',
                       'RR_higher', 'RR_lower', 'RR_hip',
                       'RL_higher', 'RL_lower', 'RL_hip']
        motor_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
        self.motors = {name: self.dynamixel.createMotor(name, motor_number=id_) 
                    for name, id_ in zip(motor_names, motor_ids)}
        
        self.leg_motor_list = [
            [self.motors['FR_lower'], self.motors['FL_lower'], self.motors['RR_lower'], self.motors['RL_lower']],
            [self.motors['FR_higher'], self.motors['FL_higher'], self.motors['RR_higher'], self.motors['RL_higher']],
            [self.motors['FR_hip'], self.motors['FL_hip'], self.motors['RR_hip'], self.motors['RL_hip']]
        ]
    
    def initialize_motor_states(self):
        self.dynamixel.rebootAllMotor()
        self.dynamixel.updateMotorData()
        self.enable_all_motor()
        self.joint_position = np.zeros(12)

    def __del__(self):
        self.cleanup()

    def cleanup(self):
        self.disable_all_motor()
        self.dynamixel.closeHandler()

    def enable_all_motor(self):
        for motor in self.motors.values():
            motor.enableMotor()

    def disable_all_motor(self):
        for motor in self.motors.values():
            motor.disableMotor()

    def read_all_motor_data(self):
        self.update_joint_state()
        return print(self.joint_position)
    
    def update_joint_state(self):
        self.dynamixel.updateMotorData()
        self.joint_position = np.zeros((3, 4))
        self.joint_position[0] = [motor.PRESENT_POSITION_value for motor in self.leg_motor_list[0]]
        self.joint_position[1] = [motor.PRESENT_POSITION_value for motor in self.leg_motor_list[1]]
        self.joint_position[2] = [motor.PRESENT_POSITION_value for motor in self.leg_motor_list[2]]

    def reset_to_original(self, position=None):
        self.motor_position_control()

    def motor_position_control(self, position=None):
        if position is None:
            position = [[2048 ,2048, 2048, 2048],
                        [1992, 2047, 2092, 2099],
                        [2048 ,2048, 2048, 2048]]
        
        for i, motor_list in enumerate(self.leg_motor_list):
            for j, motor in enumerate(motor_list):
                motor.writePosition(int(position[i][j]))

        self.dynamixel.sentAllCmd()

def main():
    rclpy.init()
    robot_control = RobotControl()

    command_dict = {
        "s":robot_control.puppy_move,
    }

    atexit.register(robot_control.cleanup)

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


if __name__ == "__main__":
    main()
