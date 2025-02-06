from DXL_motor_control import DXL_Communication
from Triceratops_Config import RobotConfiguration, RobotDynamixel
from math import pi

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import  JointState

import numpy as np
import time
import traceback
import atexit
import threading

DEGREE_TO_SERVO = 4095/360

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',  
            self.joint_state_callback,
            10
        )
        self.get_logger().info('Subscribed to joint state topic.')
        self.joint_data = {}

    def joint_state_callback(self, msg):
        joint_names = msg.name
        joint_positions = msg.position
        self.joint_data = dict(zip(joint_names, joint_positions))
    
    def get_jointAngle_data(self):
        return self.joint_data

class RobotControl:
    """High-level control of the robot."""
    def __init__(self):
        self.jointAnglesub = JointStateSubscriber()

        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.jointAnglesub)

        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self.control_cmd = ControlCmd()
        self.config = RobotConfiguration()

        self.is_walking = False
        self.motor_position = np.zeros(8)

        self.dt = 0.1 #0.05
        self.t = 0

        self.lock = threading.Lock()

        self.jointAngle_data = {}
        self.joint_angles = None

    def __del__(self): 
        self.stop_gait()

    def stop_executor(self):
        self.executor.shutdown()
        self.spin_thread.join()  
    
    def cleanup(self): 
        self.__del__()
        self.stop_gait()
        self.control_cmd.cleanup()
    
    def get_jointAngle_data(self):
        self.jointAngle_data = self.jointAnglesub.get_jointAngle_data()

        jointAngleUpper1 = pi/2 - self.jointAngle_data['fru']
        jointAngleUpper2 = pi/2 - self.jointAngle_data['flu']
        jointAngleUpper3 = pi/2 + self.jointAngle_data['bru']
        jointAngleUpper4 = pi/2 + self.jointAngle_data['blu']

        THETA01 = self.lower_leg_angle_to_servo_angle( jointAngleUpper1, - self.jointAngle_data['frd'])
        THETA02 = self.lower_leg_angle_to_servo_angle( jointAngleUpper2, - self.jointAngle_data['fld'])
        THETA03 = self.lower_leg_angle_to_servo_angle( jointAngleUpper3, self.jointAngle_data['brd'])
        THETA04 = self.lower_leg_angle_to_servo_angle( jointAngleUpper4, self.jointAngle_data['bld'])
        self.joint_angles = np.array([[THETA01, - THETA02, - THETA03, THETA04],
                                [jointAngleUpper1, - jointAngleUpper2, -jointAngleUpper3, jointAngleUpper4],
                                [self.jointAngle_data['fr'], -self.jointAngle_data['fl'], self.jointAngle_data['br'], - self.jointAngle_data['bl']]])

    def lower_leg_angle_to_servo_angle(self, THETA2, THETA3):
        ''' Converts the direct angles of the upper and lower leg joint from the inverse kinematics to the angle
        at the servo that drives the lower leg via a linkage. 
        Parameters
            ----------
        THETA2 : float
            angle of upper leg from the IK 
        THETA3 : float
            angle of lower leg from the IK 
        link: Leg_linage
            A linkage class with all link lengths and relevant angles stored. Link values are based off
            the physcial design of the link

        Returns
        -------
        THETA0: float
            The angle of the servo that drives the outside of the linkage
        '''
        
        THETA0 = 2*pi - 0.75*pi -THETA3 -THETA2
        THETA0 = pi/2- THETA0

        return THETA0
    
    def _threadUpdateAllMotorData(self, goal):
        """Thread function to send motor positions."""
        with self.lock:  
            self.control_cmd.motor_position_control(goal)

    def puppy_move(self):
        """Main walking function. Updates motor data in a loop."""
        while self.is_walking:
            self.t = (self.t + self.dt) % 1.0
            self.get_jointAngle_data()
            goal = self.joint_angles * 180 / 3.14 * DEGREE_TO_SERVO + self.config.leg_center_position
            thread_update_motor_data = threading.Thread(
                target=self._threadUpdateAllMotorData,
                args=(goal,), 
                daemon=True
            )
            thread_update_motor_data.start()

            time.sleep(0.05)

    def initiate_gait(self):
        """Starts the gait in a thread with given movement parameters."""
        self.stop_gait()  
        self.is_walking = True

        self.puppy_move_thread = threading.Thread(
            target=self.puppy_move,
            daemon=True
        )
        self.puppy_move_thread.start()

    def start_gait(self):
        """Starts the gait for moving forward."""
        self.initiate_gait()

    def stop_gait(self):
        """Stops the gait and waits for the thread to exit."""
        self.is_walking = False
        if hasattr(self, 'puppy_move_thread'):
            self.puppy_move_thread.join()  

    def stop(self):
        """Stops the gait and resets to the original position."""
        self.stop_gait()  
        self.robot_control.stop_executor()
        self.control_cmd.reset_to_original()


class ControlCmd:
    """Low-level control of Dynamixel motors."""
    def __init__(self):
        self.setup_dynamixel()
        self.setup_motors()
        self.initialize_motor_states()
        self.walking_freq = 2000  # Hz

    def setup_dynamixel(self):
        robot_dynamixel = RobotDynamixel()
        self.dynamixel = DXL_Communication(robot_dynamixel.DEVICE_NAME, robot_dynamixel.B_RATE)
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
        #print(self.joint_position/4095*360)
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
                        [2048, 2048, 2048, 2048],
                        [2048 ,2048, 2048, 2048]]
            
        for i, motor_list in enumerate(self.leg_motor_list):
            for j, motor in enumerate(motor_list):
                motor.writePosition(int(position[i][j]))

        self.dynamixel.sentAllCmd()


def main(args=None):
    rclpy.init()
    robot_control = RobotControl()

    command_dict = {
        "s":robot_control.start_gait,

        "stop":robot_control.stop,

        "disable":robot_control.control_cmd.disable_all_motor,
        "enable":robot_control.control_cmd.enable_all_motor,
        "read":robot_control.control_cmd.read_all_motor_data,
        "reset":robot_control.control_cmd.reset_to_original,
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

if __name__ == '__main__':
    main()