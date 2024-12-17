from DXL_motor_control import DXL_Communication
from Triceratops_Config import RobotConfiguration, RobotDynamixel, state
from Triceratops_Gait import RobotGait
from Triceratops_Kinematic import RobotIK
from Raibert_Gait import RaibertController

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import numpy as np
import time
import traceback
import atexit
import threading
import math
import matplotlib.pyplot as plt
from transforms3d.euler import euler2mat, quat2euler

DEGREE_TO_SERVO = 4095/360

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',  # topic name
            self.imu_callback,
            10)  # QoS profile depth
        
        # Initialize IMU data containers
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.angular_velocity = np.zeros(3)
        self.linear_acceleration = np.zeros(3)

        self.get_logger().info('IMU Subscriber has been started')

    def imu_callback(self, msg):
        orientation = msg.orientation
        
        # Convert quaternion to Euler angles
        roll = math.atan2(2.0 * (orientation.w * orientation.x + orientation.y * orientation.z),
                                1.0 - 2.0 * (orientation.x * orientation.x + orientation.y * orientation.y))
        pitch = math.asin(2.0 * (orientation.w * orientation.y - orientation.z * orientation.x))
        yaw = math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                                1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))
        
        # Convert radians to degrees
        self.roll = roll -0.01
        self.pitch = pitch -0.01
        self.yaw = yaw

        # Update angular velocity and linear acceleration
        self.angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        self.linear_acceleration = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

    def get_imu_data(self):
        return {
            'roll': self.roll,
            'pitch': self.pitch,
            'yaw': self.yaw,
            'angular_velocity': self.angular_velocity.copy(),
            'linear_acceleration': self.linear_acceleration.copy()
        }


class RobotControl:
    """High-level control of the robot."""
    def __init__(self):
        self.imusub = ImuSubscriber()

        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.imusub)

        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        # self.req_vel = Vel(linear= Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        self.control_cmd = ControlCmd()
        self.gait = RobotGait()
        self.IK = RobotIK()
        self.config = RobotConfiguration()
        self.tempgait = RaibertController()

        self.is_walking = False
        self.motor_position = np.zeros(8)
        self.gait_name = 'move_linear'

        self.t = 0
        self.tall = 0
        self.dt = 0.05

        self.speed = 1
        
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.use_imu = True

        # Thread lock for motor position control
        self.lock = threading.Lock()

        self.all_foot_locations = []
        self.all_time = []
        self.imudata_roll = []
        self.imudata_pitch = []

    def __del__(self): # Called when object is deleted
        self.stop_gait()

    
    def cleanup(self): # Ensures both high and low-level objects are cleaned up
        self.__del__()
        self.stop_gait()
        self.control_cmd.cleanup()
    
    def get_imu_data(self):
        """Get the latest IMU data"""
        imu_data = self.imusub.get_imu_data()
        self.imudata_roll.append(imu_data["roll"])
        self.imudata_pitch.append(imu_data["pitch"])
        self.roll = imu_data["roll"]
        self.pitch = imu_data["pitch"]
        # self.roll = 0
        # self.pitch = 0
        self.yaw = 0
        

    def _threadUpdateAllMotorData(self, goal):
        """Thread function to send motor positions."""
        with self.lock:  # Ensure exclusive access to the motor control
            self.control_cmd.motor_position_control(goal)
        

    def puppy_move(self, left_leg_move, right_leg_move, leg_height):
        """Main walking function. Updates motor data in a loop."""
        while self.is_walking:
            self.t = (self.t + self.dt) % 1.0
            self.tall += 1
            self.get_imu_data()
            #foot_locations = self.gait.trot(self.t, self.speed * 25, leg_height, left_leg_move, left_leg_move, right_leg_move, right_leg_move)
            new_foot_locations_12,new_foot_locations_34 = self.tempgait.get_foot_locations(self.t)
            # Tilt compensation
            if self.use_imu:
                correction_factor = -0.5
                max_tilt = 0.4
                
                roll_compensation_12 = correction_factor * np.clip(-self.roll, -max_tilt, max_tilt)
                pitch_compensation_12 = correction_factor * np.clip(- self.pitch, -max_tilt, max_tilt)
                rmat_12 = euler2mat(roll_compensation_12, pitch_compensation_12, 0)

                roll_compensation_34 = correction_factor * np.clip(self.roll, -max_tilt, max_tilt)
                pitch_compensation_34 = correction_factor * np.clip(self.pitch, -max_tilt, max_tilt)
                rmat_34 = euler2mat(roll_compensation_34, pitch_compensation_34, 0)

                new_foot_locations_12 = rmat_12.T @ new_foot_locations_12
                new_foot_locations_34 = rmat_34.T @ new_foot_locations_34

                roll_comp = np.abs(self.roll) * 15  #Scale factor for roll compensation
                print("roll_comp",roll_comp)
                if self.roll > 0:  # Leaning right
                    new_foot_locations_12[2, 0:1] += roll_comp  # Lower left feet
                    new_foot_locations_34[2, 0:1] += roll_comp  # Lower left feet
                    new_foot_locations_12[2, 1:2] -= roll_comp  # Lift right feet
                    new_foot_locations_34[2, 1:2] -= roll_comp  # Lift right feet
                else:  # Leaning left
                    new_foot_locations_12[2, 0:1] -= roll_comp  # Lift left feet
                    new_foot_locations_34[2, 0:1] -= roll_comp  # Lift left feet
                    new_foot_locations_12[2, 1:2] += roll_comp  # Lower right feet
                    new_foot_locations_34[2, 1:2] += roll_comp  # Lower right feet

                pitch_comp = np.abs(self.pitch) * 20  # Scale factor for roll compensation

                if self.pitch > 0:  # Leaning forward
                    new_foot_locations_12[2, 0:2] -= pitch_comp  # Lower forward feet
                    new_foot_locations_34[2, 0:2] += pitch_comp  # Lift backward feet
                else:  # Leaning Backward
                    new_foot_locations_12[2, 0:2] += pitch_comp  # Lift forward feet
                    new_foot_locations_34[2, 0:2] -= pitch_comp  # Lower backward feet

            foot_locations = np.array([
                [new_foot_locations_12[0, 0], new_foot_locations_12[0, 1] , new_foot_locations_34[0, 0] , new_foot_locations_34[0, 1]], 
                [0, 0, 0, 0], 
                [new_foot_locations_12[2, 0], new_foot_locations_12[2, 1] , new_foot_locations_34[2, 0] , new_foot_locations_34[2, 1]]
            ])
            #print("foot_locations",foot_locations)
            self.all_foot_locations.append(foot_locations)
            self.all_time.append(self.tall)
            joint_angles = self.IK.leg_inverse_kinematics(foot_locations)
            goal = joint_angles * 180 / 3.14 * DEGREE_TO_SERVO + self.config.leg_center_position

            # Start thread to update motor positions
            thread_update_motor_data = threading.Thread(
                target=self._threadUpdateAllMotorData,
                args=(goal,), 
                daemon=True
            )
            thread_update_motor_data.start()

            # Control the execution time to avoid CPU overload
            time.sleep(0.05)


    def initiate_gait(self, left_leg_move, right_leg_move, leg_height):
        """Starts the gait in a thread with given movement parameters."""
        self.stop_gait()  # Ensure any ongoing gait is stopped
        self.is_walking = True
        self.is_turning = True

        # Start the puppy_move thread
        self.puppy_move_thread = threading.Thread(
            target=self.puppy_move,
            args=(left_leg_move, right_leg_move, leg_height),
            daemon=True
        )
        self.puppy_move_thread.start()

    def start_gait(self, left_leg_move=1, right_leg_move=1, leg_height=20):
        """Starts the gait for moving forward."""
        self.initiate_gait(left_leg_move, right_leg_move, leg_height)

    def move_backward(self, left_leg_move=-1, right_leg_move=-1, leg_height=20):
        """Starts the gait for moving backward."""
        self.initiate_gait(left_leg_move, right_leg_move, leg_height)

    def turn_left(self, left_leg_move=1, right_leg_move=-1, leg_height=20):
        """Starts the gait for turning left."""
        self.initiate_gait(left_leg_move, right_leg_move, leg_height)

    def turn_right(self, left_leg_move=-1, right_leg_move=1, leg_height=20):
        """Starts the gait for turning right."""
        self.initiate_gait(left_leg_move, right_leg_move, leg_height)

    def stop_gait(self):
        """Stops the gait and waits for the thread to exit."""
        self.is_walking = False
        self.is_turning = False
        if hasattr(self, 'puppy_move_thread'):
            self.puppy_move_thread.join()  # Ensure thread completes before moving on

    def stop(self):
        """Stops the gait and resets to the original position."""
        self.stop_gait()  # Stop walking and turning
        self.control_cmd.reset_to_original()
        self.all_foot_locations = np.array(self.all_foot_locations)
        self.all_time = np.array(self.all_time)
        #print("all_foot_locations",self.all_foot_locations)

        # Create a figure with a 3x2 grid layout
        fig = plt.figure(figsize=(12, 12))

        # Plot X-Y trajectory for each leg (4個子圖分布在前兩行)
        leg_names = ['Front Left', 'Front Right', 'Back Left', 'Back Right']
        for i in range(4):
            ax = fig.add_subplot(3, 2, i + 1)  # 使用 3 行 2 列的布局
            ax.plot(self.all_foot_locations[:, 0, i],
                    self.all_foot_locations[:, 2, i],  # Z coordinate trajectory
                    label=leg_names[i])
            ax.set_xlabel('X Position (mm)')
            ax.set_ylabel('Z Position (mm)')
            ax.set_title(f'Leg Trajectories: {leg_names[i]}')
            ax.grid(True)
            ax.legend()

        # Plot IMU data (Roll) - 第5個子圖放在第3行第1列
        ax5 = fig.add_subplot(3, 2, 5)
        ax5.plot(self.all_time, self.imudata_roll, label="Roll")
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Roll (degrees)')
        ax5.set_title('IMU Data - Roll')
        ax5.grid(True)
        ax5.legend()

        # Plot IMU data (Pitch) - 第6個子圖放在第3行第2列
        ax6 = fig.add_subplot(3, 2, 6)
        ax6.plot(self.all_time, self.imudata_pitch, label="Pitch", color='orange')
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Pitch (degrees)')
        ax6.set_title('IMU Data - Pitch')
        ax6.grid(True)
        ax6.legend()

        # Adjust layout
        plt.tight_layout()

        # Save the plot as an image file
        plt.savefig('leg_trajectories_3x2.png', dpi=300, bbox_inches='tight')
        plt.show()


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
        motor_names = ['FL_higher', 'FL_lower', 'FR_higher', 'FR_lower',
                       'RL_lower', 'RL_higher', 'RR_lower', 'RR_higher']
        self.motors = {name: self.dynamixel.createMotor(f'motor{i+1}', motor_number=i+1) for i, name in enumerate(motor_names)}
        
        self.leg_motor_list = [
            [0, 0, 0, 0],
            [self.motors['FL_higher'], self.motors['FR_higher'], self.motors['RL_higher'], self.motors['RR_higher']],
            [self.motors['FL_lower'], self.motors['FR_lower'], self.motors['RL_lower'], self.motors['RR_lower']]
        ]

    def initialize_motor_states(self):
        self.dynamixel.rebootAllMotor()
        self.dynamixel.updateMotorData()
        self.enable_all_motor()
        self.joint_position = np.zeros(8)

    def __del__(self):
        self.cleanup()

    def cleanup(self):
        self.disable_all_motor()
        self.dynamixel.closeHandler()

    def enable_all_motor(self):
        for motor_list in self.leg_motor_list[1:]:
            for motor in motor_list:
                motor.enableMotor()

    def disable_all_motor(self):
        for motor_list in self.leg_motor_list[1:]:
            for motor in motor_list:
                motor.disableMotor()

    def read_all_motor_data(self):
        self.update_joint_state()
        print(self.joint_position/4095*360)
        return print(self.joint_position)
    
    def update_joint_state(self):
        self.dynamixel.updateMotorData()
        self.joint_position = np.zeros((3, 4))
        self.joint_position[0] = [0, 0, 0, 0]
        self.joint_position[1] = [motor.PRESENT_POSITION_value for motor in self.leg_motor_list[1]]
        self.joint_position[2] = [motor.PRESENT_POSITION_value for motor in self.leg_motor_list[2]]

    def reset_to_original(self, position=None):
        self.motor_position_control()


    def motor_position_control(self, position=None):
        if position is None:
            position = [[0, 0, 0, 0],
                        [2021, 2022, 2097, 2097],
                        [2048 ,2048, 2048, 2048]]

        for i, motor_list in enumerate(self.leg_motor_list[1:], start=1):
            for j, motor in enumerate(motor_list):
                # print(int(position[i][j]))
                motor.writePosition(int(position[i][j]))

        self.dynamixel.sentAllCmd()


def main(args=None):
    rclpy.init()
    robot_control = RobotControl()

    command_dict = {
        "s":robot_control.start_gait,
        "b":robot_control.move_backward,
        "l":robot_control.turn_left,
        "r":robot_control.turn_right,

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

        # finally:
        #     atexit._run_exitfuncs() 

if __name__ == '__main__':
    main()