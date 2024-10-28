from DXL_motor_control import DXL_Communication
from Triceratops_Config import RobotConfiguration, RobotDynamixel, state, Vel, Vector3
from Triceratops_Gait import RobotGait
from Triceratops_Kinematic import RobotIK

import numpy as np
import time
import traceback
import atexit
import threading

DEGREE_TO_SERVO = 4095/360

class RobotControl:
    """High-level control of the robot."""
    def __init__(self):

        self.req_vel = Vel(linear= Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        self.control_cmd = ControlCmd()
        self.gait = RobotGait()
        self.IK = RobotIK()
        self.config = RobotConfiguration()

        self.is_walking = False
        self.motor_position = np.zeros(8)
        self.gait_name = 'move_linear'

        self.t = 0
        self.dt = 0.1
        self.speed = 1
                
        self.lock = threading.Lock()

    def __del__(self): # Called when object is deleted
        self.stop_gait()

    
    def cleanup(self): # Ensures both high and low-level objects are cleaned up
        self.__del__()
        self.stop_gait()
        self.control_cmd.cleanup()
    
    def set_velocity(self, cmd_vel):

        self.req_vel.linear.x  = cmd_vel.linear.x
        self.req_vel.angular.z = cmd_vel.angular.z

        if cmd_vel.linear.x > 0:
            speed = min(cmd_vel.linear.x, 1.0) 
            self.speed = abs(speed)
        elif cmd_vel.linear.x < 0:
            speed = max(cmd_vel.linear.x, -1.0) 
            self.speed = abs(speed)
        elif cmd_vel.angular.z > 0:
            turn_speed = min(cmd_vel.angular.z, 1.0)
            self.speed = abs(turn_speed * 0.5)
        elif cmd_vel.angular.z < 0:
            turn_speed = max(cmd_vel.angular.z, -1.0)
            self.speed = abs(turn_speed * 0.5)
        else:
            self.speed = 0.0

    def _threadUpdateAllMotorData(self, goal):
        """Thread function to send motor positions."""
        with self.lock:  # Ensure exclusive access to the motor control
            self.control_cmd.motor_position_control(goal)
        

    def puppy_move(self, left_leg_move, right_leg_move, leg_height):
        """Main walking function. Updates motor data in a loop."""
        while self.is_walking:
            self.t = (self.t + self.dt) % 1.0
            foot_locations = self.gait.trot(self.t, self.speed * 25, leg_height, left_leg_move, left_leg_move, right_leg_move, right_leg_move)
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
            start_time = time.time()
            execution_time = time.time() - start_time
            time.sleep(max(self.dt - execution_time, 0))


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

    def start_gait(self, movement_type='forward', leg_height=25):
        """Starts the gait based on the specified movement type."""
        movements = {
            'forward': (1, 1),
            'backward': (-1, -1),
            'turn_left': (1, -1),
            'turn_right': (-1, 1)
        }
        left_leg_move, right_leg_move = movements.get(movement_type, (1, 1))
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
        print(self.joint_position/4095*360-90)
        return print(self.joint_position)
    
    def update_joint_state(self):
        self.dynamixel.updateMotorData()
        self.joint_position = np.zeros((3, 4))
        self.joint_position[0] = [0, 0, 0, 0]
        self.joint_position[1] = [motor.PRESENT_POSITION_value for motor in self.leg_motor_list[1]]
        self.joint_position[2] = [motor.PRESENT_POSITION_value for motor in self.leg_motor_list[2]]

    def reset_to_original(self, position=None):
        position = [[   0   , 0  ,  0  ,  0],
                        [2500, 1600, 1600 ,2500],
                        [2000 ,2100, 2100 ,2000]]

        for i, motor_list in enumerate(self.leg_motor_list[1:], start=1):
            for j, motor in enumerate(motor_list):
                # print(int(position[i][j]))
                motor.writePosition(int(position[i][j]))

        self.dynamixel.sentAllCmd()


    def motor_position_control(self, position=None):
        if position is None:
            position = [[   0   , 0  ,  0  ,  0],
                            [2500, 1600, 1600 ,2500],
                            [2000 ,2100, 2100 ,2000]]

        for i, motor_list in enumerate(self.leg_motor_list[1:], start=1):
            for j, motor in enumerate(motor_list):
                # print(int(position[i][j]))
                motor.writePosition(int(position[i][j]))

        self.dynamixel.sentAllCmd()


def main(args=None):
    
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

        # finally:
        #     atexit._run_exitfuncs() 

if __name__ == '__main__':
    main()
