from DXL_motor_control import DXL_Communication
from Triceratops_Config import RobotConfiguration, RobotDynamixel, state, Vel, Vector3
from Triceratops_Gait import RobotGait
from Triceratops_Kinematic import RobotIK

import numpy as np
import time
import traceback
import atexit
import queue
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
        self.dt = 0.05
        self.speed = 1
                
        self.lock = threading.Lock()
        self.goal_queue = queue.Queue()

        # Start a single thread for updating motor data
        self.exit_signal = threading.Event()  # Unified signal for stopping threads


    def __del__(self):  
        self.stop_gait()
        self.cleanup()

    def cleanup(self):
        self.stop_gait()
        self.control_cmd.cleanup()
    
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
            self.thread_update_motor_data = threading.Thread(
                target=self._threadUpdateAllMotorData,
                args=(goal,), 
                daemon=True
            )
            self.thread_update_motor_data.start()


    def initiate_gait(self, left_leg_move, right_leg_move, leg_height):
        self.stop_gait()  # Stop any existing movement
        self.is_walking = True
        print("Truw or False",self.exit_signal.is_set())

        # Start the puppy_move thread
        self.puppy_move_thread = threading.Thread(
            target=self.puppy_move,
            args=(left_leg_move, right_leg_move, leg_height)
        )
        self.puppy_move_thread.start()

    def start_gait(self, movement_type='forward', leg_height=25):
        left_leg_move, right_leg_move = movements.get(movement_type, (1, 1))
        self.initiate_gait(left_leg_move, right_leg_move, leg_height)

    def stop_gait(self):
        """Stops the gait and waits for the thread to exit."""
        self.is_walking = False
        self.is_turning = False

    def stop(self):
        """Stops the gait and resets to the original position."""
        self.exit_signal.set()  # Signal all threads to stop
        if self.thread_update_motor_data.is_alive():
            self.thread_update_motor_data.join()
        if self.puppy_move_thread.is_alive():
            self.puppy_move_thread.join()  # Ensure thread completes before moving on
        self.stop_gait()  # Stop walking and turning
        self.exit_signal.clear()  # 清除事件，讓執行緒再次進入等待狀態

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
