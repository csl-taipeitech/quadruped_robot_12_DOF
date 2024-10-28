from DXL_motor_control import DXL_Communication
from Triceratops_Config import RobotConfiguration, PangolinDynamixel, state, PuppyConfiguration
from Triceratops_Gait import RobotGait
#from Triceratops_Kinematic import RobotIK

from math import asin,acos,atan,pi,sqrt, radians, degrees
import numpy as np
import time
import traceback
import atexit
import threading

DEGREE_TO_SERVO = 4095/360


class RobotControl:
    """High-level control of the Pangolin robot."""
    def __init__(self):

        # self.req_vel = Vel(linear= Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        self.control_cmd = ControlCmd()
        self.gait = RobotGait()
        self.IK = RobotIK()
        self.config = RobotConfiguration()

        self.is_walking = False
        self.motor_position = np.zeros(8)
        self.gait_name = 'move_linear'

        self.gait = RobotGait()
        self.IK = RobotIK()

        self.t = 0
        self.dt = 0.01
        self.speed = 0


    def __del__(self): # Called when object is deleted
        self.stop_gait()

    
    def cleanup(self): # Ensures both high and low-level objects are cleaned up
        self.__del__()
        self.control_cmd.cleanup() 


    def reset_to_orginal(self): 
        """Resets the robot to its default pose."""

        self.angle_to_servo(np.zeros(8))
        self.control_cmd.motor_position_control(self.motor_position)


    def puppy_move(self, left_leg_move = 1, right_leg_move = 1, leg_height = 8):
        while self.t <= 1:
            foot_locations = self.gait.trot(self.t, self.speed*25, leg_height, left_leg_move, left_leg_move, right_leg_move, right_leg_move)
            print(foot_locations)
            # print(foot_locations)
            # foot_locations = np.array([ [ 0,   0,    0,    0],
            #                             [-100,  -100,   -100,   -100],
            #                             [       0,         0,          0,          0]])
            joint_angles = self.IK.leg_inverse_kinematics(foot_locations)
            # goal = self.IK.map_angles_to_servo(foot_locations)
            # print(joint_angles * 180/3.14)
            # self.servo_PWM.set_actuator_positions(joint_angles)
            # position = [   [0, 0, 0, 0],
            #                                     [1300, 700, 700, 1300],
            #                                     [1200 ,1900, 1900, 1200]]
            # position = self.config
            # print(joint_angles)
            goal = joint_angles*180/3.14*4095/360+self.config.leg_center_position
            # goal = joint_angles*180/3.14*4095/360+position
            print(goal/4095*360)
            self.control_cmd.motor_position_control(goal)
            self.t += self.dt
        self.t = 0

    def loop(self):
        while True:
            self.puppy_move()


    def process_gait(self):
        """Executes the selected gait pattern."""

        gait_num = 0
        while self.is_walking == True:

            gait = self.pangolin_gait.gait_dic[self.gait_name]
            if gait_num >= len(gait): gait_num = 0
            leg_gait_position = gait[gait_num]
            gait_num += 1
            leg_angle, head_angle, spine_angle = self.pangolin_kinematic.calculate_joint(self.gait_name, leg_gait_position, self.req_vel)
            motor_position = self.angle_to_servo(leg_angle, head_angle, spine_angle)
            self.control_cmd.motor_position_control(motor_position)

            


    def angle_to_servo(self, leg_motor_angle: np.array, head_motor_angle: np.array, spine_motor_angle: np.array)-> np.array: 
        """Converts desired joint angles into raw motor positions."""

        leg_motor_pos = leg_motor_angle*DEGREE_TO_SERVO*np.transpose(self.pangolin_config.leg_motor_direction) + self.leg_center_position[:4] #TODO　4095/360 to constant
        self.motor_position[:8] = leg_motor_pos
        return self.motor_position


    def start_gait(self):
        """Starts the gait in a thread."""

        self.is_walking = True
        self.is_turning = True

        self.walking_thread = threading.Thread(target=self.process_gait, args=(), daemon=True)
        self.walking_thread.start()

    # Stop moving 
    def stop_gait(self):
        """Stops the gait and resets to the original position."""

        self.is_walking = False
        self.is_turning = False

        self.reset_to_orginal()


class RobotIK():
    def __init__(self):
        self.config = PuppyConfiguration()
        self.linkage = Leg_linkage()
        self.upper_leg_length = self.config.upper_leg_length
        self.lower_leg_length = self.config.lower_leg_length
        
    def leg_inverse_kinematics(self, foot_locations):

        leg_1_x, leg_2_x, leg_3_x, leg_4_x = foot_locations[0, 0], foot_locations[0, 1], foot_locations[0, 2], foot_locations[0, 3]
        leg_1_y, leg_2_y, leg_3_y, leg_4_y = foot_locations[1, 0], foot_locations[1, 1], foot_locations[1, 2], foot_locations[1, 3]

        #Leg1
        leg_1_x = -leg_1_x
        lower_leg_1 = pi - acos((leg_1_x**2 + leg_1_y**2 - self.upper_leg_length**2 - self.lower_leg_length**2) / (-2 * self.upper_leg_length * self.lower_leg_length))
        phi_1 = acos((self.upper_leg_length**2 + leg_1_x**2 + leg_1_y**2 - self.lower_leg_length**2) / (2 * self.upper_leg_length * sqrt(leg_1_x**2 + leg_1_y**2)))

        if leg_1_x > 0:
            upper_leg_1 = (abs(atan(leg_1_y / - leg_1_x)) - phi_1)
        elif leg_1_x < 0:
            upper_leg_1 = (pi + abs(atan(leg_1_y /leg_1_x)) - phi_1)
        else:
            upper_leg_1 = (pi - 1.5707 - phi_1)

        THETA01 = self.lower_leg_angle_to_servo_angle(upper_leg_1, lower_leg_1)
        print("THETA02", THETA01 * 180 / 3.14)

        #Leg2
        leg_2_x = -leg_2_x
        lower_leg_2 = pi - acos((leg_2_x**2 + leg_2_y**2 - self.upper_leg_length**2 - self.lower_leg_length**2) / (-2 * self.upper_leg_length * self.lower_leg_length))
        phi_2 = acos((self.upper_leg_length**2 + leg_2_x**2 + leg_2_y**2 - self.lower_leg_length**2) / (2 * self.upper_leg_length * sqrt(leg_2_x**2 + leg_2_y**2)))

        if leg_2_x > 0:
            upper_leg_2 = (abs(atan(leg_2_y / leg_2_x)) - phi_2)
        elif leg_2_x < 0:
            upper_leg_2 = (pi - abs(atan(leg_2_y / leg_2_x)) - phi_2)
        else:
            upper_leg_2 = (pi - 1.5707 - phi_2)

        THETA02 = self.lower_leg_angle_to_servo_angle(upper_leg_2, lower_leg_2)
        print("THETA02", THETA02 * 180 / 3.14)

        #Leg3
        leg_3_x = -leg_3_x
        lower_leg_3 = pi - acos((leg_3_x**2 + leg_3_y**2 - self.upper_leg_length**2 - self.lower_leg_length**2) / (-2 * self.upper_leg_length * self.lower_leg_length))
        phi_3 = acos((self.upper_leg_length**2 + leg_3_x**2 + leg_3_y**2 - self.lower_leg_length**2) / (2 * self.upper_leg_length * sqrt(leg_3_x**2 + leg_3_y**2)))

        if leg_3_x > 0:
            upper_leg_3 = (abs(atan(leg_3_y / leg_3_x)) - phi_3)
        elif leg_3_x < 0:
            upper_leg_3 = (pi - abs(atan(leg_3_y / leg_3_x)) - phi_3)
        else:
            upper_leg_3 = (pi - 1.5707 - phi_3)

        THETA03 = self.lower_leg_angle_to_servo_angle(upper_leg_3, lower_leg_3)
        print("THETA03", THETA03 * 180 / 3.14)

        #Leg4
        leg_4_x = -leg_4_x
        lower_leg_4 = pi - acos((leg_4_x**2 + leg_4_y**2 - self.upper_leg_length**2 - self.lower_leg_length**2) / (-2 * self.upper_leg_length * self.lower_leg_length))
        phi_4 = acos((self.upper_leg_length**2 + leg_4_x**2 + leg_4_y**2 - self.lower_leg_length**2) / (2 * self.upper_leg_length * sqrt(leg_4_x**2 + leg_4_y**2)))

        if leg_4_x > 0:
            upper_leg_4 = (abs(atan(leg_4_y / leg_4_x)) - phi_4)
        elif leg_4_x < 0:
            upper_leg_4 = (pi - abs(atan(leg_4_y / leg_4_x)) - phi_4)
        else:
            upper_leg_4 = (pi - 1.5707 - phi_4)

        THETA04 = self.lower_leg_angle_to_servo_angle(upper_leg_4, lower_leg_4)
        print("upper_leg_4", upper_leg_4 * 180 / 3.14)

        joint_angles = np.array([[0., 0., 0., 0.],
                        [0, upper_leg_2 - pi / 2, 0, 0],
                        [-0, THETA02 +3*pi/4 - pi /8, -0, - 0]])


        # joint_angles = np.array([[0., 0., 0., 0.],
        #                         [- upper_leg_1 + pi / 4, upper_leg_2 - pi / 2, upper_leg_3 - pi / 2, - upper_leg_4 +  pi / 4],
        #                         [- THETA01  -3*pi/4 + pi /8 +pi/4, THETA02 +3*pi/4 - pi /8, THETA03 +3*pi/4 - pi /8, - THETA04 -3*pi/4 + pi /8 + pi/4]])

        state.joint_angles = joint_angles
        return joint_angles
    
    def calculate_4_bar(self, th2 ,a,b,c,d):
        """Using 'Freudensteins method', it finds all the angles within a 4 bar linkage with vertices ABCD and known link lengths a,b,c,d
        defined clockwise from point A, and known angle, th2.

        Parameters
        ----------
        th2 : float
            the input angle at the actuating joint of the 4 bar linkage, aka angle DAB
        a,b,c,d: floats
            link lengths, defined in a clockwise manner from point A.
        
        Returns
        -------
        ABC,BCD,CDA: floats
            The remaining angles in the 4 bar linkage
        
        """
        # print('th2: ',m.degrees(th2),'a: ',a,'b: ',b,'c: ',c,'d: ',d)    
        x_b = a*np.cos(th2)
        y_b = a*np.sin(th2)
        
        #define diagnonal f
        f = np.sqrt((d-x_b)**2 +y_b**2)
        beta = np.arccos((f**2+c**2-b**2)/(2*f*c))
        gamma = np.arctan2(y_b,d-x_b)
        
        th4 = np.pi - gamma - beta
        
        x_c = c*np.cos(th4)+d
        y_c = c*np.sin(th4)
        
        th3 = np.arctan2((y_c-y_b),(x_c-x_b))
        
        
        ## Calculate remaining internal angles of linkage
        ABC = np.pi-th2 + th3
        BCD  = th4-th3
        CDA = np.pi*2 - th2 - ABC - BCD
                        
        return ABC,BCD,CDA
    

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

        # First 4 bar linkages
        GDE,DEF,EFG = self.calculate_4_bar(THETA3 + self.linkage.lower_leg_bend_angle,self.linkage.i,self.linkage.h,self.linkage.f,self.linkage.g) #+ link.lower_leg_bend_angle
        # Triangle section
        CDH = 3/2*pi - THETA2 - GDE - self.linkage.EDC
        CDA = CDH +self.linkage.gamma #input angle
        # Second 4 bar linkage
        DAB,ABC,BCD = self.calculate_4_bar(CDA ,self.linkage.d,self.linkage.a,self.linkage.b,self.linkage.c)
        #Calculating Theta
        THETA0 = - DAB + self.linkage.gamma

        return THETA0

# Leg Linkage for the purpose of hardware interfacing
class Leg_linkage:
    def __init__(self):
        self.a = 35.12 #mm
        self.b = 37.6 #mm
        self.c = 43 #mm
        self.d = 35.23  #mm
        self.e = 67.1 #mm
        self.f = 130 #mm  #new will be 130.0
        self.g = 37 #mm
        self.h = 43 #mm
        self.upper_leg_length = 0.130*1000
        self.lower_leg_length = 0.13813664159*1000
        self.lower_leg_bend_angle = radians(15) # degrees found on CAD
        self.i = self.upper_leg_length
        self.hip_width = 0.05162024721 * 1000
        self.gamma = atan(28.80/20.20)
        self.EDC = acos((self.c**2+self.h**2-self.e**2)/(2*self.c*self.h))


class ControlCmd:
    """Low-level control of Dynamixel motors."""

    def __init__(self):
        self.setup_dynamixel()
        self.setup_motors()
        self.initialize_motor_states()
        self.walking_freq = 2000  # Hz

    def setup_dynamixel(self):
        pangolin_dynamixel = PangolinDynamixel()
        self.dynamixel = DXL_Communication(pangolin_dynamixel.DEVICE_NAME, pangolin_dynamixel.B_RATE)
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

    def motor_position_control(self, position=None):
        if position is None:
            position = [[   0   , 0  ,  0  ,  0],
                        [2823, 2258, 2207 ,2930],
                        [2070 ,3071, 3049 ,2127]]

        for i, motor_list in enumerate(self.leg_motor_list[1:], start=1):
            for j, motor in enumerate(motor_list):
                # print(int(position[i][j]))
                motor.writePosition(int(position[i][j]))

        self.dynamixel.sentAllCmd()
        #time.sleep(1 / self.walking_freq)


if __name__ == "__main__":
    robot_control = RobotControl()
    command_dict = {
        "joy":robot_control.loop,
        "read":robot_control.control_cmd.read_all_motor_data,
        "disable":robot_control.control_cmd.disable_all_motor,
        "move":robot_control.control_cmd.motor_position_control,
        "reset":robot_control.reset_to_orginal,
        "loop":robot_control.loop,
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