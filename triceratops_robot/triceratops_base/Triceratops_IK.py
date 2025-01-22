from math import asin,acos,atan,pi,sqrt, radians, degrees
from Triceratops_Config import RobotConfiguration, state, Leg_linkage
import numpy as np
import time

class RobotIK():
    def __init__(self):
        self.config = RobotConfiguration()
        self.linkage = Leg_linkage()
        self.upper_leg_length = self.config.upper_leg_length
        self.lower_leg_length = self.config.lower_leg_length
        
    def leg_inverse_kinematics(self, foot_locations):

        leg_1_x, leg_2_x, leg_3_x, leg_4_x = foot_locations[0, 0], foot_locations[0, 1], foot_locations[0, 2], foot_locations[0, 3]
        leg_1_y, leg_2_y, leg_3_y, leg_4_y = foot_locations[2, 0], foot_locations[2, 1], foot_locations[2, 2], foot_locations[2, 3]
        #Leg1
        leg_1_x = -leg_1_x
        lower_leg_1 = pi - acos((leg_1_x**2 + leg_1_y**2 - self.upper_leg_length**2 - self.lower_leg_length**2) / (-2 * self.upper_leg_length * self.lower_leg_length))
        phi_1 = acos((self.upper_leg_length**2 + leg_1_x**2 + leg_1_y**2 - self.lower_leg_length**2) / (2 * self.upper_leg_length * sqrt(leg_1_x**2 + leg_1_y**2)))

        if leg_1_x > 0:
            upper_leg_1 = (abs(atan(leg_1_y / - leg_1_x)) - phi_1)
        elif leg_1_x < 0:
            upper_leg_1 = (pi - abs(atan(leg_1_y /leg_1_x)) - phi_1)
        else:
            upper_leg_1 = (pi - 1.5707 - phi_1)

        THETA01 = self.lower_leg_angle_to_servo_angle(upper_leg_1, lower_leg_1)

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
        #print("THETA02",THETA02)

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

        joint_angles = np.array([[THETA01, - THETA02, - THETA03, THETA04],
                [upper_leg_1, -upper_leg_2, - upper_leg_3, upper_leg_4],
                [0., 0., 0., 0.]])
        # joint_angles = np.array([[- THETA01, THETA02, THETA03, - THETA04],
        #                 [- upper_leg_1, upper_leg_2, upper_leg_3, - upper_leg_4],
        #                 [0., 0., 0., 0.]])

        #print("joint_angles",joint_angles/3.14*180)
        state.joint_angles = joint_angles
        return joint_angles
    
    def leg_explicit_inverse_kinematics(r_body_foot, leg_index, config):
        """Find the joint angles corresponding to the given body-relative foot position for a given leg and configuration
        
        Parameters
        ----------
        r_body_foot : [type]
            [description]
        leg_index : [type]
            [description]
        config : [type]
            [description]
        
        Returns
        -------
        numpy array (3)
            Array of corresponding joint angles.
        """
        (x, y, z) = r_body_foot

        # Distance from the leg origin to the foot, projected into the y-z plane
        R_body_foot_yz = (y ** 2 + z ** 2) ** 0.5

        # Distance from the leg's forward/back point of rotation to the foot
        R_hip_foot_yz = (R_body_foot_yz ** 2 - config.ABDUCTION_OFFSET ** 2) ** 0.5

        # Interior angle of the right triangle formed in the y-z plane by the leg that is coincident to the ab/adduction axis
        # For feet 2 (front left) and 4 (back left), the abduction offset is positive, for the right feet, the abduction offset is negative.
        arccos_argument = config.ABDUCTION_OFFSETS[leg_index] / R_body_foot_yz
        arccos_argument = np.clip(arccos_argument, -0.99, 0.99)
        phi = np.arccos(arccos_argument)

        # Angle of the y-z projection of the hip-to-foot vector, relative to the positive y-axis
        hip_foot_angle = np.arctan2(z, y)

        # Ab/adduction angle, relative to the positive y-axis
        abduction_angle = phi + hip_foot_angle

        # theta: Angle between the tilted negative z-axis and the hip-to-foot vector
        theta = np.arctan2(-x, R_hip_foot_yz)

        # Distance between the hip and foot
        R_hip_foot = (R_hip_foot_yz ** 2 + x ** 2) ** 0.5

        # Angle between the line going from hip to foot and the link L1
        arccos_argument = (config.LEG_L1 ** 2 + R_hip_foot ** 2 - config.LEG_L2 ** 2) / (
            2 * config.LEG_L1 * R_hip_foot
        )
        arccos_argument = np.clip(arccos_argument, -0.99, 0.99)
        trident = np.arccos(arccos_argument)

        # Angle of the first link relative to the tilted negative z axis
        hip_angle = theta + trident

        # Angle between the leg links L1 and L2
        arccos_argument = (config.LEG_L1 ** 2 + config.LEG_L2 ** 2 - R_hip_foot ** 2) / (
            2 * config.LEG_L1 * config.LEG_L2
        )
        arccos_argument = np.clip(arccos_argument, -0.99, 0.99)
        beta = np.arccos(arccos_argument)

        # Angle of the second link relative to the tilted negative z axis
        knee_angle = hip_angle - (np.pi - beta)

        return np.array([abduction_angle, hip_angle, knee_angle])

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
        
        THETA0 = 2*pi - 0.75*pi -THETA3 -THETA2
        THETA0 = pi/2- THETA0

        return THETA0
        
        
if __name__ == "__main__":
    # test code 
    Triceratop_IK = RobotIK()

    foot_locations = np.array([[   20,        20,       -20,         -20],
                            [       0,      -100,       -100,       -100],
                            [    -100,      -100,       -100,       -100]])
    joint_angles = Triceratop_IK.leg_inverse_kinematics(foot_locations)*180/3.14

    time.sleep(0.2)

    print(joint_angles[1, 0], joint_angles[2, 0])
    print(joint_angles[1, 1], joint_angles[2, 1])
    print(joint_angles[1, 2], joint_angles[2, 2])
    print(joint_angles[1, 3], joint_angles[2, 3])

    pass