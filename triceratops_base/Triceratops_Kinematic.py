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
        
        joint_angles = np.array([[0., 0., 0., 0.],
                        [- upper_leg_1, upper_leg_2, upper_leg_3, - upper_leg_4],
                        [- THETA01, THETA02, THETA03, - THETA04]])

        #print("joint_angles",joint_angles/3.14*180)
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