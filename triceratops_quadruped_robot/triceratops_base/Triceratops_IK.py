import numpy as np
from transforms3d.euler import euler2mat
from math import pi

class InverseKinematics():
    def __init__(self, config):
        self.config = config
            
    def leg_explicit_inverse_kinematics(self, r_body_foot, leg_index):
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

        if leg_index == 2 or leg_index == 3:
            x = -x
            y = -y

        d = (y ** 2 + z ** 2) ** 0.5
        l = (d ** 2 - self.config.ABDUCTION_OFFSET ** 2) ** 0.5

        gamma1 = - np.arctan( self.config.ABDUCTION_OFFSETS[leg_index] / l )
        gamma2 = - np.arctan( y / z )
        gamma = gamma2 + gamma1
        if leg_index == 2 or leg_index == 3:
            gamma = gamma2 - gamma1
        else:
            gamma = gamma2 + gamma1

        s = (l ** 2 + x ** 2) ** 0.5
        n = (s ** 2 - self.config.upper_leg_length ** 2 - self.config.lower_leg_length ** 2) / (
            2 * self.config.upper_leg_length)
        
        beta = np.arccos( n / self.config.lower_leg_length)

        alpha1 = - np.arctan( x / l )
        alpha2 = np.arccos( (self.config.upper_leg_length + n) / s)
        
        alpha = alpha2 + alpha1

        hip_angle = pi/2 - alpha

        return np.array([gamma, hip_angle, beta])
    
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

    def four_legs_inverse_kinematics(self, r_body_foot):
        """Find the joint angles for all twelve DOF correspoinding to the given matrix of body-relative foot positions.
        
        Parameters
        ----------
        r_body_foot : numpy array (3,4)
            Matrix of the body-frame foot positions. Each column corresponds to a separate foot.
        config : Config object
            Object of robot configuration parameters.
        
        Returns
        -------
        numpy array (3,4)
            Matrix of corresponding joint angles.
        """
        alpha = np.zeros((3, 4))
        for i in range(4):
            body_offset = self.config.LEG_ORIGINS[:, i]
            alpha[:, i] = self.leg_explicit_inverse_kinematics(
                r_body_foot[:, i] - body_offset, i,
            )
            alpha[2, i] = self.lower_leg_angle_to_servo_angle(alpha[1, i], alpha[2, i])

        alpha = np.array([[   alpha[0, 0],    - alpha[0, 1],   - alpha[0, 2],     alpha[0, 3]],
                          [   alpha[1, 0],    - alpha[1, 1],   - alpha[1, 2],     alpha[1, 3]],
                          [   alpha[2, 0],    - alpha[2, 1],   - alpha[2, 2],     alpha[2, 3]]])

        return alpha