
from Triceratops_Config import ServoParams, PWMParams

class ServoPWM:
    def __init__(self):
        self.pwm_params = PWMParams()
        self.servo_params = ServoParams()
        self.initialize_pwm()

    def set_actuator_positions(self, joint_angles):
        self.send_servo_commands(joint_angles)
    
    def set_actuator_position(self, joint_angle, axis, leg):
        self.send_servo_command(joint_angle, axis, leg)

    def deactivate_servos(self):
        for leg_index in range(4):
            for axis_index in range(3):
                pass

    def pwm_to_duty_cycle(self, pulsewidth_micros):
        """Converts a pwm signal (measured in microseconds) to a corresponding duty cycle on the gpio pwm pin

        Parameters
        ----------
        pulsewidth_micros : float
            Width of the pwm signal in microseconds
        pwm_params : PWMParams
            PWMParams object

        Returns
        -------
        float
            PWM duty cycle corresponding to the pulse width
        """
        return int(pulsewidth_micros / 1e6 * self.pwm_params.freq * self.pwm_params.range)


    def angle_to_pwm(self, angle, axis_index, leg_index):
        """Converts a desired servo angle into the corresponding PWM command

        Parameters
        ----------
        angle : float
            Desired servo angle, relative to the vertical (z) axis
        servo_params : ServoParams
            ServoParams object
        axis_index : int
            Specifies which joint of leg to control. 0 is abduction servo, 1 is inner hip servo, 2 is outer hip servo.
        leg_index : int
            Specifies which leg to control. 0 is front-right, 1 is front-left, 2 is back-right, 3 is back-left.

        Returns
        -------
        float
            PWM width in microseconds
        """
        angle_deviation = (
            angle - self.servo_params.neutral_angles[axis_index, leg_index]
        ) * self.servo_params.servo_multipliers[axis_index, leg_index]
        pulse_width_micros = (
            self.servo_params.neutral_position_pwm
            + self.servo_params.micros_per_rad * angle_deviation
        )
        # print(pulse_width_micros)
        return pulse_width_micros


    def angle_to_duty_cycle(self, angle, axis_index, leg_index):
        return self.pwm_to_duty_cycle(
            self.angle_to_pwm(angle, axis_index, leg_index)
        )


    def initialize_pwm(self):
        for leg_index in range(4):
            for axis_index in range(3):
                pass


    def send_servo_commands(self, joint_angles):
        for leg_index in range(4):
            for axis_index in range(3):
                duty_cycle = self.angle_to_duty_cycle(
                    joint_angles[axis_index, leg_index],
                    axis_index,
                    leg_index,
                )


    def send_servo_command(self, joint_angle, axis, leg):
        duty_cycle = self.angle_to_duty_cycle(joint_angle, axis, leg)





if __name__ == "__main__":
    pass

    # test code
    import time 
    from math import pi
    import numpy as np
    servo_PWM = ServoPWM()
    joint_angles = np.array([   [  0.,  0.,  0.,  0.],
                                [ 1.57, 1.57, pi/4., pi/4.],
                                [-pi/8.,-pi/8.,-pi/8.,-pi/8.]]
                            )
    # pulse_width_micros = servo_PWM.angle_to_pwm(pi/2, 1, 1)
    # print(pulse_width_micros)
    angle = 100 * pi /180
    servo_PWM.send_servo_command(angle, 2, 3)
    # servo_PWM.set_actuator_positions(joint_angles)
    time.sleep(0.2)

    # joint_angles = np.array([   [  0.,  0.,  0.,  0.],
    #                             [ pi/4., pi/4., pi/4., pi/4.],
    #                             [-pi/4.,-pi/4.,-pi/4.,-pi/4.]]
    #                         )
    
    # servo_PWM.set_actuator_positions(joint_angles)
    # time.sleep(0.2)

    # joint_angles = np.array([   [  0.,  0.,  0.,  0.],
    #                             [ pi/2., pi/2., pi/2., pi/2.],
    #                             [-pi/2.,-pi/2.,-pi/2.,-pi/2.]]
    #                         )
    
    # servo_PWM.set_actuator_positions(joint_angles)
    # time.sleep(0.2)

    # joint_angles = np.array([   [  0.,  0.,  0.,  0.],
    #                             [ pi/4., pi/4., pi/4., pi/4.],
    #                             [-pi/4.,-pi/4.,-pi/4.,-pi/4.]]
    #                         )
    
    # servo_PWM.set_actuator_positions(joint_angles)
    # time.sleep(0.2)

    # servo_PWM.deactivate_servos()
    # time.sleep(1)