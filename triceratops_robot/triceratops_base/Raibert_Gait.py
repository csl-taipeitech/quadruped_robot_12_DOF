import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos
from transforms3d.euler import euler2mat, quat2euler

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.previous_error = np.zeros(2)
        self.integral = np.zeros(2)
        
    def run(self, roll, pitch, dt=0.01):
        """
        Compute PID control for roll and pitch
        
        Args:
            roll (float): Current roll angle
            pitch (float): Current pitch angle
            dt (float): Time step
        
        Returns:
            numpy.ndarray: Compensation values for roll and pitch
        """
        # Target is to maintain a level orientation (0, 0)
        errors = np.array([-roll, -pitch])
        
        # Proportional term
        P = self.kp * errors
        
        # Integral term
        self.integral += errors * dt
        I = self.ki * self.integral
        
        # Derivative term
        D = self.kd * (errors - self.previous_error) / dt
        
        # Total compensation
        compensation = P + I + D
        
        # Update previous error
        self.previous_error = errors
        
        return compensation

class RaibertController:
    def __init__(self, use_imu=True):
        # Robot parameters
        self.body_length = 160  # mm
        self.body_width = 105   # mm
        self.leg_length = 125   # mm
        self.step_height = 30  # mm
        
        # Control parameters
        self.stride_period = 0.5  # seconds
        self.duty_factor = 0.65   # ratio of stance phase
        self.desired_velocity = 80  # mm/s
        self.kp_balance = 0.2      # proportional gain for balance control

        # Initial state
        self.phase = 0.0
        self.body_position = np.array([0.0, 0.0, self.leg_length])
        self.body_velocity = np.array([self.desired_velocity, 0.0, 0.0])
        
        # IMU state variables
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.angular_velocity = np.zeros(3)
        self.linear_acceleration = np.zeros(3)

        # IMU and Tilt Compensation
        self.use_imu = use_imu
        self.pid_controller = PIDController(
            kp=0.4,   # Proportional gain
            ki=0.02,  # Integral gain
            kd=0.002  # Derivative gain
        )

        # Leg positions in body frame (FL, FR, BL, BR)
        self.default_leg_positions = np.array([
            [ 0 , self.body_width/2, -self.leg_length],   # Front Left
            [ 0 , -self.body_width/2, -self.leg_length],  # Front Right
            [ 0 , self.body_width/2, -self.leg_length],  # Back Left
            [ 0 , -self.body_width/2, -self.leg_length]  # Back Right
        ])
        
        # Gait phase offsets for trot gait
        self.phase_offsets = np.array([0.0, 0.5, 0.5, 0.0])


    def calculate_touchdown_location(self, leg_idx, phase):

        """Calculate Raibert touchdown location with IMU-based adjustments"""
        current_pos = self.default_leg_positions[leg_idx]
        # Basic Raibert balance components
        forward_touchdown = self.kp_balance * self.body_velocity[0] * self.stride_period/2
        balance_offset = 0.5 * self.body_velocity[0] * self.stride_period
        

        touchdown_pos = current_pos.copy()
        
        # Apply corrections based on leg position
        if leg_idx in [0, 1]:  # Front legs
            touchdown_pos[0] += forward_touchdown + balance_offset
        else:  # Back legs
            touchdown_pos[0] += forward_touchdown + balance_offset
        
        return touchdown_pos

    def swing_trajectory(self, start_pos, end_pos, phase):
        """Generate swing phase trajectory using cubic spline."""
        if phase < 0 or phase > 1:
            return start_pos

        # Calculate z trajectory (parabolic path)
        z = -4 * self.step_height * phase * (phase - 1)
        
        # Linear interpolation for x and y trajectory
        ratio = 0.5 * (1 - np.cos(np.pi * phase))  # Smooth interpolation
        pos = start_pos + (end_pos - start_pos) * ratio
        pos[2] += z  # Add the swing height (z-axis trajectory)
        return pos
    
    def stance_trajectory(self, touchdown_pos, liftoff_pos, phase):
        """Generate stance phase trajectory."""
        ratio = phase
        return touchdown_pos + (liftoff_pos - touchdown_pos) * ratio
    
    def get_leg_position(self, leg_idx, t):
        """Get leg position at time t with IMU compensation"""
        phase = (t / self.stride_period + self.phase_offsets[leg_idx]) % 1.0

        touchdown_pos = self.calculate_touchdown_location(leg_idx, phase)
        liftoff_pos = self.default_leg_positions[leg_idx].copy()
        
        if phase < self.duty_factor:  # Stance phase
            normalized_phase = phase / self.duty_factor
            return self.stance_trajectory(touchdown_pos, liftoff_pos, normalized_phase)
        else:  # Swing phase
            normalized_phase = (phase - self.duty_factor) / (1 - self.duty_factor)
            return self.swing_trajectory(liftoff_pos, touchdown_pos, normalized_phase)

    def get_foot_locations(self, t):
        """Get all foot locations at time t in the specified format."""
        # Get positions for all legs
        leg_positions = [self.get_leg_position(i, t) for i in range(4)]
        
        # Convert to numpy array and transpose to get x, y coordinates for all legs
        leg_positions = np.array(leg_positions)
        
        # # Create 2D foot locations array (x and z coordinates only)
        new_foot_locations_12 = np.array([
            [leg_positions[0, 0], leg_positions[1, 0]],  # x coordinates
            [0, 0],   
            [leg_positions[0, 2], leg_positions[1, 2]]     # z coordinates
        ])

        new_foot_locations_34 = np.array([
            [- leg_positions[2, 0], - leg_positions[3, 0]],  # x coordinates
            [0, 0],  
            [leg_positions[2, 2], leg_positions[3, 2]]     # z coordinates
        ])

        # Create the foot_locations array in the specified format
        # foot_locations = np.array([
        #     [leg_positions[0, 0], leg_positions[1, 0] , - leg_positions[2, 0] , - leg_positions[3, 0] ],    # [leg_1_x, leg_2_x, leg_3_x, leg_4_x]
        #     [0, 0, 0, 0],   # [-leg_1_y, -leg_2_y, -leg_3_y, -leg_4_y]
        #     [leg_positions[0, 2], leg_positions[1, 2] , leg_positions[2, 2] , leg_positions[3, 2] ]
        # ])
        # print("foot_locations",foot_locations)
        
        return new_foot_locations_12,new_foot_locations_34
    
    def visualize_gait(self, duration=1, time_steps=50):
        """Visualize the gait pattern over time."""
        times = np.linspace(0, duration, time_steps)
        
        # Store all foot locations for visualization
        all_foot_locations = []
        
        # Calculate and store foot locations for each time step
        for t in times:
            foot_locations = self.get_foot_locations(t)
            all_foot_locations.append(foot_locations)
            # Print the foot locations at each time step
            print(f"\nTime step {t:.3f}s:")
            print("foot_locations =")
            print(foot_locations)
        
        # Convert to numpy array for plotting
        all_foot_locations = np.array(all_foot_locations)
        
        # Create visualization
        fig = plt.figure(figsize=(10, 10))
        
        # Plot X-Z trajectory for each leg
        ax1 = fig.add_subplot(211)
        leg_names = ['Front Left', 'Front Right', 'Back Left', 'Back Right']
        ax1.plot(all_foot_locations[:, 0, 0],
                all_foot_locations[:, 1, 0],  # Using zeros for Z coordinate as per format
                label=leg_names[0])
        ax1.set_xlabel('X Position (mm)')
        ax1.set_ylabel('Y Position (mm)')
        ax1.set_title('Leg Trajectories (Side View)')
        ax1.grid(True)
        ax1.legend()
        
        # Plot X-Y foot positions
        ax2 = fig.add_subplot(212)
        ax2.plot(all_foot_locations[:, 0, 1],
                all_foot_locations[:, 1, 1],
                label=leg_names[1])
        ax2.set_xlabel('X Position (mm)')
        ax2.set_ylabel('Y Position (mm)')
        ax2.set_title('Leg Trajectories (Top View)')
        ax2.grid(True)
        ax2.legend()
        
        plt.tight_layout()
        return fig

# Create and run simulation
controller = RaibertController()
controller.visualize_gait()
plt.show()