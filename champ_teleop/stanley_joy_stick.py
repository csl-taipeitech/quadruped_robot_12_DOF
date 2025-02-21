import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose, Twist
from Triceratops_ControlCmd import ControlCmd, RobotControl

class BodyPosePublisher(Node):
    """Publishes body pose messages based on joystick input."""
    def __init__(self):
        super().__init__('body_pose_publisher')
        self.pose_pub = self.create_publisher(Pose, '/body_pose', 10)
        
        self.body_z = 0.0  # Default height
        self.body_pitch = 0.0  # Default pitch
        self.body_roll = 0.0  # Default roll

        self.prev_body_z = None
        self.prev_body_pitch = 0.0
        self.prev_body_roll = 0.0
        
        self.prev_button_6 = 0 
        self.prev_button_8 = 0
        self.motion_active = False # Handle the conflict between the body pose and the handshake pose.

        self.get_logger().info("Body Pose Publisher Initialized.")
        
    def update_body_pose(self, data: Joy):
        """Updates body pose based on joystick axes input."""
        if self.motion_active:
            self.get_logger().info("Motion active, ignoring other inputs")
            return

        self.adjust_body_height(data)
        self.adjust_body_tilt(data)
        self.publish_body_pose()
    
    def adjust_body_height(self, data):
        """Adjusts body height based on joystick D-pad input."""
        if len(data.axes) > 7:
            if data.axes[7] == 1:
                self.body_z = min(0.0, self.body_z + 0.0015)
            elif data.axes[7] == -1:
                self.body_z = max(-0.04, self.body_z - 0.0015)
    
    def adjust_body_tilt(self, data):
        """Handles leaning and bowing based on button presses."""
        if len(data.axes) > 6:
            self.body_roll = 0.1 if data.axes[6] == 1 else (-0.1 if data.axes[6] == -1 else 0.0)
        
        if data.buttons[6]:
            self.store_previous_pose()
            self.body_pitch = -0.1
            self.body_z = -0.02

        elif data.buttons[8]:
            self.store_previous_pose()
            self.body_pitch = 0.1
            self.body_z = -0.02
        
        if self.prev_button_6 and not data.buttons[6]:
            self.restore_previous_pose()
        if self.prev_button_8 and not data.buttons[8]:
            self.restore_previous_pose()
        
        self.prev_button_6 = data.buttons[6]
        self.prev_button_8 = data.buttons[8]

    def store_previous_pose(self):
        """Stores the current pose before modification."""
        if self.prev_body_z is None:
            self.prev_body_z = self.body_z
            self.prev_body_pitch = self.body_pitch
            self.prev_body_roll = self.body_roll
    
    def restore_previous_pose(self):
        """Restores the last stored pose when buttons are released."""
        if self.prev_body_z is not None:
            self.body_z = self.prev_body_z
        self.body_pitch = self.prev_body_pitch
        self.body_roll = self.prev_body_roll
        self.prev_body_z = None 
        self.publish_body_pose()
    
    def publish_body_pose(self):
        """Publishes the current body pose."""
        pose_msg = Pose()
        pose_msg.position.z = self.body_z
        pose_msg.orientation.x = self.body_roll
        pose_msg.orientation.y = self.body_pitch
        pose_msg.orientation.w = 1.0
        self.pose_pub.publish(pose_msg)

class JoystickListener(Node):
    """Listens to joystick input and controls robot movement."""
    def __init__(self, control_cmd, robot_control, body_pose_publisher):
        super().__init__('joystick_listener')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.control_cmd = control_cmd
        self.robot_control = robot_control
        self.body_pose_publisher = body_pose_publisher

        self.linear_x_scale = 0.1
        self.linear_y_scale = 0.1
        self.angular_scale = 1.5
        self.prev_button_7 = 0 

        self.get_logger().info("Joystick Listener Initialized.")

    def joy_callback(self, data: Joy):
        """Handles joystick input and updates robot movement."""
        if len(data.buttons) > 11 and data.buttons[11]:
            self.get_logger().info("Button 11 Pressed → Reset and Start Gait")
            self.robot_control.stop_gait()
            time.sleep(1)
            self.control_cmd.reset_to_original()
            time.sleep(1)
            self.robot_control.start_gait()
            time.sleep(1)
        
        vel = Twist()
        vel.linear.x = data.axes[1] * self.linear_x_scale if len(data.axes) > 1 and abs(data.axes[1]) > 0.1 else 0.0
        vel.linear.y = data.axes[0] * self.linear_y_scale if len(data.axes) > 0 and abs(data.axes[0]) > 0.1 else 0.0
        vel.angular.z = data.axes[2] * self.angular_scale if len(data.axes) > 2 and abs(data.axes[2]) > 0.1 else 0.0
        
        self.cmd_pub.publish(vel)
        self.get_logger().info(f"Publishing cmd_vel: {vel}")
        
        self.body_pose_publisher.update_body_pose(data)

        if data.buttons[7] and not self.prev_button_7:
            self.handle_handshake()
        self.prev_button_7 = data.buttons[7]

    def handle_handshake(self):
        """Executes the handshake movement."""
        self.get_logger().info("Starting Handshake Motion...")
        self.body_pose_publisher.motion_active = True  # Stop pose updates
        self.body_pose_publisher.publish_body_pose()
        time.sleep(0.5)
        
        self.robot_control.stop_gait()
        time.sleep(1)
        self.control_cmd.reset_to_original()
        time.sleep(1)
        
        fl_lower_target = 1600
        fl_upper_target = 1300
        fl_bottom_target = 2048
        
        for i in range(20):
            progress = i / 10.0
            current_lower = int(2048 + progress * (fl_lower_target - 2048))
            current_upper = int(2048 + progress * (fl_upper_target - 2048))
            position = [[1989, current_lower, 2048, 2001],
                        [2541, current_upper, 2100, 2100],
                        [2048, fl_bottom_target, 2048, 2048]]
            self.control_cmd.motor_position_control(position)
        time.sleep(2)
        
        self.control_cmd.reset_to_original()
        time.sleep(1)
        self.robot_control.start_gait()
        time.sleep(1)
        
        self.get_logger().info("Restoring body pose after handshake...")
        self.body_pose_publisher.motion_active = False  # Resume pose updates
        self.body_pose_publisher.restore_previous_pose()

def main():
    """Initializes ROS2 nodes and starts joystick listener."""
    rclpy.init()
    
    control_cmd = ControlCmd()
    robot_control = RobotControl()
    body_pose_publisher = BodyPosePublisher()
    joystick_listener = JoystickListener(control_cmd, robot_control, body_pose_publisher)
    
    control_cmd.enable_all_motor()
    
    try:
        rclpy.spin(joystick_listener)
    except KeyboardInterrupt:
        print("Shutting down...")
    
    robot_control.stop_gait()
    control_cmd.disable_all_motor()
    robot_control.stop_executor()
    body_pose_publisher.destroy_node()
    joystick_listener.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
