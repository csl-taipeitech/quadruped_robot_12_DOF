import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import sys

sys.path.append('/home/csl/ros_triceratop/src/triceratops_base')

from Triceratops_ControlCmd import RobotControl

class PingPongPrimitiveSubscriber(Node):
    def __init__(self):
        super().__init__('pingpong_primitive_subscriber')
        self.control_cmd = RobotControl()
        self.subscription = self.create_subscription(
            String,
            'pong/primitive',
            self.listener_callback,
            10
        )
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()
        
        self.linear_speed = 0.5
        self.angular_speed = 1.0

    def listener_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: "{command}"')

        # 根據指令更新速度
        if command == "Forward":
            self.twist.linear.x = self.linear_speed
            self.twist.angular.z = 0.0
            self.control_cmd.start_gait('forward')
        elif command == "Backward":
            self.twist.linear.x = -self.linear_speed
            self.twist.angular.z = 0.0
            self.control_cmd.start_gait('backward')
        elif command == "Left":
            self.twist.linear.x = 0.0
            self.twist.angular.z = self.angular_speed
            self.control_cmd.start_gait('turn_left')
        elif command == "Right":
            self.twist.linear.x = 0.0
            self.twist.angular.z = -self.angular_speed
            self.control_cmd.start_gait('turn_right')
        elif command == "Pause":
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.control_cmd.stop_gait()
        elif command == "Accelerate Forward/Backward":
            self.twist.linear.x *= 1.1
        elif command == "Decelerate Forward/Backward":
            self.twist.linear.x *= 0.9
        elif command == "Accelerate Left/Right":
            self.twist.angular.z *= 1.1
        elif command == "Decelerate Left/Right":
            self.twist.angular.z *= 0.9
        else:
            self.get_logger().warn(f'Unknown command: "{command}"')
            return
        
        # 發布更新後的速度
        self.control_cmd.set_velocity(self.twist)
        self.publisher.publish(self.twist)
        self.get_logger().info(f'Published linear: {self.twist.linear.x}, angular: {self.twist.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    node = PingPongPrimitiveSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
