#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

sys.path.append('/home/csl/ros_triceratop/src/triceratops_base')

from Triceratops_ControlCmd import RobotControl

class Triceratop(Node):
    def __init__(self):
        super().__init__('triceratop_control')
        self.control_cmd = RobotControl()
        
        # Subscribe to cmd_vel topic
        self.cmd_vel_subscriber_ = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)
        self.is_first_time = True

    # Callback for cmd_vel messages
    def cmd_vel_callback(self, msg):

        self.control_cmd.set_velocity(msg)

        if msg is not None and (round(msg.linear.x, 1) != 0 or round(msg.angular.z, 1) != 0):
            if round(msg.linear.x, 1) > 0:
                self.get_logger().info('Moving forward')
                self.control_cmd.start_gait('forward')
            elif round(msg.linear.x, 1) < 0:
                self.get_logger().info('Moving backward')
                self.control_cmd.start_gait('backward')
            elif round(msg.angular.z, 1) < 0:
                self.get_logger().info('Turning right')
                self.control_cmd.start_gait('turn_left')
            elif round(msg.angular.z, 1) > 0:
                self.get_logger().info('Turning left')
                self.control_cmd.start_gait('turn_right')
        else:
            self.get_logger().info('Stopping')
            self.control_cmd.stop_gait()


    # Destroy the node
    def destroy(self):
        self.cmd_vel_subscriber_.destroy()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    Triceratop_control = Triceratop()
    rclpy.spin(Triceratop_control)
    Triceratop_control.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
