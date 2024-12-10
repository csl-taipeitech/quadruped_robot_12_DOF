#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',  # topic name
            self.imu_callback,
            10)  # QoS profile depth
        self.subscription  # prevent unused variable warning
        self.get_logger().info('IMU Subscriber has been started')

    def imu_callback(self, msg):
        # Extract orientation quaternion
        orientation = msg.orientation
        # Extract angular velocity
        angular_vel = msg.angular_velocity
        # Extract linear acceleration
        linear_acc = msg.linear_acceleration
        
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll = math.atan2(2.0 * (orientation.w * orientation.x + orientation.y * orientation.z),
                         1.0 - 2.0 * (orientation.x * orientation.x + orientation.y * orientation.y))
        pitch = math.asin(2.0 * (orientation.w * orientation.y - orientation.z * orientation.x))
        yaw = math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))

        # Convert radians to degrees
        roll_rad = roll -0.01
        pitch_rad = pitch -0.01
        yaw_rad = yaw

        # Log the data
        self.get_logger().info(
            f'\nOrientation (deg) - Roll: {roll_rad:.2f}, Pitch: {pitch_rad:.2f}, Yaw: {yaw_rad:.2f}\n'
            f'Angular Velocity (rad/s) - X: {angular_vel.x:.4f}, Y: {angular_vel.y:.4f}, Z: {angular_vel.z:.4f}\n'
            f'Linear Acceleration (m/s²) - X: {linear_acc.x:.4f}, Y: {linear_acc.y:.4f}, Z: {linear_acc.z:.4f}'
        )

def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = ImuSubscriber()
    
    try:
        rclpy.spin(imu_subscriber)
    except KeyboardInterrupt:
        print('Stopped by keyboard interrupt')
    finally:
        # Cleanup
        imu_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()