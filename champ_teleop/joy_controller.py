#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from champ_msgs.msg import Pose as PoseLite
from geometry_msgs.msg import Pose

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

class Teleop(Node):
    def __init__(self):
        super().__init__('champ_teleop')

        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.pose_lite_publisher = self.create_publisher(PoseLite, 'body_pose/raw', 1)
        self.pose_publisher = self.create_publisher(Pose, 'body_pose', 1)
        
        self.joy_subscriber = self.create_subscription(Joy, 'joy', self.joy_callback, 1)

        self.declare_parameter("speed", 0.5)
        self.declare_parameter("turn", 1.0)

        self.speed = self.get_parameter("speed").value
        self.turn = self.get_parameter("turn").value

    def joy_callback(self, data):
        # print("data", data)
        if data is not None:
            if data.axes[1] > 0.5:
                linearX_speed = 1
            elif data.axes[1] < - 0.5:
                linearX_speed = -1
            else:
                linearX_speed = 0

            if data.axes[0] > 0.5:
                angularZ_speed = 1
            elif data.axes[0] < - 0.5:
                angularZ_speed = -1
            else:
                angularZ_speed = 0

        twist = Twist()
        twist.linear.x = linearX_speed * self.speed
        twist.linear.y = data.buttons[4] * angularZ_speed * self.speed
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angularZ_speed * self.turn
        print("twist", twist)

        self.velocity_publisher.publish(twist)

        # body_pose_lite = PoseLite()
        # body_pose_lite.x = float(0)  # Explicitly cast to float
        # body_pose_lite.y = float(0)  # Explicitly cast to float
        # body_pose_lite.roll = float((not data.buttons[5]) * -data.axes[3] * 0.349066)
        # body_pose_lite.pitch = float(data.axes[4] * 0.174533)
        # body_pose_lite.yaw = float(data.buttons[5] * data.axes[3] * 0.436332)

        # if data.axes[5] < 0:
        #     body_pose_lite.z = float(data.axes[5] * 0.5)  # Explicitly cast to float

        # self.pose_lite_publisher.publish(body_pose_lite)

        # body_pose = Pose()
        # body_pose.position.z = body_pose_lite.z

        # quaternion = quaternion_from_euler(
        #     body_pose_lite.roll, body_pose_lite.pitch, body_pose_lite.yaw
        # )
        # body_pose.orientation.x = quaternion[0]
        # body_pose.orientation.y = quaternion[1]
        # body_pose.orientation.z = quaternion[2]
        # body_pose.orientation.w = quaternion[3]

        # self.pose_publisher.publish(body_pose)

if __name__ == "__main__":
    rclpy.init()
    teleop = Teleop()
    rclpy.spin(teleop)
    teleop.destroy_node()
    rclpy.shutdown()
