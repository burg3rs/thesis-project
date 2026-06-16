#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class CmdVelToAckermann(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann')

        self.wheelbase = 0.22
        self.max_steer = 0.838
        self.speed_scale = 5.0
        self.timeout = 0.35

        self.last_cmd_time = self.get_clock().now()
        self.last_v = 0.0
        self.last_wz = 0.0

        self.rear_pub = self.create_publisher(
            Float64MultiArray,
            '/rear_velocity_controller/commands',
            10
        )

        self.steer_pub = self.create_publisher(
            Float64MultiArray,
            '/steering_controller/commands',
            10
        )

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.create_timer(0.05, self.publish_commands)  # 20 Hz

        self.get_logger().info('cmd_vel to Ackermann bridge started')

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()
        self.last_v = msg.linear.x
        self.last_wz = msg.angular.z

    def publish_commands(self):
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9

        if dt > self.timeout:
            v = 0.0
            wz = 0.0
        else:
            v = self.last_v
            wz = self.last_wz

        rear_speed = v * self.speed_scale

        if abs(v) < 1e-4 or abs(wz) < 1e-4:
            steer_angle = 0.0
        else:
            steer_angle = math.atan((self.wheelbase * wz) / v)

        steer_angle = max(min(steer_angle, self.max_steer), -self.max_steer)
        if abs(steer_angle) < 0.03:
            steer_angle = 0.0

        rear_cmd = Float64MultiArray()
        rear_cmd.data = [rear_speed, rear_speed]

        steer_cmd = Float64MultiArray()
        steer_cmd.data = [steer_angle]

        self.rear_pub.publish(rear_cmd)
        self.steer_pub.publish(steer_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToAckermann()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
