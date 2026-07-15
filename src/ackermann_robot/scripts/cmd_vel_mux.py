#!/usr/bin/env python3
"""
cmd_vel_mux.py — Switchable cmd_vel source multiplexer

Subscribes to two cmd_vel sources (DWB controller + NeuPAN planner) and
forwards the selected one to the ackermann_steering_controller as TwistStamped.

Switch between DWB and NeuPAN at runtime via ROS 2 parameter:
    ros2 param set /cmd_vel_mux active_planner neupan
    ros2 param set /cmd_vel_mux active_planner dwb

Usage:
    ros2 run ackermann_robot cmd_vel_mux.py
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')

        # Declare parameter for switching planner source
        self.declare_parameter('active_planner', 'dwb')

        # Pub to ackermann_steering_controller
        self.pub = self.create_publisher(
            TwistStamped, '/ackermann_steering_controller/reference', 10
        )

        # Sub to DWB controller (Nav2 default)
        self.dwb_sub = self.create_subscription(
            Twist, '/cmd_vel', self.dwb_callback, 10
        )

        # Sub to NeuPAN planner (separate topic, no conflict)
        self.neupan_sub = self.create_subscription(
            Twist, '/neupan_cmd_vel', self.neupan_callback, 10
        )

        # Latest messages from each source
        self.dwb_msg: Twist | None = None
        self.neupan_msg: Twist | None = None

        # Forwarding timer (20 Hz) — always publishes latest from active source
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        self.get_logger().info(
            'cmd_vel_mux started: DWB(/cmd_vel) + NeuPAN(/neupan_cmd_vel) '
            '→ /ackermann_steering_controller/reference'
        )
        self.get_logger().info(
            'Switch via: ros2 param set /cmd_vel_mux active_planner <dwb|neupan>'
        )

    def dwb_callback(self, msg: Twist):
        self.dwb_msg = msg

    def neupan_callback(self, msg: Twist):
        self.neupan_msg = msg

    def timer_callback(self):
        active = self.get_parameter('active_planner').get_parameter_value().string_value

        if active == 'neupan' and self.neupan_msg is not None:
            twist = self.neupan_msg
            source = 'neupan'
        elif self.dwb_msg is not None:
            twist = self.dwb_msg
            source = 'dwb'
        else:
            return  # No data yet

        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'base_link'
        ts.twist = twist
        self.pub.publish(ts)

        # Log active source changes (throttled to 10s)
        self.get_logger().debug(
            f'Active: {source} | v={twist.linear.x:.2f}, ω={twist.angular.z:.2f}',
            throttle_duration_sec=10.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
