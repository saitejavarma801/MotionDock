"""ROS 2 bridge used by the GUI to publish mode and manual joystick commands."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class MotionDockRosBridge:
    def __init__(self) -> None:
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init(args=None)
        self.node = Node("motiondock_gui")
        self._mode_pub = self.node.create_publisher(String, "/motiondock/mode", 10)
        self._manual_pub = self.node.create_publisher(Twist, "/motiondock/cmd_vel_manual", 10)

    def publish(self, mode: str, linear: float = 0.0, angular: float = 0.0) -> None:
        mode_message = String()
        mode_message.data = mode
        self._mode_pub.publish(mode_message)
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self._manual_pub.publish(twist)

    def close(self) -> None:
        try:
            self.publish("stop", 0.0, 0.0)
        finally:
            self.node.destroy_node()
            if self._owns_context and rclpy.ok():
                rclpy.shutdown()
