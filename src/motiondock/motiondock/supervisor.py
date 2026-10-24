"""Single authorized output gate for manual and autonomous base velocity."""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class MotionDockSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("motiondock_supervisor")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("manual_input_topic", "/motiondock/cmd_vel_manual")
        self.declare_parameter("autonomous_input_topic", "/motiondock/cmd_vel_autonomous")
        self.declare_parameter("mode_topic", "/motiondock/mode")
        self.declare_parameter("max_linear_velocity", 0.3)
        self.declare_parameter("max_angular_velocity", 0.8)
        self.declare_parameter("command_timeout", 0.35)

        output_topic = self.get_parameter("output_topic").value
        manual_topic = self.get_parameter("manual_input_topic").value
        autonomous_topic = self.get_parameter("autonomous_input_topic").value
        mode_topic = self.get_parameter("mode_topic").value
        if output_topic in (manual_topic, autonomous_topic, mode_topic):
            raise ValueError("The authorized output topic must be separate from all input and mode topics")
        self.max_linear = abs(float(self.get_parameter("max_linear_velocity").value))
        self.max_angular = abs(float(self.get_parameter("max_angular_velocity").value))
        self.command_timeout = max(0.05, float(self.get_parameter("command_timeout").value))

        self._mode = "stop"
        self._last_mode_update = 0.0
        self._last_input = 0.0
        self._command = Twist()
        self._output = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(String, mode_topic, self._on_mode, 10)
        self.create_subscription(Twist, manual_topic, self._on_manual, 10)
        self.create_subscription(Twist, autonomous_topic, self._on_autonomous, 10)
        self.create_timer(0.05, self._publish_authorized_command)
        self.get_logger().info(
            f"Velocity gate active: {manual_topic} (manual), {autonomous_topic} (autonomous) -> {output_topic}"
        )

    def _on_mode(self, message: String) -> None:
        if message.data not in ("manual", "autonomous", "stop"):
            return
        self._last_mode_update = time.monotonic()
        if self._mode != message.data:
            self._mode = message.data
            self._command = Twist()
            self._last_input = 0.0
            self._output.publish(Twist())
            self.get_logger().info(f"Control mode: {self._mode}")

    def _clamp(self, message: Twist) -> Twist:
        result = Twist()
        result.linear.x = max(-self.max_linear, min(self.max_linear, float(message.linear.x)))
        result.angular.z = max(-self.max_angular, min(self.max_angular, float(message.angular.z)))
        return result

    def _accept(self, message: Twist, required_mode: str) -> None:
        if self._mode != required_mode:
            return
        self._command = self._clamp(message)
        self._last_input = time.monotonic()

    def _on_manual(self, message: Twist) -> None:
        self._accept(message, "manual")

    def _on_autonomous(self, message: Twist) -> None:
        self._accept(message, "autonomous")

    def _publish_authorized_command(self) -> None:
        if self._last_mode_update and time.monotonic() - self._last_mode_update > self.command_timeout:
            if self._mode != "stop":
                self._mode = "stop"
                self._last_input = 0.0
                self._command = Twist()
                self.get_logger().warning("GUI heartbeat timed out; switching velocity gate to STOP")
        if self._mode == "stop" or not self._last_input or time.monotonic() - self._last_input > self.command_timeout:
            self._command = Twist()
        self._output.publish(self._command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionDockSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._output.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
