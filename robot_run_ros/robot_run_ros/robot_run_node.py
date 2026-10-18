"""Connect Robot Run to ROS 2 topics.

Created by Elena Villalba for Intelligent Robotics, UPC-EEBE.
The graphical game runs in the main thread while ROS 2 callbacks are processed
in a background thread. Students interact only through /scan and /cmd_vel.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import LaserScan

from robot_arcade import FORWARD, RobotGame, STOP, TURN_LEFT, TURN_RIGHT


class RobotRunNode(Node):
    """Publish virtual ranges and translate velocity commands into game actions."""

    def __init__(self) -> None:
        super().__init__("robot_run")

        self.declare_parameter("robot_name", "diff_robot")
        self.declare_parameter("team_number", 0)
        self.declare_parameter("response_timeout", 0.25)

        self.robot_name = str(self.get_parameter("robot_name").value)
        self.team_number = int(self.get_parameter("team_number").value)
        self.response_timeout = float(
            self.get_parameter("response_timeout").value
        )

        self.scan_publisher = self.create_publisher(LaserScan, "/scan", 10)
        self.velocity_subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self._velocity_callback,
            10,
        )

        self._condition = threading.Condition()
        self._command_sequence = 0
        self._latest_action = STOP
        self._last_timeout_warning = 0.0

        self.get_logger().info(
            f"Robot Run ready for Team {self.team_number:02d} "
            f"({self.robot_name})"
        )

    def _velocity_callback(self, message: Twist) -> None:
        """Convert the latest Twist command into a Robot Run action."""
        linear = message.linear.x
        angular = message.angular.z

        if abs(linear) < 1e-3 and abs(angular) < 1e-3:
            action = STOP
        elif angular > 1e-3:
            action = TURN_LEFT
        elif angular < -1e-3:
            action = TURN_RIGHT
        elif linear > 1e-3:
            action = FORWARD
        else:
            action = STOP

        with self._condition:
            self._latest_action = action
            self._command_sequence += 1
            self._condition.notify_all()

    def _publish_scan(self, scan) -> None:
        """Publish [left, front, right] as a simplified LaserScan message."""
        message = LaserScan()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "virtual_ultrasonic_array"

        # The virtual array is ordered from left to right. A negative angular
        # increment preserves that order in the LaserScan message.
        message.angle_min = math.pi / 4.0
        message.angle_max = -math.pi / 4.0
        message.angle_increment = -math.pi / 4.0
        message.time_increment = 0.0
        message.scan_time = 0.0
        message.range_min = 0.0
        message.range_max = 8.0
        message.ranges = [float(distance) for distance in scan.ranges]
        self.scan_publisher.publish(message)

    def request_action(self, scan) -> str:
        """Publish a scan and wait briefly for a new student command."""
        with self._condition:
            previous_sequence = self._command_sequence

        self._publish_scan(scan)

        with self._condition:
            received = self._condition.wait_for(
                lambda: self._command_sequence > previous_sequence,
                timeout=self.response_timeout,
            )
            action = self._latest_action if received else STOP

        if not received:
            now = time.monotonic()
            if now - self._last_timeout_warning > 2.0:
                self.get_logger().warning(
                    "No new /cmd_vel command received. Robot Run will STOP."
                )
                self._last_timeout_warning = now

        return action


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotRunNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    executor_thread = threading.Thread(
        target=executor.spin,
        daemon=True,
    )
    executor_thread.start()

    game = RobotGame(
        robot_name=node.robot_name,
        team_number=node.team_number,
        controller=node.request_action,
    )

    try:
        game.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        executor_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()

