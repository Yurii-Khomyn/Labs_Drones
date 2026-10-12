import math
import time
from typing import TYPE_CHECKING
from geometry_msgs.msg import TwistStamped
from ..flight_state import FlightState
from .handler import FlightStateHandler

if TYPE_CHECKING:
    from ..flight_test_node import FlightTestNode


class FlyingToTargetHandler(FlightStateHandler):
    def handle(self, node: "FlightTestNode") -> None:
        if node.current_pose is None or node.start_position is None:
            return

        target_x = node.start_position[0] - node.TARGET_DX
        target_y = node.start_position[1] - node.TARGET_DY

        current_x = node.current_pose.pose.position.x
        current_y = node.current_pose.pose.position.y

        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance <= node.POSITION_TOLERANCE:
            self.stop(node)
            node.get_logger().info(
                f"Target reached. Distance error: {distance:.3f} m "
                f"(current: x={current_x:.2f}, y={current_y:.2f}; "
                f"target: x={target_x:.2f}, y={target_y:.2f})"
            )
            node.hover_started_at = time.monotonic()
            node.state = FlightState.HOVERING
            return

        speed = min(node.MAX_HORIZONTAL_SPEED, distance)
        vx = (dx / distance) * speed
        vy = (dy / distance) * speed

        self.send_velocity(node, vx, vy)

    def send_velocity(self, node: "FlightTestNode", vx: float, vy: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = 0.0
        node.cmd_vel_publisher.publish(msg)

    def stop(self, node: "FlightTestNode") -> None:
        self.send_velocity(node, 0.0, 0.0)
