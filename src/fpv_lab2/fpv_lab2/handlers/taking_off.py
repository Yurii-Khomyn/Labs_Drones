import time
from functools import partial
from typing import TYPE_CHECKING
from ardupilot_msgs.srv import Takeoff
from ..flight_state import FlightState
from .handler import FlightStateHandler

if TYPE_CHECKING:
    from ..flight_test_node import FlightTestNode

ALTITUDE_TOLERANCE = 0.15


class TakingOffHandler(FlightStateHandler):
    def handle(self, node: "FlightTestNode") -> None:
        if not node.request_in_progress and not node.takeoff_requested:
            self.call_takeoff(node)
            return

        if node.takeoff_requested and node.current_pose is not None:
            altitude = node.current_pose.pose.position.z
            if altitude >= node.TAKEOFF_ALTITUDE - ALTITUDE_TOLERANCE:
                node.get_logger().info(
                    f"Target altitude reached: {altitude:.2f} m"
                )
                node.get_logger().info("Flying to target point")
                node.state = FlightState.FLYING_TO_TARGET

    def call_takeoff(self, node: "FlightTestNode") -> None:
        node.request_in_progress = True
        request = Takeoff.Request()
        request.alt = float(node.TAKEOFF_ALTITUDE)
        future = node.takeoff_client.call_async(request)
        future.add_done_callback(partial(self.takeoff_callback, node))
        node.get_logger().info(
            f"Requesting takeoff to {node.TAKEOFF_ALTITUDE:.1f} m"
        )

    def takeoff_callback(self, node: "FlightTestNode", future) -> None:
        node.request_in_progress = False
        try:
            response = future.result()
        except Exception as error:
            node.fail(f"Takeoff service failed: {error}")
            return

        if not response.status:
            node.fail("ArduPilot rejected the takeoff request")
            return

        node.get_logger().info("Takeoff command accepted")
        node.get_logger().info("Waiting to reach target altitude")
        node.takeoff_requested = True
