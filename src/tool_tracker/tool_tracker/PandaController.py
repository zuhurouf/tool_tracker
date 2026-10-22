#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


JOINT_NAMES = (
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
)

HOME_POSITION = (0.0, 0.0, -0.172, -1.2, -0.266, 1.592, 0.486)
# Keep the sweep close to home so the hand does not fold too far inward/outward.
LEFT_POSITION = (*HOME_POSITION[:2], -0.422, *HOME_POSITION[3:])
RIGHT_POSITION = (*HOME_POSITION[:2], 0.078, *HOME_POSITION[3:])
TRAJECTORY = (
    (0.0, HOME_POSITION),
    (2.0, HOME_POSITION),
    (5.0, LEFT_POSITION),
    (7.0, LEFT_POSITION),
    (11.0, RIGHT_POSITION),
    (13.0, RIGHT_POSITION),
    (16.0, HOME_POSITION),
    (18.0, HOME_POSITION),
)
TRAJECTORY_DURATION = TRAJECTORY[-1][0]
PUBLISH_PERIOD = 0.05


class PandaController(Node):
    def __init__(self):
        super().__init__("panda_trajectory_controller")
        self.joint_publishers = {
            joint_name: self.create_publisher(
                Float64,
                f"/model/panda_tracker/joint/{joint_name}/cmd_pos",
                10,
            )
            for joint_name in JOINT_NAMES
        }
        self.started_at = None
        self.start_timer = self.create_timer(2.0, self.start_trajectory)
        self.trajectory_timer = None
        self.get_logger().info(
            "Panda controller ready; holding the screenshot home pose and "
            "sweeping panda_joint3 left and right."
        )

    def start_trajectory(self):
        self.start_timer.cancel()
        self.started_at = time.monotonic()
        self.trajectory_timer = self.create_timer(
            PUBLISH_PERIOD, self.publish_trajectory
        )

    def position_at(self, elapsed):
        cycle_time = elapsed % TRAJECTORY_DURATION
        for (start_time, start_position), (end_time, end_position) in zip(
            TRAJECTORY, TRAJECTORY[1:]
        ):
            if cycle_time <= end_time:
                fraction = (cycle_time - start_time) / (end_time - start_time)
                fraction = fraction * fraction * (3.0 - 2.0 * fraction)
                return tuple(
                    start + (end - start) * fraction
                    for start, end in zip(start_position, end_position)
                )
        return HOME_POSITION

    def publish_trajectory(self):
        elapsed = time.monotonic() - self.started_at
        positions = self.position_at(elapsed)
        for joint_name, position in zip(JOINT_NAMES, positions):
            message = Float64()
            message.data = position
            self.joint_publishers[joint_name].publish(message)


def main(args=None):
    rclpy.init(args=args)
    controller = PandaController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
