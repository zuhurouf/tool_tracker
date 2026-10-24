#!/usr/bin/env python3
import time
import math

import numpy as np
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

HOME_POSITION = (0.0, 0.0, 0.0, -1.652, -0.047, 1.613, 0.705)
JOINT_ORIGINS = (
    ((0.0, 0.0, 0.333), (0.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (-math.pi / 2, 0.0, 0.0)),
    ((0.0, -0.316, 0.0), (math.pi / 2, 0.0, 0.0)),
    ((0.0825, 0.0, 0.0), (math.pi / 2, 0.0, 0.0)),
    ((-0.0825, 0.384, 0.0), (-math.pi / 2, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (math.pi / 2, 0.0, 0.0)),
    ((0.088, 0.0, 0.0), (math.pi / 2, 0.0, 0.0)),
)
JOINT_LIMITS = np.array((
    (-2.9671, 2.9671),
    (-1.8326, 1.8326),
    (-2.9671, 2.9671),
    (-3.1416, 0.0873),
    (-2.9671, 2.9671),
    (-0.0873, 3.8223),
    (-2.9671, 2.9671),
))
HAND_TRANSFORM = ((0.0, 0.0, 0.107), (0.0, 0.0, -math.pi / 4))
MARKER_TRANSFORM = ((0.050, 0.0, 0.050), (0.0, math.pi / 2, 0.0))
CIRCLE_RADIUS = 0.14
CIRCLE_PERIOD = 10.0
APPROACH_DURATION = 3.0
CIRCLE_HEIGHT_OFFSET = 0.0
ORIENTATION_WEIGHT = 0.25
PUBLISH_PERIOD = 0.05


def rotation_matrix(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array((
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    ))


def transform(xyz, rpy):
    result = np.eye(4)
    result[:3, :3] = rotation_matrix(rpy)
    result[:3, 3] = xyz
    return result


def rotation_about_z(angle):
    result = np.eye(4)
    cosine, sine = math.cos(angle), math.sin(angle)
    result[:3, :3] = ((cosine, -sine, 0.0), (sine, cosine, 0.0), (0.0, 0.0, 1.0))
    return result


def marker_pose_and_jacobian(joint_positions):
    world_from_link = np.eye(4)
    joint_origins = []
    joint_axes = []

    for position, (xyz, rpy) in zip(joint_positions, JOINT_ORIGINS):
        world_from_joint = world_from_link @ transform(xyz, rpy)
        joint_origins.append(world_from_joint[:3, 3].copy())
        joint_axes.append(world_from_joint[:3, :3] @ np.array((0.0, 0.0, 1.0)))
        world_from_link = world_from_joint @ rotation_about_z(position)

    world_from_hand = world_from_link @ transform(*HAND_TRANSFORM)
    world_from_marker = world_from_hand @ transform(*MARKER_TRANSFORM)
    marker_position = world_from_marker[:3, 3]
    position_jacobian = np.column_stack([
        np.cross(axis, marker_position - origin)
        for origin, axis in zip(joint_origins, joint_axes)
    ])
    orientation_jacobian = np.column_stack(joint_axes)
    return world_from_marker, position_jacobian, orientation_jacobian


def rotation_vector(rotation):
    cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    angle = math.acos(cosine)
    skew_vector = np.array((
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    ))
    if angle < 1e-6:
        return 0.5 * skew_vector
    return angle * skew_vector / (2.0 * math.sin(angle))


def solve_marker_ik(target_position, target_orientation, seed):
    joint_positions = np.asarray(seed, dtype=float).copy()
    for _ in range(30):
        pose, position_jacobian, orientation_jacobian = marker_pose_and_jacobian(
            joint_positions
        )
        position_error = target_position - pose[:3, 3]
        orientation_error = rotation_vector(target_orientation @ pose[:3, :3].T)
        error = np.concatenate((
            position_error,
            ORIENTATION_WEIGHT * orientation_error,
        ))
        if np.linalg.norm(position_error) < 0.002 and np.linalg.norm(orientation_error) < 0.02:
            return joint_positions

        jacobian = np.vstack((
            position_jacobian,
            ORIENTATION_WEIGHT * orientation_jacobian,
        ))
        damping = 0.04
        delta = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + damping**2 * np.eye(6),
            error,
        )
        delta = np.clip(delta, -0.08, 0.08)
        joint_positions = np.clip(
            joint_positions + delta,
            JOINT_LIMITS[:, 0],
            JOINT_LIMITS[:, 1],
        )
    return None


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
        self.home_marker_pose, _, _ = marker_pose_and_jacobian(HOME_POSITION)
        self.circle_center = self.home_marker_pose[:3, 3].copy()
        self.circle_center[2] += CIRCLE_HEIGHT_OFFSET
        self.circle_start = self.circle_center.copy()
        self.circle_start[2] += CIRCLE_RADIUS
        self.last_position = np.asarray(HOME_POSITION, dtype=float)
        self.ik_warning_logged = False
        self.started_at = None
        self.start_timer = self.create_timer(2.0, self.start_trajectory)
        self.trajectory_timer = None
        self.get_logger().info(
            "Panda controller ready; lifting the camera-facing marker, then tracing a reachable vertical circle."
        )

    def start_trajectory(self):
        self.start_timer.cancel()
        self.started_at = time.monotonic()
        self.trajectory_timer = self.create_timer(
            PUBLISH_PERIOD, self.publish_trajectory
        )

    def position_at(self, elapsed):
        if elapsed < APPROACH_DURATION:
            # Lift the marker to the elevated circle's starting point before drawing.
            progress = max(0.0, elapsed / APPROACH_DURATION)
            progress = progress * progress * (3.0 - 2.0 * progress)
            target_position = (
                self.home_marker_pose[:3, 3]
                + progress * (self.circle_start - self.home_marker_pose[:3, 3])
            )
        else:
            phase = (
                2.0 * math.pi
                * ((elapsed - APPROACH_DURATION) % CIRCLE_PERIOD)
                / CIRCLE_PERIOD
            )
            # The Y-Z plane faces the camera; the whole loop stays elevated.
            target_position = self.circle_center.copy()
            target_position[1] += CIRCLE_RADIUS * math.sin(phase)
            target_position[2] += CIRCLE_RADIUS * math.cos(phase)
        solution = solve_marker_ik(
            target_position,
            self.home_marker_pose[:3, :3],
            self.last_position,
        )
        if solution is None:
            if not self.ik_warning_logged:
                self.get_logger().warning(
                    "Circle IK did not converge; holding the last valid arm position."
                )
                self.ik_warning_logged = True
            return tuple(self.last_position)
        self.ik_warning_logged = False
        self.last_position = solution
        return tuple(solution)

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
