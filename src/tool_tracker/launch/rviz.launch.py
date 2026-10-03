import os
import xacro
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_panda_share_dir = get_package_share_directory("tool_tracker")
    xacro_file = os.path.join(pkg_panda_share_dir, "urdf", "simulation_scene.xacro")

    if not os.path.exists(xacro_file):
        raise FileNotFoundError(f"Xacro file not found at expected path: {xacro_file}")
    robot_description = xacro.process_file(xacro_file).toxml()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])