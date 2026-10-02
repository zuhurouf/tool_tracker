import os
import xml.etree.ElementTree as ET
import xacro
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_tool_tracker = get_package_share_directory("tool_tracker")
    gazebo_resource_path = os.pathsep.join(
        filter(
            None,
            [
                os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
                os.path.dirname(get_package_share_directory("open_manipulator_x_description")),
                os.path.dirname(get_package_share_directory("zed_description")),
            ],
        )
    )

    xacro_file = os.path.join(pkg_tool_tracker, "urdf", "simulation_scene.xacro")
    raw_robot_description = xacro.process_file(xacro_file).toxml()
    robot_description = ET.fromstring(raw_robot_description)
    camera_mount = robot_description.find("link[@name='zed2i_camera_link']")
    if camera_mount is None:
        raise RuntimeError("ZED camera mount link is missing from the robot description")

    camera_inertial = ET.SubElement(camera_mount, "inertial")
    ET.SubElement(camera_inertial, "mass", value="0.001")
    ET.SubElement(
        camera_inertial,
        "inertia",
        ixx="0.000001",
        ixy="0",
        ixz="0",
        iyy="0.000001",
        iyz="0",
        izz="0.000001",
    )
    raw_robot_description = ET.tostring(robot_description, encoding="unicode")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": raw_robot_description, "use_sim_time": True}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui"
    )

    gazebo_sim = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "empty.sdf"],
        output="screen"
    )

    robot_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-string", raw_robot_description, "-name", "open_manipulator_zed"],
        output="screen"
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/zed2i/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/zed2i/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'
        ],
        output='screen'
    )

    tracker_node = Node(
        package="tool_tracker",
        executable="tracker_node",
        output="screen"
    )

    return LaunchDescription([
        SetEnvironmentVariable(name="IGN_GAZEBO_RESOURCE_PATH", value=gazebo_resource_path),
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        gazebo_sim,
        robot_entity_node,
        ros_gz_bridge,
        tracker_node
    ])