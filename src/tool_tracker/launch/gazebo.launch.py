import os
import xacro
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    ros_gz_pkg_dir = get_package_share_directory("ros_gz_sim")
    pkg_panda_share_dir = get_package_share_directory("tool_tracker")
    xacro_file = os.path.join(pkg_panda_share_dir, "urdf", "simulation_scene.xacro")
    set_ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value="/opt/ros/humble/share"
    )

    if not os.path.exists(xacro_file):
            raise FileNotFoundError(f"Xacro file not found at expected path: {xacro_file}")
    robot_description = xacro.process_file(xacro_file).toxml()

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_pkg_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            'gz_args': 'empty.sdf -r'
        }.items()
    )

    robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
    )

    robot_spawner_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
              "-topic", "robot_description",
              "-name", "panda_vision_stage",
              "-x", "0.0",
              "-y", "0.0",
              "-z", "0.0"
        ],
        parameters=[{
              "robot_description": robot_description,
              "use_sim_time": True
        }],
        output="screen"
    )

    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo"
        ],
        output="screen"
    )

    return LaunchDescription([
        set_ign_resource_path,  
        gazebo_node,
        robot_state_publisher_node,
        robot_spawner_node,
        bridge_node
    ])