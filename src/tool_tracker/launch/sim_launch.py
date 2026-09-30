import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_tool_tracker = get_package_share_directory('tool_tracker')
    xacro_file = os.path.join(pkg_tool_tracker, 'urdf', 'simulation_scene.xacro')
    
    # Process Xacro
    xacro_file = os.path.join(pkg_tool_tracker, 'urdf', 'simulation_scene.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    # Joint State Publisher GUI (to move arm joints manually in simulation)
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Spawn Gazebo Fortress Simulator
    gazebo_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', 'empty.sdf'],
        output='screen'
    )

    # Spawn Robot Entity into Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', robot_description_raw, '-name', 'open_manipulator_zed'],
        output='screen'
    )

    # ROS-Gazebo Topic Bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/zed2i/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/zed2i/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'
        ],
        output='screen'
    )

    # Tool Tracking Node
    tracking_node = Node(
        package='tool_tracker',
        executable='tracking_node',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_sim,
        spawn_entity,
        ros_gz_bridge,
        tracking_node
    ])