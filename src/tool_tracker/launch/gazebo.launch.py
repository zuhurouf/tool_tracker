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
    package_resource_root = os.path.dirname(pkg_panda_share_dir)

    ign_resource_paths = [
        package_resource_root,
        os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
        "/opt/ros/humble/share",
    ]
    set_ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=os.pathsep.join(path for path in ign_resource_paths if path),
    )

    gz_resource_paths = [
        package_resource_root,
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    ]
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.pathsep.join(path for path in gz_resource_paths if path),
    )


    if not os.path.exists(xacro_file):
        raise FileNotFoundError(
            f"Xacro file not found at expected path: {xacro_file}"
        )
    robot_description = xacro.process_file(xacro_file).toxml()

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_pkg_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items()
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
            "-name", "panda_tracker",
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
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            *[
                (
                    f"/model/panda_tracker/joint/{joint_name}/cmd_pos"
                    "@std_msgs/msg/Float64]ignition.msgs.Double"
                )
                for joint_name in (
                    "panda_joint1",
                    "panda_joint2",
                    "panda_joint3",
                    "panda_joint4",
                    "panda_joint5",
                    "panda_joint6",
                    "panda_joint7",
                )
            ],
        ],
        output="screen"
    )

    cam_subscriber_node = Node(
          package="tool_tracker",
          executable="CameraSubscriber_py",
          output="screen"
    )

    panda_controller_node = Node(
        package="tool_tracker",
        executable="PandaController_py",
        output="screen",
    )

    return LaunchDescription([
        set_ign_resource_path,
        set_gz_resource_path,
        gazebo_node,
        robot_state_publisher_node,
        bridge_node,
        robot_spawner_node,
        cam_subscriber_node,
        panda_controller_node,
    ])
