# Tool Tracker

ROS 2 and Gazebo simulation of a Franka Panda manipulator with a camera and an AprilTag mounted on the hand. The detector looks for a `tag36h11` tag and displays the camera image with the detection outline and tag ID in an OpenCV window.

## Supported setup

- Ubuntu 22.04 (Jammy)
- ROS 2 Humble
- Gazebo Fortress through `ros_gz`

The launch files use ROS 2 Humble paths and the Gazebo Fortress `ros_gz` integration. See the [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) and [Gazebo Fortress ROS 2 integration guide](https://gazebosim.org/docs/fortress/ros2_integration/) for platform details.

## Install dependencies

Install ROS 2 Humble Desktop and initialize `rosdep` by following the ROS installation guide above. Then install the packages used by this repository:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-opencv \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 \
  ros-humble-cv-bridge \
  ros-humble-ros-gz \
  ros-humble-moveit-resources-panda-description

python3 -m pip install --user apriltag
```

`apriltag` is installed with pip because it is imported directly by the detector. OpenCV is installed from Ubuntu packages so its Python module is available to the system Python used by ROS.

Initialize `rosdep` once per machine, then refresh its package index:

```bash
sudo rosdep init
rosdep update
```

If `rosdep init` reports that it has already been initialized, continue with `rosdep update`.

## Clone and build

Create a ROS 2 workspace and clone this repository into its `src` directory:

```bash
mkdir -p ~/tool_tracker_ws/src
cd ~/tool_tracker_ws/src
git clone https://github.com/zuhurouf/tool_tracker.git
cd ~/tool_tracker_ws

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

For new terminals, source both ROS and the workspace overlay before using package commands:

```bash
source /opt/ros/humble/setup.bash
source ~/tool_tracker_ws/install/setup.bash
```

Build again after changing Python entry points, launch files, URDF/Xacro files, or meshes/textures:

```bash
cd ~/tool_tracker_ws
colcon build --packages-select tool_tracker --symlink-install
source install/setup.bash
```

## Run the simulation and detector

From a terminal with the ROS and workspace setup files sourced:

```bash
ros2 launch tool_tracker gazebo.launch.py
```

This starts Gazebo, spawns the Panda model, publishes camera data to `/camera/image_raw`, and starts the looping arm controller and AprilTag detector. The detector opens an OpenCV window titled **Tool tracker**. It draws a green outline and ID when it detects a tag. The annotated image is also published on `/camera/detections_image`.

The marker texture is a `tag36h11` tag with ID `0`. Keep the OpenCV window on a graphical desktop session; `cv2.imshow()` cannot open a window in a headless SSH session without a display server.

An RViz launch file is also available:

```bash
ros2 launch tool_tracker rviz.launch.py
```

## Useful checks

In another sourced terminal, confirm that Gazebo is publishing camera images:

```bash
ros2 topic list | grep camera
ros2 topic hz /camera/image_raw
```

If the OpenCV window shows the scene but no box, check that the marker is large and facing the camera, that its black-and-white pattern is visible without occlusion, and that the terminal running the detector reports no image conversion or GUI errors. Restart Gazebo after rebuilding to ensure it reloads updated meshes and textures.

## Repository layout

- `launch/` — Gazebo and RViz launch descriptions
- `urdf/panda.urdf` — Panda robot and attached marker model
- `urdf/simulation_scene.xacro` — camera, Gazebo sensor, and simulation integration
- `meshes/` — marker mesh and tag texture
- `tool_tracker/Detector.py` — camera subscription, AprilTag detection, and OpenCV display
- `tool_tracker/PandaController.py` — example Panda joint trajectory

