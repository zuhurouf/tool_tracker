# Tool Tracker

Real-time tracking of a manipulator's end-effector using vision.

## Supported setup

- Ubuntu 22.04 with ROS 2 Humble
- Gazebo Fortress through `ros_gz`
- A graphical desktop session for Gazebo and the OpenCV image window

Install [ROS 2 Humble Desktop](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
before continuing. The simulation uses a virtual ZED model; no camera hardware
or ZED SDK is required.

## Clone and install dependencies

Install the workspace tools (initialize `rosdep` only once per workstation):

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```

Clone the repository and its pinned description packages:

```bash
mkdir -p ~/ros2_ws
cd ~/ros2_ws
git clone --recurse-submodules https://github.com/zuhurouf/tool_tracker.git
cd tool_tracker
```

If the repository was cloned without `--recurse-submodules`, initialize the
submodules with:

```bash
git submodule update --init --recursive
```

Install ROS dependencies and build the tracker and its required description
packages:

```bash
source /opt/ros/humble/setup.bash
rosdep install \
  --from-paths src/tool_tracker \
               src/external/open_manipulator/open_manipulator_x_description \
               src/external/zed-ros2-description \
  --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --packages-up-to tool_tracker
```

## Run the simulation

In each new terminal, source ROS and this workspace, then launch the simulation:

```bash
cd ~/ros2_ws/tool_tracker
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tool_tracker sim_launch.py
```

The launch file starts Ignition Gazebo, spawns the arm and camera, bridges the
camera image and camera info to ROS, and starts the tracker. The tracker displays
images received on `/zed2i/camera/image` in an OpenCV window. The launch file
sets `IGN_GAZEBO_RESOURCE_PATH` from the installed description packages, so no
workstation-specific mesh paths should be needed.

Check the camera stream from another sourced terminal:

```bash
ros2 topic list | grep zed2i/camera
ros2 topic hz /zed2i/camera/image
```

## Troubleshooting

- If ROS cannot find `tool_tracker` or one of the description packages, source
  `/opt/ros/humble/setup.bash`, rebuild from the workspace root, then source
  `install/setup.bash` again.
- If a fresh clone has empty dependency directories, run
  `git submodule update --init --recursive`.
- Missing mesh errors usually mean the launch file was not started from the
  sourced workspace installation. The launch file configures the installed
  package-share paths automatically.
- `libEGL` / `dri2` warnings indicate a graphics-driver or display issue, not a
  missing URDF mesh. Run from a desktop session with working OpenGL support if
  Gazebo or the OpenCV window fails to render.