# Tool Tracker

Real-time tracking of a manipulator's end-effector using vision.

## Supported Setup

- **OS**: Ubuntu 22.04 LTS
- **ROS Version**: ROS 2 Humble Desktop
- **Simulator**: Gazebo Ignition Fortress 

---

## Prerequisites & System Dependencies

Before building the workspace, ensure all required system packages, Gazebo Fortress integrations, and description packages are installed:

```bash
# 1. Update package lists
sudo apt update

# 2. Install ROS 2 Humble Gazebo Fortress dependencies & MoveIt Panda Description
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-moveit-resources-panda-description \
  ros-humble-robot-state-publisher \
  ros-humble-xacro