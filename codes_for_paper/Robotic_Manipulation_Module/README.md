# Environment Setup Guide

## 1. Basic Environment

- OS: Ubuntu 20.04 LTS (recommended)
- ROS Version: Noetic (for Ubuntu 20.04)
- Python Version: 3.8+ (Noetic)

## 2. Core ROS Dependencies Installation

### Essential ROS Packages

```bash
# For ROS Noetic
sudo apt install ros-noetic-catkin ros-noetic-rostopic ros-noetic-rospy ros-noetic-moveit-core ros-noetic-moveit-kinematics ros-noetic-moveit-planners-ompl ros-noetic-gazebo-ros-control ros-noetic-joint-trajectory-controller ros-noetic-ur-description ros-noetic-trac-ik-kinematics-plugin ros-noetic-controller-manager ros-noetic-geometry-msgs ros-noetic-joy ros-noetic-sensor-msgs ros-noetic-actionlib ros-noetic-control-msgs ros-noetic-kdl-parser ros-noetic-pluginlib ros-noetic-tf2-ros ros-noetic-xacro ros-noetic-rviz ros-noetic-robot-state-publisher

# Configure ROS environment variables
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc  # For Noetic
source ~/.bashrc
```

## 3. Python Dependencies

### requirements.txt

```
# Basic dependencies
numpy>=1.19.5
rospkg>=1.2.9
catkin_pkg>=0.4.23
pyserial>=3.5
pyyaml>=5.4.1
scipy>=1.5.4
```

### Install Python Dependencies

```bash
# Upgrade pip
pip install --upgrade pip

# Install from requirements.txt
pip install -r requirements.txt
```

## 4. Environment Verification

```bash
# Check ROS version
rosversion -d

# Start ROS core (no error means normal)
roscore

# Verify Python dependencies
python -c "import rospy, numpy, serial"
```
## 5. Key Module Implementation Paths
- The fine compliance controller is implemented in `./src/selfdefined_trajectory_controller`.
- The fuzzy controller for weighing is implemented in `./src/dual_arm/manipulator/src/Screw_capsulation.cpp`.

## 6. Launch the Program
```bash
# Grant execution permission to the launch script (if not already set)
chmod +x ./src/PWS/screwlaunch.sh

# Run the script to start the program
./src/PWS/screwlaunch.sh
```