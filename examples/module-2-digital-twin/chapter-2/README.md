# Chapter 2: Gazebo Basics - Code Examples

This directory contains URDF/SDF files and launch scripts for Gazebo robot simulation.

## Files

- `simple_robot.urdf.xacro` - Four-wheeled mobile robot with camera sensor
- `simple_world.sdf` - Gazebo world with ground plane and obstacles
- `README.md` - This file

## Prerequisites

```bash
# Install Gazebo Classic 11 (Ubuntu 22.04)
sudo apt install gazebo libgazebo-dev

# Install Gazebo ROS 2 packages
sudo apt install ros-humble-gazebo-ros-pkgs

# Install xacro for URDF processing
sudo apt install ros-humble-xacro
```

## Usage

### 1. Launch Gazebo with Custom World

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Launch Gazebo
gazebo simple_world.sdf
```

### 2. Spawn Robot Model

In a new terminal:
```bash
source /opt/ros/humble/setup.bash

# Process XACRO to URDF
xacro simple_robot.urdf.xacro > simple_robot.urdf

# Spawn entity
ros2 run gazebo_ros spawn_entity.py \
  -file simple_robot.urdf \
  -entity simple_robot \
  -x 0 -y 0 -z 0.2
```

### 3. Control Robot

```bash
# Publish velocity commands
ros2 topic pub /simple_robot/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.2}}"
```

### 4. View Camera Feed

```bash
# Install image tools
sudo apt install ros-humble-rqt-image-view

# View camera
rqt_image_view /simple_robot/camera/image_raw
```

## Troubleshooting

**Robot falls through ground:**
- Check collision geometry in URDF
- Verify ground plane in world file

**No camera data:**
- Check `GAZEBO_PLUGIN_PATH`: `echo $GAZEBO_PLUGIN_PATH`
- Verify libgazebo_ros_camera.so exists
- Check topic remapping in URDF

**Joints vibrate:**
- Reduce time step in world physics
- Check inertia tensors are realistic
- Increase solver iterations

## Exercises

1. Add a LiDAR sensor to the robot
2. Create ramps and stairs in world file
3. Configure joint limits for steering
4. Add textures to visual geometries

## References

- [Gazebo Tutorials](https://classic.gazebosim.org/tutorials)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
