# Chapter 4: TF Transforms - Code Examples

This directory contains code examples for Chapter 4: TF Transforms (Transform Library).

## Prerequisites

- ROS 2 Humble installed with `tf2_ros` package
- Python 3.8+ with rclpy
- Basic understanding of ROS 2 nodes and topics (Chapter 1-2)
- Understanding of coordinate frames and transformations

## Examples

### 1. `tf_broadcaster.py` - Publishing Static & Dynamic Transforms

**Learning Objectives:**
- Create and publish static transforms (e.g., sensor mounting)
- Publish dynamic transforms (e.g., robot movement)
- Use `StaticTransformBroadcaster` for fixed transforms
- Use `TransformBroadcaster` for moving transforms
- Represent 3D rotations using quaternions

**How to Run:**

```bash
# Terminal 1: Start the broadcaster (publishes transforms)
source /opt/ros/humble/setup.bash
python3 tf_broadcaster.py

# Terminal 2: View the transform tree
source /opt/ros/humble/setup.bash
ros2 run tf2_tools view_frames.py
# Creates a PDF showing the transform hierarchy

# Terminal 3: Inspect a specific transform (in real-time)
ros2 run tf2_ros tf2_echo odom base_link
```

**Expected Output:**

```
[INFO] [tf_broadcaster]: TF Broadcaster node started
[INFO] [tf_broadcaster]: Published static transform: base_link → camera_link
[INFO] [tf_broadcaster]: Published dynamic transform: odom → base_link (x=1.00, y=0.00, θ=0.0°)
[INFO] [tf_broadcaster]: Published dynamic transform: odom → base_link (x=0.95, y=0.31, θ=28.6°)
[INFO] [tf_broadcaster]: Published dynamic transform: odom → base_link (x=0.81, y=0.59, θ=57.3°)
```

**Key Concepts Demonstrated:**

- **Frame IDs**: Hierarchical naming convention (e.g., `odom`, `base_link`, `camera_link`)
- **Static Transforms**: Fixed relationships (sensor mounting) published once at startup
- **Dynamic Transforms**: Time-varying relationships (robot motion) updated periodically
- **Quaternion Rotation**: 4D representation `(x, y, z, w)` for 3D orientation
- **Transform Tree**: Directed graph where nodes are frames and edges are transformations
- **Timestamp Handling**: Each transform includes timestamp for temporal interpolation

**Transform Tree Created:**
```
odom
 └── base_link
      └── camera_link
```

### 2. `tf_listener.py` - Querying Transforms

**Learning Objectives:**
- Listen to transforms published by broadcasters
- Query specific transforms between any two frames
- Use TF2 buffer for temporal interpolation
- Transform points between coordinate frames
- Handle timing and lookup failures

**How to Run:**

```bash
# Terminal 1: Start the broadcaster (in separate process)
python3 tf_broadcaster.py

# Terminal 2: Start the listener
python3 tf_listener.py

# You should see queries printed every 1 second
```

**Expected Output:**

```
[INFO] [tf_listener]: TF Listener node started (waiting for transforms...)
[INFO] [tf_listener]: odom → base_link: x=1.00m, y=0.00m, z=0.00m, yaw=0.0°
[INFO] [tf_listener]: odom → camera_link (via base_link): x=1.10m, y=0.00m, z=0.05m, yaw=0.0°
[INFO] [tf_listener]: Point in camera_link (2.0m forward) → odom: x=3.10m, y=0.00m, z=0.05m
```

**Key Concepts Demonstrated:**

- **Transform Buffer**: Maintains a history of transforms for temporal queries
- **Lookup Transform**: Query relationship between any two connected frames
- **Frame Chain Resolution**: Automatically resolves multi-hop paths (odom → base_link → camera_link)
- **Point Transformation**: Convert coordinates from one frame to another
- **Error Handling**: Gracefully handle timing mismatches and disconnected frames

### Use Cases

#### Multi-Sensor Fusion
```python
# Listener: Combine camera and LiDAR observations in a common frame
camera_detection_in_base = tf_buffer.transform(camera_point, 'base_link')
lidar_detection_in_base = tf_buffer.transform(lidar_point, 'base_link')
# Now both observations are in the same frame for fusion
```

#### Navigation & Localization
```python
# Query robot position in world frame
robot_pose_in_world = tf_buffer.lookup_transform('world', 'base_link', Time())
```

#### Manipulation & Grasping
```python
# Find gripper position relative to observed object
gripper_in_camera = tf_buffer.lookup_transform('camera_link', 'gripper', Time())
# Use for grasp planning
```

## Troubleshooting

**Error: `LookupException: base_link does not exist`**
- Solution: Wait for broadcaster to start and publish transforms
- Check with `ros2 topic list` to verify transforms are being published

**Error: `ExtrapolationException`**
- Reason: Querying a transform too far in the future (timestamp too new)
- Solution: Use `Time()` (current time) instead of future timestamps

**Transform tree looks disconnected:**
- Verify both static and dynamic broadcasters are running
- Check frame IDs match exactly (case-sensitive)
- Use `ros2 run tf2_tools view_frames.py` to visualize the tree

**High latency in queries:**
- TF2 buffer maintains a history; older queries are slower
- For real-time control, use recent timestamps

## Next Steps

1. Modify the broadcaster to add more static frames (e.g., `lidar_link`, `imu_link`)
2. Create a listener that transforms sensor data from multiple frames
3. Visualize the transform tree with RViz
4. Implement a simple localization node that publishes `map → odom` transform
5. Combine with Chapter 2 (pub/sub) to publish sensor data in transformed frames

## References

- [ROS 2 TF2 Documentation](https://docs.ros.org/en/humble/Concepts/Intermediate/Tf2/Tf2-Main.html)
- [TF2 Broadcaster & Listener](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Broadcasters-Listeners.html)
- [Transform Library (TF2) Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [Quaternion Mathematics](https://en.wikipedia.org/wiki/Quaternion)
