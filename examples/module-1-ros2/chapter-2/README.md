# Chapter 2: Publisher-Subscriber Pattern - Code Examples

This directory contains code examples demonstrating ROS 2 topic-based communication using the publisher-subscriber pattern.

## Prerequisites

- ROS 2 Humble installed
- Python 3.8+
- Basic understanding of ROS 2 nodes (see Chapter 1)

## Examples

### 1. Minimal Publisher/Subscriber (`minimal_publisher.py` & `minimal_subscriber.py`)

**Learning Objectives:**
- Create basic publisher and subscriber nodes
- Configure QoS profiles for reliable communication
- Synchronize QoS settings between endpoints

**How to Run:**

Terminal 1 (Publisher):
```bash
source /opt/ros/humble/setup.bash
python3 minimal_publisher.py
```

Terminal 2 (Subscriber):
```bash
source /opt/ros/humble/setup.bash
python3 minimal_subscriber.py
```

**Expected Output:**

Publisher:
```
[INFO] [minimal_publisher]: Minimal publisher started on /chatter
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
...
```

Subscriber:
```
[INFO] [minimal_subscriber]: Minimal subscriber started, listening to /chatter
[INFO] [minimal_subscriber]: I heard: "Hello World: 0"
[INFO] [minimal_subscriber]: I heard: "Hello World: 1"
...
```

**Key Concepts:**
- **QoS Matching**: Both publisher and subscriber use RELIABLE reliability
- **History Depth**: KEEP_LAST with depth=10 buffers up to 10 messages
- **Timer-based Publishing**: Publisher uses 2 Hz timer (0.5s period)
- **Callback Pattern**: Subscriber processes messages asynchronously

---

### 2. Custom Message Publisher (`custom_publisher.py` + `msg/SensorData.msg`)

**Learning Objectives:**
- Define custom ROS 2 message types
- Use composite messages with multiple data types
- Configure QoS for high-frequency sensor data streaming

**Custom Message Structure:**

`SensorData.msg` contains:
- `std_msgs/Header header` - Timestamp and frame ID
- `string sensor_id` - Unique sensor identifier
- `float64 temperature, humidity, pressure` - Sensor readings
- `bool is_calibrated` - Calibration status
- `uint8 error_code` - Error status (0 = OK)
- `float32 measurement_confidence` - Data quality metric

**QoS Configuration:**

This example uses **Best Effort** reliability for sensor data:
- **Reliability**: Best Effort (tolerate loss for performance)
- **Durability**: Volatile (no persistence)
- **History**: Keep Last 1 (only latest reading)

**Why Best Effort for Sensors?**
- High-frequency data (10 Hz) - latest reading most important
- Network bandwidth optimization
- Acceptable to skip old frames if network is congested
- Lower latency compared to Reliable mode

**Building Custom Messages:**

To use `SensorData.msg` in a real ROS 2 package:

1. Create package structure:
```bash
ros2 pkg create my_interfaces --build-type ament_cmake
mkdir -p my_interfaces/msg
cp msg/SensorData.msg my_interfaces/msg/
```

2. Update `CMakeLists.txt`:
```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorData.msg"
  DEPENDENCIES std_msgs
)
```

3. Update `package.xml`:
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
<depend>std_msgs</depend>
```

4. Build and source:
```bash
colcon build --packages-select my_interfaces
source install/setup.bash
```

5. Update Python import:
```python
from my_interfaces.msg import SensorData
```

---

## Troubleshooting

**Publisher and subscriber don't connect:**
- Check QoS compatibility: Run `ros2 topic info /chatter --verbose`
- Verify both nodes are on the same ROS domain: `echo $ROS_DOMAIN_ID`
- Check network discovery: `ros2 node list` should show both nodes

**Subscriber misses messages:**
- Increase `depth` in KEEP_LAST history
- Switch to RELIABLE reliability policy
- Check callback execution time (long callbacks block message reception)

**Custom message not found:**
- Ensure `my_interfaces` package is built and sourced
- Verify message generation: `ros2 interface show my_interfaces/msg/SensorData`
- Check Python path: `python3 -c "from my_interfaces.msg import SensorData"`

---

## Introspection Commands

Monitor topics in real-time:
```bash
# List all active topics
ros2 topic list

# Show topic details including QoS
ros2 topic info /chatter --verbose

# Echo messages to console
ros2 topic echo /chatter

# Measure message frequency
ros2 topic hz /chatter

# Visualize pub/sub graph
rqt_graph
```

---

## QoS Experimentation

Try modifying QoS policies to see effects:

1. **Mismatched Reliability**: Set publisher to BEST_EFFORT, subscriber to RELIABLE - connection fails
2. **Durability Test**: Set publisher to TRANSIENT_LOCAL, start subscriber after publisher - late joiner receives buffered messages
3. **History Depth**: Set depth=1, publish faster than subscriber processes - messages get dropped

---

## Next Steps

After mastering these examples:
1. Create a sensor fusion node that subscribes to multiple topics
2. Implement a QoS adapter that republishes data with different QoS
3. Add message filtering to subscriber callback
4. Measure latency between publish and subscribe timestamps

## References

- [ROS 2 Topics Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [QoS Policies](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- [Creating Custom Messages](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
