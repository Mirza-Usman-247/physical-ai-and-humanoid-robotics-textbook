# Chapter 1: ROS 2 Architecture - Code Examples

This directory contains code examples for Chapter 1: ROS 2 Architecture & Core Concepts.

## Prerequisites

- ROS 2 Humble installed (see [ROS 2 Installation](https://docs.ros.org/en/humble/Installation.html))
- Python 3.8+ or C++17 compiler
- Basic familiarity with ROS 2 workspace structure

## Examples

### 1. `simple_node.py` - Basic Node Creation

**Learning Objectives:**
- Create a minimal ROS 2 node using Python
- Understand node lifecycle (init, spin, shutdown)
- Use ROS 2 logging system
- Implement timer callbacks

**How to Run:**

```bash
# Navigate to your ROS 2 workspace
cd ~/ros2_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Run the node directly
python3 simple_node.py

# OR run using ros2 run (if installed as a package)
ros2 run chapter_1_examples simple_node
```

**Expected Output:**

```
[INFO] [simple_node]: Simple node has been started!
[INFO] [simple_node]: Timer callback #1
[INFO] [simple_node]: Timer callback #2
[INFO] [simple_node]: Timer callback #3
[INFO] [simple_node]: Timer callback #4
[INFO] [simple_node]: Timer callback #5
[WARN] [simple_node]: This is a warning at count 5
...
[INFO] [simple_node]: Timer callback #10
[ERROR] [simple_node]: This is an error message at count 10
```

**Key Concepts Demonstrated:**

- **Node Initialization**: `super().__init__('simple_node')` creates a node with name 'simple_node'
- **Timer Callbacks**: `create_timer(period, callback)` schedules periodic execution
- **Logging Levels**: `get_logger().info()`, `.warn()`, `.error()` for different severity levels
- **Lifecycle Management**: `rclpy.init()`, `rclpy.spin()`, `rclpy.shutdown()` pattern
- **Clean Shutdown**: Try-except-finally ensures proper resource cleanup

## Troubleshooting

**Error: `ModuleNotFoundError: No module named 'rclpy'`**
- Solution: Source your ROS 2 environment with `source /opt/ros/humble/setup.bash`

**Node doesn't stop with Ctrl+C:**
- The KeyboardInterrupt handler should catch this. If not, use `ros2 node kill /simple_node`

**Permission denied error:**
- Make the script executable: `chmod +x simple_node.py`

## Next Steps

After mastering this example:
1. Modify the timer period to see faster/slower callbacks
2. Add a second timer with a different period
3. Experiment with different log levels
4. Try inspecting the node with `ros2 node info /simple_node`

## References

- [ROS 2 Python Client Library (rclpy)](https://docs.ros.org/en/humble/p/rclpy/)
- [Creating a Node](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
