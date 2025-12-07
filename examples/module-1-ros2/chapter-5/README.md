# Chapter 5: Robot Control (ros2_control) - Code Examples

This directory contains code examples for Chapter 5: Robot Control with ros2_control.

## Prerequisites

- ROS 2 Humble with `ros2_control` and `control_msgs` packages
- A robot with ros2_control integration (see `YAML configuration` section below)
- Understanding of ROS 2 actions (Chapter 3)
- Understanding of joint coordinates and kinematics

## ros2_control Architecture Overview

```
┌─────────────────────────────────────────────────┐
│             Your ROS 2 Application              │
│          (This example node)                    │
└────────────────┬────────────────────────────────┘
                 │ (action goals)
         ┌───────▼───────┐
         │   Controller  │  (Joint Trajectory Controller)
         │   Manager     │  Manages timing & joint commands
         └───────┬───────┘
                 │ (joint setpoints)
         ┌───────▼───────┐
         │   Hardware    │  (Robot hardware/simulation)
         │   Interface   │  Reads sensors, writes actuators
         └───────┬───────┘
                 │ (sensor feedback)
         ┌───────▼───────┐
         │    Robot      │  (Physical or simulated)
         └───────────────┘
```

## Example 1: `joint_trajectory_controller.py` - Trajectory Execution

**Learning Objectives:**
- Send trajectory goals to a robot using actions
- Monitor joint state feedback in real-time
- Handle controller responses (acceptance, feedback, results)
- Understand the relationship between trajectory controller and hardware interface

**How to Run (Simulation):**

```bash
# Terminal 1: Launch the robot (with ros2_control) in Gazebo/Isaac Sim
ros2 launch my_robot_bringup robot.launch.py sim:=true

# Terminal 2: Verify controller is loaded
ros2 control list_controllers
# Expected output:
# arm_controller [joint_trajectory_controller/JointTrajectoryController] active

# Terminal 3: Run the trajectory example
source /opt/ros/humble/setup.bash
python3 joint_trajectory_controller.py
```

**Expected Output:**

```
[INFO] [joint_trajectory_controller]: Waiting for trajectory controller...
[INFO] [joint_trajectory_controller]: Trajectory controller ready!
[INFO] [joint_trajectory_controller]: Sending trajectory goal: {'shoulder': 0.785, 'elbow': 0.0, 'wrist': 0.0}
[INFO] [joint_trajectory_controller]: Goal accepted!
[INFO] [joint_trajectory_controller]: Executing: shoulder=0.10rad, elbow=0.00rad, wrist=0.00rad
[INFO] [joint_trajectory_controller]: Executing: shoulder=0.39rad, elbow=0.00rad, wrist=0.00rad
[INFO] [joint_trajectory_controller]: Executing: shoulder=0.78rad, elbow=0.00rad, wrist=0.00rad
[INFO] [joint_trajectory_controller]: Trajectory execution SUCCESSFUL!
```

**Key Concepts Demonstrated:**

1. **Action Client**: Sends trajectory goals asynchronously (non-blocking)
2. **Goal Response**: Server accepts/rejects based on readiness
3. **Feedback**: Real-time joint position feedback during execution
4. **Result**: Final status (success/failure) after completion
5. **Joint State Subscription**: Monitor actual joint positions from hardware
6. **Trajectory Message**: Describes path with waypoints, velocities, timing

**Example YAML Configuration (robot_controllers.yaml):**

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz (real-time loop frequency)

arm_controller:
  ros__parameters:
    joints:
      - shoulder
      - elbow
      - wrist
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    allow_partial_joints_goal: false
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.0  # No timeout for reaching goal
```

## Use Cases

### 1. Pick & Place Task
```python
# Move arm to approach position
approach_goal = {'shoulder': 0.3, 'elbow': 0.5, 'wrist': 0.0}
send_trajectory(approach_goal, duration_sec=2.0)

# Move to grasp position
grasp_goal = {'shoulder': 0.3, 'elbow': 0.7, 'wrist': 0.0}
send_trajectory(grasp_goal, duration_sec=1.0)

# Wait for gripper feedback, then retract
retract_goal = {'shoulder': 0.3, 'elbow': 0.5, 'wrist': 0.0}
send_trajectory(retract_goal, duration_sec=1.0)
```

### 2. Multi-Waypoint Path
```python
waypoints = [
    {'shoulder': 0.0, 'elbow': 0.0, 'wrist': 0.0},      # Home
    {'shoulder': 0.3, 'elbow': 0.4, 'wrist': -0.2},    # Position 1
    {'shoulder': 0.5, 'elbow': 0.3, 'wrist': 0.5},     # Position 2
    {'shoulder': 0.0, 'elbow': 0.0, 'wrist': 0.0},     # Return home
]

for goal_pos in waypoints:
    send_trajectory(goal_pos, duration_sec=2.0)
    time.sleep(2.5)  # Wait for execution + margin
```

### 3. Real-Time Impedance Control
```python
# Monitor feedback to detect contact forces
if contact_force_detected():
    # Reduce stiffness (compliance)
    send_soft_trajectory(goal_pos, stiffness=0.3)
else:
    # Full stiffness for precise movement
    send_trajectory(goal_pos)
```

## Debugging & Monitoring

### Check Controller Status
```bash
# List all controllers
ros2 control list_controllers

# Get controller details
ros2 control list_controller_types

# Check hardware interface status
ros2 service call /controller_manager/load_controller ros2controlcmd/srv/LoadController "{controller_name: arm_controller}"
```

### Monitor Joint States
```bash
# In a separate terminal
ros2 topic echo /joint_states
# Output: name: [shoulder, elbow, wrist], position: [0.0, 0.0, 0.0], ...
```

### Inspect Trajectory Execution
```bash
# Monitor feedback topic
ros2 topic echo /arm_controller/follow_joint_trajectory/_action/feedback
```

## Troubleshooting

**Error: `Could not find the requested 'arm_controller' controller`**
- Verify controller is loaded: `ros2 control list_controllers`
- Check robot launch file includes controller spawner
- Ensure YAML config is loaded: `ros2 param list`

**Error: `Goal rejected by server`**
- Controller may not be in ACTIVE state
- Check: `ros2 control list_controllers` (should show "active")
- Try: `ros2 service call /controller_manager/switch_controller ...`

**Trajectory executes slowly or incompletely**
- Verify update rate matches controller expectations (usually 100-1000 Hz)
- Check for network latency (local execution recommended)
- Monitor CPU usage on robot hardware

**Joint positions don't match trajectory commands**
- Verify joint velocity limits (check URDF and controller config)
- Check acceleration limits
- Ensure hardware can achieve commanded velocities

## Next Steps

1. **Multi-Arm Control**: Command multiple robot arms simultaneously
2. **Interactive Manipulation**: Use force feedback for compliant control
3. **Path Planning**: Generate trajectories with motion planning (MoveIt 2)
4. **Safety**: Implement joint limits, velocity saturation, collision avoidance
5. **Performance**: Measure control loop latency and optimize update rates

## Real-Time Control Loop Considerations

- **Update Rate**: Typical 100-1000 Hz for robot control
- **Determinism**: Hardware interface must be real-time capable (preempt_rt kernel)
- **Cycle Time**: Must be < period (e.g., 10ms for 100Hz, 1ms for 1000Hz)
- **Feedback**: Use QoS profile BEST_EFFORT for soft real-time

## References

- [ROS 2 Control Documentation](https://control.ros.org/)
- [Joint Trajectory Controller](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/index.html)
- [Hardware Interface Concept](https://control.ros.org/humble/doc/concept_articles/hardware_architecture/index.html)
- [Controller Manager](https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html)
- [Real-Time Linux (preempt_rt)](https://wiki.linuxfoundation.org/realtime/start)
