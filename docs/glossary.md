---
sidebar_position: 100
title: Glossary
description: Technical terms and concepts in Physical AI and Humanoid Robotics
---

# Glossary

Comprehensive glossary of terms used throughout the textbook.

## A

### Action Space
The set of all possible actions a robot or agent can take in an environment. In humanoid robotics, this includes joint positions, velocities, and torques.

### Actuator
A mechanical or electromechanical device that produces motion by converting energy into mechanical force. Common types include servo motors, stepper motors, and hydraulic cylinders.

### Agent (Reinforcement Learning)
An autonomous decision-making entity that learns to maximize cumulative rewards by interacting with an environment. In robotics, the agent's policy maps observations (sensor data, robot state) to actions (motor commands). Trained using algorithms like PPO, SAC, or TD3 in simulated or real environments.

## D

### Digital Twin
A virtual replica of a physical system that mirrors its behavior with validated accuracy. Digital twins enable simulation, testing, and optimization before deployment to real hardware. Three levels exist: Descriptive (geometric CAD model), Predictive (physics-based simulation), and Prescriptive (system-identified with parameters tuned to match real data). Used for training AI policies, failure prediction, and sim-to-real transfer validation.

### Domain Randomization
A technique to improve sim-to-real transfer by randomizing simulation parameters (mass, friction, lighting, textures) during training. By exposing the AI policy to diverse simulated conditions, it learns robust strategies that generalize to real-world variations. Prevents overfitting to simulation artifacts and improves zero-shot transfer performance.

### DDS (Data Distribution Service)
An Object Management Group (OMG) standard middleware protocol that provides real-time, scalable, peer-to-peer publish-subscribe communication. ROS 2 uses DDS (typically eProsima Fast DDS or Eclipse Cyclone DDS) as its underlying communication layer, replacing ROS 1's custom TCPROS/UDPROS protocol. DDS handles discovery, endpoint matching, quality of service (QoS) enforcement, and data serialization.

### Discovery (ROS 2)
The automatic process by which ROS 2 nodes find each other on the network without requiring a central master. DDS uses Simple Participant Discovery Protocol (SPDP) to announce node presence via UDP multicast, followed by Simple Endpoint Discovery Protocol (SEDP) to share information about publishers, subscribers, services, and actions. This decentralized discovery enables dynamic system composition and resilience to single points of failure.

## E

### Embodied AI
AI systems that perceive and act in the physical world through sensors and actuators. Unlike disembodied AI (language models, game agents), embodied AI must handle real-world physics constraints, sensor noise, and uncertainty. Examples include humanoid robots, autonomous vehicles, and manipulation systems. Often trained using vision-language-action (VLA) models or reinforcement learning.

### Embodied Intelligence
AI systems that interact with the physical world through sensors and actuators, learning through physical interaction rather than pure computation.

### Environment (Reinforcement Learning)
The simulated or physical world that an RL agent interacts with. Environments define the observation space (what the agent perceives), action space (what it can do), reward function (what it optimizes), and dynamics (how actions affect state). In Isaac Gym, environments are massively parallelized on GPU for fast training.

### Executor (ROS 2)
A ROS 2 component responsible for managing the execution of callbacks (timers, subscriptions, services, actions) within one or more nodes. The executor polls for ready callbacks and dispatches them according to a scheduling policy. ROS 2 provides SingleThreadedExecutor (default) and MultiThreadedExecutor for concurrent callback execution.

### Extension (Isaac Sim)
A modular plugin for NVIDIA Omniverse that adds custom functionality to Isaac Sim. Extensions can provide new UI panels, simulation tools, sensor models, or automation workflows. Written in Python or C++ using the Omniverse Kit SDK. The Isaac Sim ecosystem includes official extensions (e.g., Replicator, ROS 2 Bridge) and community-contributed extensions.

## F

### Fidelity (Simulation)
The degree to which a simulation accurately represents real-world physics, sensors, and dynamics. High-fidelity simulations (ray-traced sensors, deformable contact, stochastic noise) are slower but more realistic. Low-fidelity simulations (rigid bodies, point contact, deterministic) are faster for RL training but may have larger sim-to-real gaps. Goldilocks principle: use minimum fidelity that captures task-relevant physics.

## G

### Gazebo
An open-source 3D robot simulator with ROS integration. Gazebo Classic (version 11) uses ODE physics engine and SDFormat world files. Gazebo Harmonic (new version) uses modern architecture with plugin-based sensors, improved performance, and native ROS 2 support. Commonly used for testing navigation, manipulation, and multi-robot systems before hardware deployment.

### Graph (ROS 2)
The network of all ROS 2 nodes and their communication pathways (topics, services, actions, parameters). The ROS 2 graph is dynamic—nodes can join or leave at runtime, and connections are established automatically through DDS discovery. Use `ros2 node list`, `ros2 topic list`, and `rqt_graph` to introspect the graph structure.

## M

### Message (ROS 2)
A data structure used for communication between ROS 2 nodes. Messages are defined using `.msg` files with typed fields (e.g., `float64 temperature`, `string sensor_id`). Standard messages are provided in packages like `std_msgs`, `sensor_msgs`, and `geometry_msgs`. Custom messages can be defined for application-specific data. Messages are serialized to Common Data Representation (CDR) format for network transmission.

### Multimodal Learning
Training AI models on multiple data modalities simultaneously (vision, language, proprioception, audio). In robotics, multimodal learning enables richer representations that leverage complementary information sources. Vision-Language-Action (VLA) models are a key example, combining RGB images, text instructions, and robot actions for general-purpose manipulation.

## P

### PhysX
NVIDIA's GPU-accelerated physics engine used in Isaac Sim and Unity. PhysX 5 supports massive parallelization (thousands of actors), temporal Gauss-Seidel (TGS) contact solver, soft body simulation (FEM), and deterministic mode. Significantly faster than CPU-based engines (ODE, Bullet) for large-scale simulations. Used for digital twins, RL training, and game engines.

### Physical AI
A paradigm of artificial intelligence that incorporates the physical constraints and properties of real-world systems into the learning and decision-making process, emphasizing the interaction between an agent and its physical environment.

### Physics Engine
Software that simulates Newtonian dynamics, collision detection, and constraint solving for rigid and deformable bodies. Common engines include ODE (Gazebo), Bullet (PyBullet), PhysX (Unity/Isaac Sim), and MuJoCo. Physics engines use numerical integration (Euler, RK4) and constraint solvers (PGS, LCP) to compute realistic motion under forces, torques, contacts, and joint constraints.

### Physics-Informed Learning
Machine learning approaches that incorporate physical laws and constraints directly into the learning process, enabling AI systems to better understand and interact with the physical world.

### Planning (Hierarchical)
Multi-level robot control where high-level reasoning (task decomposition, symbolic planning) is combined with low-level execution (motion planning, control). Hierarchical planners use large language models (LLMs) to break instructions into subtasks, motion planners to generate collision-free trajectories, and RL policies to execute actions. Enables complex, long-horizon tasks.

### Policy (Reinforcement Learning)
A function that maps observations (sensor data, robot state) to actions (motor commands). Policies can be deterministic (fixed action for each state) or stochastic (probability distribution over actions). Represented as neural networks in deep RL, trained to maximize expected cumulative reward. Deployed to robots for autonomous control.

### Production Deployment
The process of moving AI/robotics systems from research/development to real-world operational environments. Production deployment requires robust safety systems, performance monitoring, error handling, and maintenance procedures. Key considerations include system reliability, safety compliance, and operational support.

### PPO (Proximal Policy Optimization)
A popular reinforcement learning algorithm that balances sample efficiency and stability by limiting policy updates. PPO uses clipped objective functions to prevent large, destabilizing gradient steps. Widely used for robot locomotion, manipulation, and navigation due to its robustness. Implemented in Isaac Gym for massively parallel training.

### Publisher (ROS 2)
A ROS 2 endpoint that sends messages to a topic. Created using `node.create_publisher(msg_type, topic_name, qos_profile)`. Publishers operate asynchronously—calling `publish()` returns immediately without waiting for subscribers. A single publisher can send to multiple subscribers, and multiple publishers can send to the same topic. Publishers are unaware of subscribers (loose coupling).

## Q

### QoS (Quality of Service)
A DDS concept adopted by ROS 2 that defines the behavioral contract between publishers and subscribers. QoS policies include Reliability (Best Effort vs. Reliable), Durability (Volatile vs. Transient Local), History (Keep Last N vs. Keep All), Deadline (max expected message period), and Lifespan (message validity duration). Publishers and subscribers must have compatible QoS policies to communicate, enabling fine-grained control over performance vs. reliability trade-offs.

## R

### Reinforcement Learning (RL)
A machine learning paradigm where an agent learns to maximize cumulative rewards through trial-and-error interaction with an environment. The agent observes states, takes actions, receives rewards, and updates its policy to improve performance. Used for robot locomotion, manipulation, and navigation. Key algorithms include PPO, SAC, and TD3.

### Replicator (Isaac Sim)
NVIDIA's synthetic data generation framework built into Isaac Sim. Replicator automates domain randomization, camera placement, lighting variation, and annotation capture (RGB, depth, semantic segmentation, bounding boxes). Uses a graph-based workflow (triggers, randomizers, writers) to generate large-scale datasets for training vision models.

### Reward Function
A scalar signal that guides reinforcement learning by specifying task objectives. Reward functions map state-action pairs to numerical values (higher is better). Designing effective rewards is critical—sparse rewards (success/failure only) are hard to learn from, while dense rewards (shaped progress signals) accelerate training but risk reward hacking. Often combines multiple weighted terms.

### ROS 2 (Robot Operating System 2)
A flexible framework for writing robot software that provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more for robotics applications.

### Rigid Body Dynamics
The study of the motion of solid objects (bodies) that do not deform, governed by Newton-Euler equations that describe how forces and torques affect motion.

### RGB-D Camera
A camera sensor that captures both color (RGB) and depth (D) information. Depth is measured using structured light (Kinect v1), time-of-flight (Kinect v2, RealSense), or stereo vision (ZED). Isaac Sim can simulate RGB-D cameras with realistic lens models, rolling shutter effects, and depth noise. Used for 3D perception, object detection, and SLAM.

## S

### SDF (Simulation Description Format)
An XML format used by Gazebo to describe worlds, models, and scenes. SDF files (.sdf) specify environments with multiple robots, static objects, lights, and physics parameters. More expressive than URDF (supports sensors, plugins, world-level configuration) but Gazebo-specific. Used in conjunction with URDF for robot descriptions.

### Sensorimotor Loop
The continuous cycle of sensing the environment, processing information, and generating motor actions that characterizes embodied intelligence systems.

### Sensor Fusion
Combining data from multiple sensors (cameras, LiDAR, IMU, encoders) to create a more accurate and robust perception of the environment. Fusion algorithms handle sensor noise, failure modes, and complementary information (e.g., camera for texture, LiDAR for geometry). Common methods include Kalman filtering, particle filtering, and learned fusion with deep networks.

### Sim-to-Real Transfer
The process of deploying policies or behaviors trained in simulation to real physical robots. Success depends on minimizing the sim-to-real gap through accurate physics modeling, domain randomization, and system identification. Zero-shot transfer (no real-world training) is the goal but often requires fine-tuning on hardware.

### Synthetic Data
Artificially generated training data created in simulation rather than collected from real sensors. Isaac Sim Replicator generates synthetic datasets with perfect ground-truth annotations (segmentation masks, bounding boxes, depth maps) for computer vision training. Overcomes data scarcity, enables edge case testing, and reduces labeling costs.

### Subscriber (ROS 2)
A ROS 2 endpoint that receives messages from a topic. Created using `node.create_subscription(msg_type, topic_name, callback, qos_profile)`. When a message arrives, the subscriber's callback function is invoked by the executor. Callbacks should be fast and non-blocking to avoid message queue overflow. Multiple subscribers can listen to the same topic independently.

### Simulation-to-Reality Gap
The difference between how a system behaves in simulation versus in the real world, which can cause problems when transferring learned behaviors from simulated to physical environments.

### System Identification
The process of tuning simulation parameters (mass, inertia, friction, damping) to match observed real-world behavior. Involves collecting data from physical robot (joint trajectories, contact forces), running identical motions in simulation, and optimizing parameters to minimize error. Essential for creating prescriptive digital twins with validated accuracy.

### System Integration
The process of combining multiple subsystems (perception, planning, control, safety) into a cohesive, functioning whole. System integration requires careful interface design, testing of interactions between components, and validation that the integrated system meets all requirements. Critical for capstone projects and real-world robotics deployments.

### State-Space Representation
A mathematical model of a physical system as a set of input, output, and state variables related by first-order differential equations.

### System Identification (Legacy Definition)
The process of determining a mathematical model of a system based on observed input-output data, often used to improve simulation accuracy.

## U

### Unity
A cross-platform game engine with high-fidelity graphics and PhysX physics engine. Unity 2022.x LTS integrates with ROS 2 via TCP/UDP bridges (ros-tcp-connector, ros2-for-unity packages), enabling realistic visual simulation for human-in-the-loop testing, VR/AR applications, and synthetic dataset generation. Supports GPU-accelerated ray tracing and real-time rendering.

### URDF (Unified Robot Description Format)
An XML format for describing robot kinematic and dynamic properties. URDF files specify links (rigid bodies with mass, inertia, collision/visual geometry) and joints (revolute, prismatic, fixed, continuous) that connect them. Used by ROS, Gazebo, and MuJoCo for robot modeling. XACRO (XML Macros) extends URDF with parameters and reusable macros for complex robots.

### USD (Universal Scene Description)
Pixar's open-source file format for describing 3D scenes with composition, layering, and lazy loading. Isaac Sim uses USD as its native format for robots, environments, and assets. USD enables non-destructive edits (layers), collaborative workflows (multiple users editing simultaneously), and efficient streaming (only load visible geometry). Key USD file types: `.usd` (binary), `.usda` (ASCII), `.usdc` (crate).

## T

### Topic (ROS 2)
A named communication channel in the ROS 2 graph used for many-to-many asynchronous message passing. Topics have a name (e.g., `/camera/image_raw`), a message type (e.g., `sensor_msgs/Image`), and associated QoS policies. Nodes publish to and subscribe to topics without direct coupling. Topics follow a hierarchical naming convention using forward slashes (e.g., `/robot1/sensors/imu`).

### Transfer Learning
Reusing knowledge from one task/domain to improve learning in a related task/domain. In robotics, transfer learning enables sim-to-real deployment (train in Isaac Sim, deploy to hardware) and task generalization (train on object A, adapt to object B). Foundation models (VLAs, LLMs) enable zero-shot transfer to novel tasks without retraining.

## N

### Node (ROS 2)
The fundamental computational unit in ROS 2—a self-contained process that performs a specific task (e.g., sensor driver, controller, planner). Nodes communicate via topics, services, and actions. Each node has a unique name within its namespace, and can be written in Python (rclpy), C++ (rclcpp), or other supported languages. Nodes can be composed into a single process for performance or run as separate processes for isolation.

## K

### Kinematics
The branch of mechanics that describes the motion of points, bodies (objects), and systems of bodies (groups of objects) without considering the forces that cause them to move.

### Forward Kinematics
The process of determining the position and orientation of a robot's end-effector based on the known joint angles.

### Inverse Kinematics
The process of determining the joint angles required to achieve a desired position and orientation of a robot's end-effector.

## C

### Capstone Project
A culminating academic or professional project that integrates knowledge and skills from multiple domains to solve a complex, real-world problem. In robotics education, capstone projects typically involve designing, implementing, and validating complete autonomous systems that demonstrate mastery of fundamental concepts and advanced techniques.

### Control Theory
A branch of engineering and mathematics that deals with the behavior of dynamical systems with inputs, and how their behavior is modified by feedback.

### Cross-Modal Attention
A neural network mechanism that allows different modalities (e.g., vision and language) to attend to relevant information in each other. In Vision-Language-Action (VLA) models, cross-modal attention enables visual features to influence language processing and vice versa, creating unified representations for robotic action generation.

### PID Control (Proportional-Integral-Derivative Control)
A control loop mechanism employing feedback that calculates an error value as the difference between a desired setpoint and a measured process variable.

### State Feedback Control
A control system design approach where the control input is computed as a function of the system's state variables.

## F

### Feedback (Actions)
Information sent from an action server to an action client during the execution of a goal. Feedback allows clients to monitor the progress of long-running tasks without waiting for completion, providing real-time status updates (e.g., percentage complete, current state, intermediate measurements). Essential for interactive and responsive autonomous systems.

### Frame (Coordinate Frame)
A reference system in 3D space defined by an origin, three orthogonal axes, and a name. Robots use multiple frames (e.g., `base_link`, `camera_link`, `world`) to describe positions and orientations of sensors, actuators, and objects. TF2 manages transformations between frames, enabling coordinate system conversions for sensor fusion and control.

## G

### Goal (Actions)
A request sent by an action client to an action server to perform a long-running task. Goals specify what needs to be accomplished but allow the server to process it asynchronously. The action server provides feedback and eventually returns a result. Unlike services (which block), goals enable non-blocking, cancellable, long-duration operations.

## L

### LiDAR (Light Detection and Ranging)
A remote sensing method that uses laser pulses to measure distances. LiDAR sensors emit laser beams and measure time-of-flight to create 3D point clouds. Used for mapping, obstacle detection, and localization. Isaac Sim supports rotating LiDAR (Velodyne-style) and solid-state LiDAR simulation with configurable range, resolution, and noise models.

### Listener (TF2)
A ROS 2 component that receives and buffers published transforms. The TF2 listener maintains a transform history, allowing applications to query the relationship between any two frames at any time within the buffer window. Essential for sensor fusion and coordinate frame conversions in multi-sensor robotic systems.

## R

### Result (Actions)
The final output or outcome returned by an action server when a goal has completed, either successfully or with failure. Results provide terminal state information and any final data (e.g., final position, error status, computed plan). Combined with feedback, results allow clients to track both progress and completion of asynchronous tasks.

## T

### Transform (TF2)
A mathematical representation of the position and orientation (6D pose) of one coordinate frame relative to another, consisting of a translation vector (x, y, z) and a rotation quaternion (x, y, z, w). TF2 manages transforms between frames, enabling robots to reason about sensor and actuator positions in multiple reference systems.

### TF2 (Transform Library)
A ROS 2 library that maintains and queries transformations between coordinate frames. TF2 handles the temporal interpolation, buffering, and graph resolution of frame relationships, enabling applications to work with multiple sensors and actuators without explicitly managing coordinate conversions.

## B

### Broadcaster (TF2)
A ROS 2 component that publishes coordinate frame transformations. Broadcasters publish both static transforms (fixed mounting relationships) and dynamic transforms (changing positions). The TF2 `TransformBroadcaster` and `StaticTransformBroadcaster` classes are used to send transforms to the TF2 tree for use by listeners.

## S

### Service (ROS 2)
A synchronous, request-reply communication pattern in ROS 2 where a service client sends a request to a service server and blocks (waits) for a response. Services are used for quick, deterministic operations (e.g., parameter updates, database queries) unlike topics (asynchronous) and actions (long-running). Each service has a unique name, request type, and response type, and DDS handles transport and serialization.

## C

### Controller (ROS 2 Control)
A plugin that implements a control algorithm in the ROS 2 Control framework. Controllers read from state interfaces (e.g., joint positions) and write to command interfaces (e.g., joint velocities) at a fixed rate. Common controllers include JointTrajectoryController, DiffDriveController, and GripperActionController. Controllers follow a managed lifecycle (inactive → configured → active).

### Controller Manager (ROS 2 Control)
The central orchestrator in ROS 2 Control that loads, configures, activates, and deactivates controllers. The Controller Manager runs the real-time control loop, calling `read()` on hardware interfaces, `update()` on all active controllers, and `write()` to send commands to actuators. It enforces controller resource claims (preventing conflicts) and provides lifecycle management services.

### Control Loop
The cyclic process of sensing (read), computing (control algorithm), and actuating (write) that runs at a fixed frequency (typically 100-1000 Hz for robots). In ROS 2 Control, the Controller Manager executes this loop: Hardware Interface `read()` → Controller `update()` → Hardware Interface `write()`. Loop timing determinism is critical for stable control.

## H

### Hardware Interface (ROS 2 Control)
An abstraction layer that provides uniform access to robot actuators and sensors regardless of the underlying hardware (real robot, simulator, mock hardware). Hardware interfaces expose state interfaces (readable sensor data) and command interfaces (writable actuator commands). Developers implement `read()` and `write()` methods to communicate with physical hardware drivers.

### Humanoid Robot
A bipedal robot with human-like form factor, typically featuring a head, torso, two arms, and two legs. Humanoid robots have 30+ degrees of freedom and are designed for human environments. Examples include Boston Dynamics Atlas, Honda ASIMO, and Unitree H1. Used for research, entertainment, and assistive applications.

### Humanoid Control
Control strategies specifically designed for humanoid robots with 30+ degrees of freedom. Humanoid control must address challenges of bipedal locomotion, balance maintenance, and coordinated manipulation. Requires whole-body control approaches that handle multiple simultaneous tasks while respecting joint limits and contact constraints.

## J

### Joint (Robotics)
A mechanical connection between two rigid links that allows relative motion. Common joint types include revolute (rotational, like elbow), prismatic (linear, like telescoping), and fixed (no motion). Each non-fixed joint has associated state variables (position, velocity, effort/torque) and may accept commands from controllers.

### Joint State
The current configuration of a robot joint, typically represented as position (angle in radians or distance in meters), velocity (rad/s or m/s), and effort (torque in N·m or force in N). Joint states are published on the `/joint_states` topic using the `sensor_msgs/JointState` message type, enabling visualization and monitoring.

### Joint Trajectory
A time-parameterized sequence of joint positions, velocities, and accelerations that defines a desired motion path for a robot. Represented by `trajectory_msgs/JointTrajectory`, which contains an array of waypoints (JointTrajectoryPoint) each with timestamps and target joint values. Used by JointTrajectoryController to execute smooth, coordinated multi-joint movements.

## I

### Interface (ROS 2 Control)
A named data channel for reading sensor data (state interface) or writing actuator commands (command interface). Interfaces are typed (e.g., "position", "velocity", "effort") and belong to specific joints or sensors. Controllers claim exclusive access to command interfaces to prevent conflicts.

### Isaac Gym
NVIDIA's GPU-accelerated RL training environment that enables thousands of parallel physics simulations on a single GPU. Unlike Isaac Sim (which uses PhysX 5), Isaac Gym uses a specialized tensor-based physics engine optimized for batched environments. Achieves 100-1000x speedups over CPU-based simulators for policy training. Used for locomotion, manipulation, and dexterous manipulation research.

### Isaac Sim
NVIDIA's robotics simulation platform built on Omniverse. Combines photorealistic RTX ray tracing, PhysX 5 GPU physics, synthetic data generation (Replicator), and ROS 2 integration. Supports USD scene format, collaborative workflows via Nucleus server, and Python/C++ APIs. Used for digital twins, sim-to-real transfer, and autonomous system testing.

## R

### Real-Time Control
Control loops that must meet strict timing deadlines to ensure system stability and safety. For robots, this typically means executing the sense-compute-actuate cycle at a fixed frequency (100-1000 Hz) with minimal jitter (less than 1 ms). ROS 2 Control supports real-time execution when used with PREEMPT_RT kernel patches and real-time-safe controllers.

## T

### Trajectory (Robotics)
A time-ordered path through configuration space (joint space or task space) that specifies both the geometric path and the velocity profile. Trajectories ensure smooth motion by defining acceleration limits, avoiding discontinuous jumps that could damage hardware or destabilize control. Represented as sequences of waypoints with timestamps.

## O

### Omniverse
NVIDIA's platform for 3D collaboration, simulation, and USD-based content creation. Omniverse provides Nucleus (cloud asset server), Connectors (plugins for Maya/Blender/Unreal), and Apps (Isaac Sim, Create, Code). Enables real-time multi-user editing of shared 3D scenes with live sync. Used for digital twin workflows, virtual production, and robotics simulation.

## V

### Vision-Language-Action (VLA)
A class of multimodal AI models that map visual observations and natural language instructions directly to robot actions. VLAs are trained end-to-end on large datasets of (image, text, action) tuples, enabling zero-shot generalization to novel objects and tasks. Examples include RT-2, PaLM-E, and OpenVLA. Integrate with Isaac Sim for sim-to-real evaluation.

### Vision-Language Model (VLM)
AI models that process both visual (images/video) and textual (language) inputs to perform tasks like image captioning, visual question answering, and grounding. When extended with action prediction capabilities, VLMs become VLAs for embodied AI. Foundation VLMs include CLIP, BLIP, Flamingo, and GPT-4 Vision.

## W

### Whole-Body Control
A control approach that coordinates multiple tasks (balance, manipulation, locomotion) simultaneously using optimization. Whole-body controllers manage the redundancy of humanoid robots (30+ DOF) by prioritizing tasks and computing optimal joint commands that satisfy all constraints. Uses quadratic programming or other optimization techniques.

## Z

### Zero-Shot Transfer
Deploying a policy or model to a new task/environment without any training on that specific scenario. Achieved through robust training (domain randomization), foundation models (VLAs with broad priors), or accurate simulation (high-fidelity digital twins). The gold standard for sim-to-real transfer but often requires fine-tuning in practice.

---

**Note**: This glossary covers terminology from all modules (Modules 0-4: Foundations, ROS 2, Digital Twin, NVIDIA Isaac, VLA Humanoid Robotics) and the Capstone project. The complete textbook provides comprehensive coverage of Physical AI and Humanoid Robotics from fundamentals to advanced deployment.
