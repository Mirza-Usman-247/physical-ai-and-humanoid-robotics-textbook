# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

**Feature**: 001-physical-ai-textbook
**Date**: 2025-12-06
**Status**: Phase 1 Complete

## Overview

This guide provides step-by-step instructions for developers and content authors working on the Physical AI & Humanoid Robotics Docusaurus textbook. Follow these workflows to set up your environment, create chapters, validate content, and deploy to GitHub Pages.

---

## Prerequisites

**Required Software**:
- **Node.js** 18+ ([Download](https://nodejs.org/))
- **Git** ([Download](https://git-scm.com/))
- **Docker** (for code validation) ([Download](https://www.docker.com/))
- **VS Code** or preferred editor ([Download](https://code.visualstudio.com/))

**Optional (for local code testing)**:
- **ROS 2 Humble** ([Installation Guide](https://docs.ros.org/en/humble/Installation.html))
- **Python 3.10+** with pip
- **NVIDIA GPU** (RTX 3060+ for Isaac Sim/Unity examples)

**Recommended VS Code Extensions**:
- MDX (syntax highlighting)
- Prettier (code formatting)
- ESLint (TypeScript linting)
- Docker (container management)

---

## Initial Setup

### 1. Clone Repository

```bash
git clone https://github.com/your-org/humanoid-physical-ai-textbook.git
cd humanoid-physical-ai-textbook
```

### 2. Install Dependencies

```bash
npm install
```

This installs:
- Docusaurus 3.x core + classic theme
- React 18
- TypeScript 5.x
- Algolia DocSearch client
- FlexSearch (glossary indexing)
- ajv (JSON Schema validation)
- Lighthouse CI
- Link checker

### 3. Start Development Server

```bash
npm start
```

Opens browser at `http://localhost:3000` with live reload enabled.

**Available Scripts**:
- `npm start` - Dev server with hot reload
- `npm run build` - Production build (validates all schemas)
- `npm run serve` - Serve production build locally
- `npm run clear` - Clear Docusaurus cache
- `npm run optimize-images` - Compress images in `/static/img/`
- `npm run validate-metadata` - Run JSON Schema validation only
- `npm run check-links` - Check for broken links
- `npm run lighthouse` - Run Lighthouse audit locally

---

## Chapter Creation Workflow

### Step 1: Create Chapter Directory Structure

```bash
# Example: Create Module 1, Chapter 2
mkdir -p docs/module-1-ros2/chapter-2-pub-sub
cd docs/module-1-ros2/chapter-2-pub-sub
```

### Step 2: Create `index.mdx` with Frontmatter

Create `index.mdx` and add frontmatter (validated against `chapter-metadata-schema.json`):

```mdx
---
title: "Publisher-Subscriber Communication in ROS 2"
description: "Learn ROS 2 pub/sub architecture, create publishers and subscribers, and understand DDS Quality of Service policies for reliable robot communication."
sidebar_position: 2
keywords:
  - ros2
  - publisher
  - subscriber
  - dds
  - qos
  - middleware
  - communication
module: "module-1"
week_mapping: "Week 3 – Tuesday & Thursday"
prerequisites:
  - type: internal
    title: "ROS 2 Architecture Overview"
    link: "/docs/module-1/chapter-1"
  - type: external
    title: "Python basics"
    description: "Functions, classes, decorators"
learning_objectives:
  - "Explain ROS 2 publish-subscribe communication pattern and DDS middleware role"
  - "Create custom message types using ROS 2 interface definition language"
  - "Implement publishers and subscribers in Python with proper error handling"
  - "Configure Quality of Service (QoS) policies for reliable communication"
assessment_type: "implementation"
difficulty_level: "intermediate"
capstone_component: "Sensor data streaming for perception pipeline integration"
code_dependencies:
  - "rclpy==3.3.11"
  - "std_msgs==4.2.3"
  - "custom_interfaces==0.1.0"
estimated_time: 4
---

<!-- Chapter content starts here -->
```

**Frontmatter Validation**:
- All fields are validated during `npm run build`
- Build fails if any field violates schema (type, format, enum, min/max)
- See `/specs/001-physical-ai-textbook/contracts/chapter-metadata-schema.json` for full schema

### Step 3: Write 12 Required Sections

Every chapter MUST include these sections in order:

1. **Frontmatter** ✅ (added above)
2. **Learning Objectives**
3. **Prerequisites**
4. **Weekly Mapping**
5. **Motivating Scenario**
6. **Core Theory & Mathematics**
7. **Worked Example**
8. **Hands-On Code**
9. **Application to Humanoid Robots**
10. **Common Pitfalls & Debugging**
11. **Exercises**
12. **Further Reading & References**

**Example Structure**:

```mdx
## Learning Objectives

By the end of this chapter, you will be able to:

- Explain ROS 2 publish-subscribe communication pattern
- Create custom message types using interface definition language
- Implement publishers and subscribers in Python
- Configure QoS policies for reliable communication

## Prerequisites

Before starting, ensure you understand:

- <Link to="/docs/module-1/chapter-1">ROS 2 Architecture Overview</Link>
- Python basics (functions, classes, decorators)
- Linux command-line navigation

## Weekly Mapping

**Week 3 – Tuesday & Thursday**

This chapter is designed for two 90-minute sessions:
- **Tuesday**: Sections 1-6 (concepts, theory, worked example)
- **Thursday**: Sections 7-11 (hands-on code, exercises)

## Motivating Scenario

Imagine a humanoid robot navigating a warehouse. The robot's camera publishes RGB images at 30 FPS, LiDAR publishes point clouds at 10 Hz, and IMU publishes orientation at 100 Hz. How do we efficiently route this sensor data to perception, planning, and control nodes?

ROS 2's publish-subscribe pattern provides the answer...

## Core Theory & Mathematics

### Publish-Subscribe Pattern

In ROS 2, nodes communicate asynchronously through **topics**. A topic is a named channel for typed messages.

**Mathematical Model**:

Let $P$ = set of publishers, $S$ = set of subscribers, $T$ = set of topics.

$$
\text{Message flow: } P_i \xrightarrow{T_j} S_k
$$

Where publisher $P_i$ sends message $m$ on topic $T_j$, and all subscribers $S_k$ listening to $T_j$ receive $m$.

**DDS Middleware**:

ROS 2 uses Data Distribution Service (DDS) for message transport. Key properties:

- **Decoupling**: Publishers/subscribers don't know each other's identity
- **Many-to-many**: Multiple publishers and subscribers per topic
- **Type safety**: Messages validated against schema at compile time

[Continue with derivations, equations, diagrams...]

## Worked Example

Let's create a simple temperature sensor publisher and monitoring subscriber.

**Step 1**: Define custom message type...

[Step-by-step solution]

## Hands-On Code

### Simple Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TemperatureSensor(Node):
    def __init__(self):
        super().__init__('temperature_sensor')
        # Why: Create publisher on 'temperature' topic with Float32 messages
        self.publisher = self.create_publisher(Float32, 'temperature', 10)
        # Why: Timer ensures regular publishing at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_temperature)

    def publish_temperature(self):
        msg = Float32()
        msg.data = 25.0  # Simulated temperature in Celsius
        # Why: Log confirms message published successfully
        self.get_logger().info(f'Publishing: {msg.data}°C')
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = TemperatureSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Safety Note**: ⚠️ Always run ROS 2 examples in simulation first before hardware deployment.

For complex multi-file examples, see `/examples/module-1-ros2/chapter-2-pub-sub/`.

## Application to Humanoid Robots

Humanoid robots like Boston Dynamics' Atlas use pub/sub extensively:

- **Sensor fusion**: IMU, cameras, force-torque sensors publish to perception nodes
- **Locomotion**: Footstep planner publishes trajectories to joint controllers
- **Safety**: Emergency stop broadcasts to all actuators simultaneously

Real-world example: Atlas uses ~50 topics for full-body control...

## Common Pitfalls & Debugging

### Pitfall 1: QoS Mismatch

**Problem**: Subscriber doesn't receive messages despite correct topic name.

**Cause**: Publisher uses `RELIABLE` QoS, subscriber uses `BEST_EFFORT`.

**Solution**: Match QoS profiles:
```python
qos_profile = rclpy.qos.QoSProfile(
    reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
    depth=10
)
self.publisher = self.create_publisher(Float32, 'topic', qos_profile)
```

### Pitfall 2: Message Rate Too High

**Problem**: Subscriber drops messages, system becomes unresponsive.

**Cause**: Publishing at 1000 Hz without proper buffer configuration.

**Solution**: Use appropriate QoS depth and optimize callback processing...

[Continue with 3-5 common pitfalls]

## Exercises

### Exercise 1: Conceptual (Estimated time: 20 minutes)

**Question**: Explain why ROS 2's pub/sub pattern is better suited for robot systems than direct function calls between nodes.

**Assessment Rubric**:
- **Novice**: Mentions decoupling but lacks detail
- **Proficient**: Discusses decoupling, scalability, fault tolerance
- **Advanced**: Analyzes trade-offs (latency vs flexibility), compares to alternatives (RPC, shared memory)

### Exercise 2: Computational (Estimated time: 30 minutes)

**Problem**: A LiDAR publishes 360-point scans at 10 Hz. Each point is 12 bytes (x, y, z as float32). Calculate:
1. Message size in bytes
2. Network bandwidth (bytes/sec)
3. Buffer depth needed for 2-second history

[Continue with implementation exercises...]

## Further Reading & References

1. **ROS 2 Design Documentation** (2023). "Quality of Service Policies". https://design.ros2.org/articles/qos.html

2. Smith, J. et al. (2022). "Efficient Message Passing in Distributed Robot Systems". *IEEE Robotics & Automation Letters*, 7(2), 1234-1241. DOI: 10.1109/LRA.2022.1234567

3. Object Management Group (2015). "Data Distribution Service (DDS) Specification v1.4". https://www.omg.org/spec/DDS/

[Continue with 5+ references...]
```

### Step 4: Add Code Examples

**For Simple Snippets (≤20 lines)**: Embed directly in MDX (see Hands-On Code section above)

**For Complex Projects**: Create `/examples/module-1-ros2/chapter-2-pub-sub/`

```bash
mkdir -p examples/module-1-ros2/chapter-2-pub-sub
cd examples/module-1-ros2/chapter-2-pub-sub
```

Create files:
- `simple_publisher.py`
- `simple_subscriber.py`
- `requirements.txt` (with pinned versions)
- `README.md` (setup instructions)
- `run_demo.sh` (automated runner script)

**requirements.txt Example**:
```
rclpy==3.3.11
std_msgs==4.2.3
```

### Step 5: Add Diagrams (if applicable)

```bash
# Create image directory
mkdir -p static/img/module-1-ros2/chapter-2-pub-sub

# Add diagrams (≤500KB each)
# Use PNG or SVG format
```

**Image Requirements**:
- ≤500KB per image (enforced by CI)
- Meaningful alt-text for accessibility
- Clear labels and legends

### Step 6: Update Sidebar Configuration

Edit `docs/module-1-ros2/_category_.json`:

```json
{
  "label": "Module 1: ROS 2 for Physical AI",
  "position": 1,
  "link": {
    "type": "generated-index",
    "title": "Module 1: ROS 2 for Physical AI",
    "description": "Master ROS 2 Humble architecture, communication patterns, and robot control.",
    "keywords": ["ros2", "middleware", "robot operating system"]
  },
  "collapsible": true,
  "collapsed": false,
  "className": "sidebar-module-1"
}
```

### Step 7: Validate Locally

```bash
# Build site (validates frontmatter schemas)
npm run build

# Check for broken links
npm run check-links

# Validate metadata only
npm run validate-metadata

# Run Lighthouse audit
npm run lighthouse

# Optimize images
npm run optimize-images
```

**Expected Output**:
```
✅ Build successful
✅ Frontmatter validation: PASS
✅ 0 broken links
✅ Lighthouse score: 95 (≥90 required)
✅ All images ≤500KB
```

### Step 8: Test Code Examples in Docker

```bash
# Build validation container (if not exists)
docker build -t textbook-code-validator -f .docker/code-validator.Dockerfile .

# Run code validation
docker run --rm \
  -v $(pwd)/examples:/examples \
  textbook-code-validator \
  /examples/module-1-ros2/chapter-2-pub-sub
```

**Expected Output**:
```
Running: simple_publisher.py
✅ PASS: Script executed successfully
Running: simple_subscriber.py
✅ PASS: Script executed successfully
All code examples validated: 2/2 PASS
```

### Step 9: Commit and Push

```bash
git add docs/module-1-ros2/chapter-2-pub-sub/
git add examples/module-1-ros2/chapter-2-pub-sub/
git add static/img/module-1-ros2/chapter-2-pub-sub/
git commit -m "Add Module 1 Chapter 2: Publisher-Subscriber Communication"
git push origin 001-physical-ai-textbook
```

**CI/CD Automatically Runs**:
1. Docusaurus build validation
2. Frontmatter schema validation
3. Docker code execution tests
4. Image optimization + size check
5. Broken link checker
6. Lighthouse audit (≥90 required)

If all checks pass, GitHub Actions deploys to GitHub Pages.

---

## Glossary Update Workflow

### 1. Edit `docs/glossary.md`

Add new term in alphabetical order:

```markdown
## Inverse Kinematics

**Definition**: The mathematical process of calculating joint angles required to position a robot's end-effector at a desired location and orientation in space.

**Related Terms**: [Forward Kinematics](#forward-kinematics), [Jacobian Matrix](#jacobian-matrix), [End-Effector](#end-effector)

**Aliases**: IK

**Used In**: [Module 0 Chapter 3](/docs/module-0/chapter-3), [Module 1 Chapter 5](/docs/module-1/chapter-5), [Capstone](/docs/capstone/chapter-1)

**Category**: Robotics
```

### 2. Rebuild FlexSearch Index

FlexSearch index is automatically rebuilt during `npm run build`. No manual action required.

### 3. Test Search Locally

```bash
npm start
# Navigate to http://localhost:3000/glossary
# Test search functionality
```

---

## CI Checks Reference

**GitHub Actions runs these checks on every PR**:

| Check | Tool | Pass Criteria | Fix Command |
|-------|------|---------------|-------------|
| **Build** | Docusaurus | Zero errors | Fix syntax/frontmatter |
| **Metadata Validation** | ajv | All schemas valid | Update frontmatter |
| **Code Validation** | Docker | 90% examples pass | Fix code, update deps |
| **Link Checker** | broken-link-checker | Zero broken links | Fix URLs |
| **Lighthouse** | Lighthouse CI | Score ≥90 | Optimize images/code |
| **Image Optimization** | sharp | All images ≤500KB | Run `npm run optimize-images` |

**View CI Results**:
- GitHub Actions tab in repository
- PR checks section (✅ or ❌)
- Detailed logs for each workflow

---

## Troubleshooting

### Build Failed: Frontmatter Schema Violation

**Error**:
```
ERROR: Validation failed for docs/module-1/chapter-2/index.mdx
  - Field 'learning_objectives' must have 3-7 items (found 2)
  - Field 'code_dependencies' must match pattern package==version
```

**Solution**:
1. Check error message for specific field
2. Review schema: `/specs/001-physical-ai-textbook/contracts/chapter-metadata-schema.json`
3. Fix frontmatter in MDX file
4. Run `npm run validate-metadata` to verify

### Code Example Failed in Docker

**Error**:
```
Running: ros2_example.py
ERROR: ModuleNotFoundError: No module named 'rclpy'
```

**Solution**:
1. Check `requirements.txt` has pinned version: `rclpy==3.3.11`
2. Verify Docker image has ROS 2 Humble installed
3. Test locally: `python3 ros2_example.py`

### Image Too Large (>500KB)

**Error**:
```
ERROR: static/img/module-1/diagram.png exceeds 500KB (actual: 832KB)
```

**Solution**:
```bash
# Optimize single image
npm run optimize-images -- static/img/module-1/diagram.png

# Optimize all images
npm run optimize-images

# Manual optimization
# Use online tools: TinyPNG, ImageOptim, Squoosh
```

### Broken Link Detected

**Error**:
```
ERROR: Broken link in docs/module-1/chapter-2/index.mdx
  - [ROS 2 Docs](https://nonexistent-url.com) → 404
```

**Solution**:
1. Verify URL exists and is accessible
2. Update link in MDX file
3. Run `npm run check-links` locally
4. For internal links, use relative paths: `/docs/module-1/chapter-1`

---

## MCP Context-7 Integration

**When to Call Context-7**:
- **EVERY chapter generation**: Retrieve documentation for ROS 2, Isaac Sim, Unity, Gazebo, Whisper
- **Docusaurus site modifications**: Retrieve Docusaurus 3.x docs when creating custom components, plugins, or config changes

**Batch Retrieval Workflow** (Recommended):
At the start of EVERY chapter generation, retrieve ALL relevant tool documentations in a single batch call to optimize efficiency while maintaining per-chapter freshness guarantee (per FR-026).

**Example Batch Context-7 Usage**:
```bash
# Before writing any chapter - retrieve all tool docs in single batch
# This ensures fresh documentation while minimizing API calls
mcp-context7 fetch --batch \
  --library ros2-humble \
  --library isaac-sim-4.x \
  --library unity-2022-lts \
  --library gazebo-harmonic \
  --library whisper

# Single-tool retrieval for focused updates
mcp-context7 fetch --library ros2-humble --topic publisher-subscriber

# Before creating custom Docusaurus component
mcp-context7 fetch --library docusaurus-3.x --topic react-components
```

**Version Pinning**:
- ROS 2 Humble (latest within Humble LTS)
- Isaac Sim 4.x (latest within v4)
- Unity 2022.x LTS (latest within 2022 LTS)
- Docusaurus 3.x (latest stable within v3)

---

## Incremental Publishing

**Staging Environment** (all modules):
```bash
PUBLISH_MODULES=all npm run build
```

**Production Phased Rollout**:
```bash
# Week 1-2: Module 0 only
PUBLISH_MODULES=0 npm run build

# Week 3-5: Module 0 + Module 1
PUBLISH_MODULES=0,1 npm run build

# Full release
PUBLISH_MODULES=0,1,2,3,4,capstone npm run build
```

Configure in GitHub Actions deploy workflow:
```yaml
env:
  PUBLISH_MODULES: ${{ secrets.PUBLISH_MODULES || 'all' }}
```

---

## Additional Resources

- **Docusaurus Documentation**: https://docusaurus.io/docs
- **ROS 2 Humble Docs**: https://docs.ros.org/en/humble/
- **FlexSearch**: https://github.com/nextapps-de/flexsearch
- **JSON Schema**: https://json-schema.org/
- **Lighthouse CI**: https://github.com/GoogleChrome/lighthouse-ci

---

**Quickstart Version**: 1.0.0
**Last Updated**: 2025-12-06
**Status**: Complete (ready for use)
