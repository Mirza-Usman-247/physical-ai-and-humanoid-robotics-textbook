# Chapter 2 Diagrams - Publisher-Subscriber Pattern

This directory contains Mermaid diagrams for Chapter 2: Publisher-Subscriber Pattern in ROS 2.

## Diagrams

### 1. `pubsub-pattern.mermaid` - Basic Pub/Sub Architecture

**Purpose:** Illustrate the fundamental publisher-subscriber pattern with multiple subscribers.

**Concepts Shown:**
- **1:N Communication**: One publisher, multiple subscribers
- **Topic as Intermediary**: `/chatter` topic decouples publishers from subscribers
- **Asynchronous Callbacks**: Each subscriber processes messages independently
- **QoS Configuration**: All endpoints use Reliable QoS for guaranteed delivery
- **Data Flow Steps**: Timer trigger → Publish → DDS delivery → Callback execution

**Learning Objectives:**
- Understand decoupled communication (publishers don't know about subscribers)
- Visualize how one message is delivered to multiple subscribers
- See the role of timers in periodic publishing
- Learn callback-based message processing

**Teaching Points:**
- Adding/removing subscribers doesn't affect publisher code
- Message delivery is parallel (not sequential)
- Each subscriber receives its own copy of the message

---

### 2. `qos-comparison.mermaid` - QoS Profiles for Different Use Cases

**Purpose:** Compare three common QoS patterns and their matching rules.

**Profiles Compared:**

#### Profile 1: Sensor Data (Best Effort)
- **Reliability**: BEST_EFFORT
- **Use Cases**: Camera streams, LiDAR, IMU, high-frequency odometry
- **Trade-offs**: Low latency, lower bandwidth ✅ | May lose messages ❌

#### Profile 2: Command Data (Reliable)
- **Reliability**: RELIABLE
- **Use Cases**: Velocity commands, joint targets, emergency stop
- **Trade-offs**: Guaranteed delivery ✅ | Higher latency ❌

#### Profile 3: Configuration (Transient Local)
- **Durability**: TRANSIENT_LOCAL
- **Use Cases**: Robot description, map data, parameters
- **Trade-offs**: Late-joiner support ✅ | Memory overhead ❌

**QoS Matching Rules:**
- Best Effort ↔ Best Effort: ✅ Compatible
- Reliable ↔ Reliable: ✅ Compatible
- Best Effort (pub) ↔ Reliable (sub): ❌ INCOMPATIBLE
- Reliable (pub) ↔ Best Effort (sub): ✅ Compatible (subscriber relaxes requirement)

**Learning Objectives:**
- Map QoS settings to real-world robot communication patterns
- Understand performance vs. reliability trade-offs
- Learn QoS compatibility rules to debug connection issues
- Identify when to use each profile type

**Common Pitfall:**
Students often try to subscribe with RELIABLE to a BEST_EFFORT publisher (e.g., standard sensor drivers). This fails silently. Solution: Query topic QoS with `ros2 topic info --verbose` and match subscriber accordingly.

---

### 3. `message-lifecycle.mermaid` - Detailed Message Flow Sequence

**Purpose:** Show the complete lifecycle of a message from publication to callback execution, including error handling.

**Sequence Steps:**

1. **Publication**:
   - Timer callback triggers
   - Message created and populated
   - `publish()` called

2. **DDS Serialization**:
   - Message serialized to Common Data Representation (CDR) format
   - Sequence number assigned (SN=42)
   - Stored in history cache

3. **Transport**:
   - **Local**: Zero-copy shared memory (`/dev/shm`)
   - **Remote**: UDP/TCP serialized transmission

4. **Reception**:
   - DDS DataReader deserializes
   - QoS constraints checked
   - Message queued for subscriber

5. **Callback Execution**:
   - Executor retrieves message from queue
   - Subscriber callback invoked
   - ACK sent back to publisher (Reliable QoS only)

**Error Handling Scenarios:**

- **Message Loss Detection**:
  - Subscriber detects sequence gap (SN=41 → SN=43, missing 42)
  - Sends NACK (Negative Acknowledgment) to publisher
  - Publisher retransmits from history cache

- **History Overflow**:
  - Publisher history full (e.g., KEEP_LAST depth=10)
  - Oldest message evicted
  - Late-joining subscribers miss evicted messages

- **Subscriber Busy**:
  - Callback taking too long
  - Messages buffer in queue up to history depth
  - Overflow results in message loss

**Learning Objectives:**
- Understand end-to-end message flow with timing
- See how Reliable QoS implements ACK/NACK mechanism
- Learn performance implications of history depth
- Debug message loss issues (overflow vs. network vs. QoS mismatch)

**Performance Insights:**
- Shared memory eliminates serialization overhead (microsecond latency)
- Network transport adds milliseconds of latency
- Long callbacks block executor, causing queue overflow
- ACK/NACK protocol adds roundtrip latency vs. Best Effort

---

## Using These Diagrams in Teaching

### Suggested Presentation Order:

1. **Start with `pubsub-pattern.mermaid`**: Introduce basic concepts before diving into details
2. **Move to `qos-comparison.mermaid`**: Show why QoS matters and common patterns
3. **Conclude with `message-lifecycle.mermaid`**: Deep dive for advanced students or troubleshooting

### Interactive Exercise Ideas:

**Using `pubsub-pattern.mermaid`:**
- Have students predict what happens when Subscriber Node 2 crashes
- Ask: "What changes if we add a third subscriber?"
- Demonstrate with live `ros2 topic echo` in multiple terminals

**Using `qos-comparison.mermaid`:**
- Experiment: Start publisher with BEST_EFFORT, subscriber with RELIABLE → connection fails
- Challenge: Design QoS for a new use case (e.g., "telemetry data to cloud")
- Debug scenario: "My subscriber doesn't receive messages" → check QoS compatibility

**Using `message-lifecycle.mermaid`:**
- Trace a message through each step with timestamps
- Simulate message loss: Kill network interface, observe NACK behavior
- Profile latency: Measure time from `publish()` to callback entry

---

## Accessibility and Rendering

All diagrams use:
- High-contrast colors (WCAG AA compliant)
- Clear text labels (not color-only information)
- Consistent styling across diagrams

**Color Legend:**
- Blue (#4A90E2): Publisher components
- Green (#7ED321): Topics
- Orange (#F5A623): Subscriber components
- Purple (#BD10E0): DDS middleware
- Red (#D0021B): Errors/incompatibilities
- Gray (#E8E8E8): Informational boxes

**Rendering in Docusaurus:**

```mdx
```mermaid
${require('!!raw-loader!@site/static/img/module-1-ros2/chapter-2/pubsub-pattern.mermaid').default}
```
```

---

## Further Reading

- [ROS 2 QoS Documentation](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- [DDS RTPS Specification](https://www.omg.org/spec/DDSI-RTPS/)
- [rclpy Publisher API](https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.create_publisher)
- [rclpy Subscriber API](https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.create_subscription)
