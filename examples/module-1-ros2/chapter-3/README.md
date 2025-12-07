# Chapter 3: Services & Actions - Code Examples

## Examples

### 1. Simple Service (Add Two Ints)

**Server:**
```bash
python3 simple_service_server.py
```

**Client:**
```bash
python3 simple_service_client.py 5 3
# Output: Result: 8
```

**Key Concepts:**
- Synchronous request-reply pattern
- Blocking client call with `call_async()` + `spin_until_future_complete()`
- Service always returns a response

---

### 2. Simple Action (Fibonacci Sequence)

**Server:**
```bash
python3 simple_action_server.py
```

**Client:**
```bash
python3 simple_action_client.py
# Computes Fibonacci(10) with feedback
```

**Key Concepts:**
- Asynchronous goal-based pattern
- Periodic feedback during execution
- Goal can be cancelled mid-execution
- Final result returned upon completion

---

## Service vs Action Decision Tree

**Use Service when:**
- Operation completes quickly (<1 second)
- Client needs immediate response
- No progress updates needed
- Examples: Get robot state, compute inverse kinematics, trigger emergency stop

**Use Action when:**
- Long-running operation (>1 second)
- Client needs progress feedback
- Cancellation may be required
- Examples: Navigate to waypoint, grasp object, follow trajectory

---

## Introspection Commands

```bash
# List services
ros2 service list

# Call service from command line
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"

# List actions
ros2 action list

# Send action goal from command line
ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 5}" --feedback
```
