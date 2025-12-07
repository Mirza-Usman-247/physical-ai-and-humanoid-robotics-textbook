# Dockerfile for validating Physical AI textbook code examples
# Based on ROS 2 Humble with Python 3.10+

FROM ros:humble-ros-base

# Install Python 3.10+ and development tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-pytest \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Python packages
RUN apt-get update && apt-get install -y \
    ros-humble-rclpy \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /workspace

# Copy code examples
COPY examples/ /examples/

# Create validation script
RUN echo '#!/bin/bash\n\
set -e\n\
echo "=== Code Validation for Physical AI Textbook ==="\n\
\n\
# Source ROS 2 environment\n\
source /opt/ros/humble/setup.bash\n\
\n\
# Count total and passing examples\n\
TOTAL=0\n\
PASSED=0\n\
\n\
# Find all Python test files\n\
for test_file in $(find /examples -name "test_*.py" -o -name "*_test.py"); do\n\
    TOTAL=$((TOTAL + 1))\n\
    echo "Testing: $test_file"\n\
    if python3 -m pytest "$test_file" -v; then\n\
        PASSED=$((PASSED + 1))\n\
        echo "✅ PASS"\n\
    else\n\
        echo "❌ FAIL"\n\
    fi\n\
done\n\
\n\
echo ""\n\
echo "=== Results ==="\n\
echo "Total tests: $TOTAL"\n\
echo "Passed: $PASSED"\n\
\n\
if [ $TOTAL -eq 0 ]; then\n\
    echo "⚠️  No test files found"\n\
    exit 0\n\
fi\n\
\n\
SUCCESS_RATE=$(awk "BEGIN {printf \"%.0f\", ($PASSED/$TOTAL)*100}")\n\
echo "Success rate: ${SUCCESS_RATE}%"\n\
\n\
# Require 90% success rate (per SC-002)\n\
if [ $SUCCESS_RATE -lt 90 ]; then\n\
    echo "❌ FAIL: Success rate below 90% threshold"\n\
    exit 1\n\
fi\n\
\n\
echo "✅ PASS: Code validation successful"\n\
' > /usr/local/bin/validate-code.sh && chmod +x /usr/local/bin/validate-code.sh

# Set entrypoint
ENTRYPOINT ["/usr/local/bin/validate-code.sh"]
