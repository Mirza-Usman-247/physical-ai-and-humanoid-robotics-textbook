# Custom message type for sensor data streaming
# Demonstrates composite message structure

# Header with timestamp and frame_id
std_msgs/Header header

# Sensor identification
string sensor_id
string sensor_type

# Sensor readings
float64 temperature    # degrees Celsius
float64 humidity       # percentage (0-100)
float64 pressure       # kilopascals (kPa)

# Status flags
bool is_calibrated
uint8 error_code       # 0 = no error, >0 = error codes

# Confidence metrics
float32 measurement_confidence  # 0.0-1.0
