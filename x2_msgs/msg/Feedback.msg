# This message represents high-frequency feedback from the MCU,
# as necessary to support closed-loop control and thermal monitoring.
# Default publish frequency is 50Hz.

Header header

DriveFeedback[2] drivers

# Temperatures
float32 pcb_temperature
float32 mcu_temperature

# Commanded control mode, use the TYPE_ constants from x2_msgs/Drive.
int8 commanded_mode

# Actual control mode. This may differ from the commanded in cases where
# the motor enable is off, the motors are in over-current, etc.
int8 actual_mode
