# This message represents feedback data from a single drive unit (driver + motor).

# Current flowing from battery into the MOSFET bridge.
float32 current

# Instantaneous duty cycle of MOSFET bridge.
float32 duty_cycle

# Temperatures measured in the MOSFET bridge and on the motor casing, in deg C.
float32 bridge_temperature
float32 motor_temperature

# Encoder data
float32 measured_velocity   # rad/s
float32 measured_travel     # rad

# True if the underlying driver chip reports a fault condition.
bool driver_fault
