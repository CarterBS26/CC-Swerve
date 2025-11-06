# TODO: Add CAN IDs for SparkMax motors

import math

# Controller ports
CONTROLLER_PORT = 0

# Swerve module constants
MODULE_COUNT = 4
MODULE_PORTS = [16, 17, 18, 19]  # Example ports, replace with actual ports


# Encoder constants
ENCODER_TICKS_PER_REV = 2048 # Replace with actual value
ENCODER_GEAR_RATIO = 1 # Example value, replace with actual value

# Gyro constants
GYRO_PORT = 20  # Example port, replace with actual port

# Motor ports
DRIVE_MOTOR_PORTS = [1, 3, 5, 8]  # Example ports, replace with actual ports
STEER_MOTOR_PORTS = [2, 4, 6, 9]  # Example ports, replace with actual ports