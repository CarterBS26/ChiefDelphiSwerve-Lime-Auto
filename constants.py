"""
Robot-wide constant definitions for swerve drive configuration.
Update all placeholder values before deployment.
"""

import math

CONTROLLER_PORT = 0
BUTTONBOARD_PORT = 1

MODULE_COUNT = 4

WHEEL_DIAMETER = 0.1016  # 4 inches in meters
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
DRIVE_GEAR_RATIO = 6.75
STEER_GEAR_RATIO = 12.8

ENCODER_TICKS_PER_REV = 2048        # TODO: Confirm encoder resolution
ENCODER_GEAR_RATIO = 1              # TODO: Replace with actual gear ratio

GYRO_PORT = 20

DRIVE_CAN_BUS = "rio"
STEER_CAN_BUS = "rio"
ENCODER_CAN_BUS = "canivore"
FL, FR, BL, BR = 0, 1, 2, 3
DRIVE_MOTOR_PORTS = {
    FL: 1,
    FR: 3,
    BL: 5,
    BR: 8
}
STEER_MOTOR_PORTS = {
    FL: 2,
    FR: 4,
    BL: 6,
    BR: 9
}
MODULE_PORTS = {
    FL: 16, 
    FR: 17, 
    BL: 18, 
    BR: 19
}