import math
from typing import Optional
import wpilib
from wpilib import SmartDashboard
from commands2 import SubsystemBase, CommandBase
from networktables import NetworkTablesInstance


# ------------------------
#   Limelight Subsystem
# ------------------------
class Limelight(SubsystemBase):
    def __init__(
        self,
        table_name: str = "limelight",
        mount_angle_deg: float = 25.0,
        lens_height: float = 0.5,
        target_height: float = 2.5,
    ):
        super().__init__()
        self.table = NetworkTablesInstance.getDefault().getTable(table_name)
        self.mount_angle_deg = mount_angle_deg
        self.lens_height = lens_height
        self.target_height = target_height

    # Vision data
    def has_target(self) -> bool:
        return self.table.getNumber("tv", 0.0) == 1.0

    def get_tx(self) -> float:
        return self.table.getNumber("tx", 0.0)

    def get_ty(self) -> float:
        return self.table.getNumber("ty", 0.0)

    def get_ta(self) -> float:
        return self.table.getNumber("ta", 0.0)

    # LED / camera control
    def set_led_mode(self, mode: int):
        """0=default,1=off,2=blink,3=on"""
        self.table.putNumber("ledMode", mode)

    def set_cam_mode(self, mode: int):
        """0=vision, 1=driver"""
        self.table.putNumber("camMode", mode)

    def set_pipeline(self, pipeline: int):
        self.table.putNumber("pipeline", pipeline)

    # Distance calculation
    def get_distance(self) -> Optional[float]:
        if not self.has_target():
            return None
        ty = self.get_ty()
        total_angle_rad = math.radians(self.mount_angle_deg + ty)
        distance = (self.target_height - self.lens_height) / math.tan(total_angle_rad)
        return distance

    # Periodic dashboard logging
    def periodic(self):
        SmartDashboard.putBoolean("Limelight Has Target", self.has_target())
        SmartDashboard.putNumber("Limelight TX", self.get_tx())
        SmartDashboard.putNumber("Limelight TY", self.get_ty())
        SmartDashboard.putNumber("Limelight TA", self.get_ta())
        dist = self.get_distance()
        SmartDashboard.putNumber("Limelight Distance", dist if dist is not None else 0)

# ------------------------
#   AutoAlign Command
# ------------------------

class AutoAlign(CommandBase):
    """Rotate the robot to face the Limelight target."""

    def __init__(self, swerve, limelight, alignment_tolerance_deg: float = 1.0):
        """
        :param swerve: your Swerve subsystem
        :param limelight: Limelight subsystem
        :param alignment_tolerance_deg: degrees within which the robot is considered aligned
        """
        super().__init__()
        self.swerve = swerve
        self.limelight = limelight
        self.alignment_tolerance = alignment_tolerance_deg
        self.kP_turn = 0.03  # proportional constant for turning
        self.max_turn = 0.3  # max rotational speed
        self.addRequirements(swerve)

    def initialize(self):
        # Turn LEDs on when starting alignment
        self.limelight.set_led_mode(3)

    def execute(self):
        if not self.limelight.has_target():
            # Stop if target disappears
            self.swerve.stop()
            return

        # Proportional turning
        error = self.limelight.get_tx()  # positive = target to right, negative = left
        turn_speed = max(min(self.kP_turn * error, self.max_turn), -self.max_turn)
        self.swerve.drive(0, 0, turn_speed)  # rotate only

    def isFinished(self) -> bool:
        # Finished when within alignment tolerance
        if not self.limelight.has_target():
            return False
        return abs(self.limelight.get_tx()) <= self.alignment_tolerance

    def end(self, interrupted: bool):
        # Stop rotation and turn off LEDs
        self.swerve.stop()
        self.limelight.set_led_mode(1)

# ------------------------
#   DriveToTarget Command
# ------------------------
class DriveToTarget(CommandBase):
    """Drive forward and align to a vision target using Limelight."""

    def __init__(self, swerve, limelight: Limelight, desired_distance: float = 1.0):
        super().__init__()
        self.swerve = swerve
        self.limelight = limelight
        self.desired_distance = desired_distance
        self.addRequirements(swerve)

        # PID-like proportional constants
        self.kP_drive = 0.3
        self.kP_turn = 0.03
        self.max_forward = 0.4
        self.max_turn = 0.3
        self.tolerance = 0.05  # meters

    def initialize(self):
        self.limelight.set_led_mode(3)  # turn LEDs on

    def execute(self):
        if not self.limelight.has_target():
            self.swerve.stop()
            return

        distance = self.limelight.get_distance()
        if distance is None:
            self.swerve.stop()
            return

        # Forward proportional control
        error = distance - self.desired_distance
        forward_speed = max(min(self.kP_drive * error, self.max_forward), -self.max_forward)

        # Turn proportional control
        turn_error = self.limelight.get_tx()
        turn_speed = max(min(self.kP_turn * turn_error, self.max_turn), -self.max_turn)

        # Drive swerve
        self.swerve.drive(forward_speed, 0, turn_speed)

    def isFinished(self) -> bool:
        """Stop when robot is within tolerance of target distance."""
        if not self.limelight.has_target():
            return False
        distance = self.limelight.get_distance()
        if distance is None:
            return False
        return abs(distance - self.desired_distance) < self.tolerance

    def end(self, interrupted: bool):
        self.swerve.stop()
        self.limelight.set_led_mode(1)  # turn LEDs off
