import math
import wpilib
from wpilib import SmartDashboard
from commands2 import Subsystem
from networktables import NetworkTablesInstance


class Limelight(Subsystem):
    """Basic Limelight vision subsystem for target tracking."""

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

    # --------------------
    # Vision data getters
    # --------------------
    def has_target(self) -> bool:
        """Return True if a valid target is detected."""
        return self.table.getNumber("tv", 0.0) == 1.0

    def get_tx(self) -> float:
        """Horizontal offset from crosshair to target (degrees)."""
        return self.table.getNumber("tx", 0.0)

    def get_ty(self) -> float:
        """Vertical offset from crosshair to target (degrees)."""
        return self.table.getNumber("ty", 0.0)

    def get_ta(self) -> float:
        """Target area (% of image)."""
        return self.table.getNumber("ta", 0.0)

    # --------------------
    # LED / pipeline control
    # --------------------
    def set_led_mode(self, mode: int):
        """
        LED modes:
          0 = pipeline default
          1 = off
          2 = blink
          3 = on
        """
        self.table.putNumber("ledMode", mode)

    def set_cam_mode(self, mode: int):
        """
        Camera mode:
          0 = vision processor
          1 = driver camera
        """
        self.table.putNumber("camMode", mode)

    def set_pipeline(self, pipeline: int):
        """Set active pipeline (0-9)."""
        self.table.putNumber("pipeline", pipeline)

    # --------------------
    # Distance calculation
    # --------------------
    def get_distance(self) -> float | None:
        """
        Estimate distance to target in meters.
        Returns None if no target is detected.
        """
        if not self.has_target():
            return None

        ty = self.get_ty()
        total_angle_deg = self.mount_angle_deg + ty
        total_angle_rad = math.radians(total_angle_deg)
        distance = (self.target_height - self.lens_height) / math.tan(total_angle_rad)
        return distance

    # --------------------
    # Periodic dashboard updates
    # --------------------
    def periodic(self):
        SmartDashboard.putBoolean("Limelight Has Target", self.has_target())
        SmartDashboard.putNumber("Limelight TX", self.get_tx())
        SmartDashboard.putNumber("Limelight TY", self.get_ty())
        SmartDashboard.putNumber("Limelight TA", self.get_ta())
        distance = self.get_distance()
        if distance is not None:
            SmartDashboard.putNumber("Limelight Distance", distance)
