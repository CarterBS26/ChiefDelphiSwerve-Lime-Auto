# swerve.py
"""
Swerve subsystem implementation targeting:
 - REV CANSparkMax (NEO) for drive and steer motors
 - Phoenix 6 CANcoder for absolute steering encoder
 - Phoenix 6 Pigeon2 for gyro
 - MK4i L2 style defaults (wheel diameter & sample gear ratio)
 
Notes:
 - Replace constants.* values with your actual robot values (CAN IDs, gear ratios, wheelbase).
 - Some phoenix6/REV method names may vary by library version; TODO markers show where to adapt.
 - Test on the bench at low speeds first (encoder reads, absolute zeroing) before driving.
"""

from typing import List, Tuple
import math
import wpilib
from commands2 import SubsystemBase
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.kinematics import (
    SwerveDrive4Kinematics,
    SwerveModuleState,
    SwerveModulePosition,
    SwerveDrive4Odometry,
    ChassisSpeeds,
)
import constants

# REV imports
from rev import SparkMax, MotorType, RelativeEncoder, SparkMaxPIDController

# Phoenix 6 imports (Pigeon2, CANcoder)
try:
    import phoenix6 as ph
    from phoenix6.configs import (
        CANCoderConfiguration,
        Pigeon2Configuration,
    )
except Exception:
    ph = None  # We'll raise helpful errors if used without library


# ------------------------
# Physics / geometry constants (MK4i L2 defaults, tune if needed)
# ------------------------
WHEEL_DIAMETER_M = getattr(constants, "WHEEL_DIAMETER", 0.1016)  # 4 in default
WHEEL_CIRCUMFERENCE_M = math.pi * WHEEL_DIAMETER_M
DRIVE_GEAR_RATIO = getattr(constants, "DRIVE_GEAR_RATIO", 6.75)  # MK4i L2 common example; tune
STEER_GEAR_RATIO = getattr(constants, "STEER_GEAR_RATIO", 12.8)  # example; tune

MAX_SPEED_MPS = getattr(constants, "MAX_SPEED_MPS", 5.0)  # tune to robot
MAX_ANGULAR_VEL = getattr(constants, "MAX_ANGULAR_VEL", math.radians(360))  # rad/s


def rpm_to_mps(rpm: float, gear_ratio: float = DRIVE_GEAR_RATIO) -> float:
    # motor RPM -> wheel linear m/s
    rps = rpm / 60.0
    wheel_rps = rps / gear_ratio
    return wheel_rps * WHEEL_CIRCUMFERENCE_M


def rotations_to_meters(rotations: float, gear_ratio: float = DRIVE_GEAR_RATIO) -> float:
    wheel_rot = rotations / gear_ratio
    return wheel_rot * WHEEL_CIRCUMFERENCE_M


# ------------------------
# Swerve Module (hardware-specific implementation)
# ------------------------
class SwerveModule:
    """
    Single swerve module using:
      - CANSparkMax (drive) with its relative encoder (for velocity/position)
      - CANSparkMax (steer) in position mode (open-loop fallback available)
      - CANcoder (Phoenix 6) as the absolute encoder for steering zeroing
    """

    def __init__(
        self,
        name: str,
        drive_id: int,
        steer_id: int,
        cancoder_id: int,
        module_position: Tuple[float, float],
        drive_motor_inverted: bool = False,
        steer_motor_inverted: bool = False,
        drive_gear_ratio: float = DRIVE_GEAR_RATIO,
        steer_gear_ratio: float = STEER_GEAR_RATIO,
    ):
        self.name = name
        self.drive_id = drive_id
        self.steer_id = steer_id
        self.cancoder_id = cancoder_id
        self.location = Translation2d(module_position[0], module_position[1])

        self.drive_gear_ratio = drive_gear_ratio
        self.steer_gear_ratio = steer_gear_ratio

        # Hardware objects (initialized in init_hardware)
        self.drive_motor: SparkMax = None
        self.drive_encoder: RelativeEncoder = None
        self.drive_pid: SparkMaxPIDController = None

        self.steer_motor: SparkMax = None
        self.steer_encoder: RelativeEncoder = None
        self.steer_pid: SparkMaxPIDController = None

        # Phoenix6 CANcoder object
        self.cancoder = None

        # Soft state & tuning
        self.encoder_offset_rotations = 0.0  # offset to apply to CANcoder rotations (calibration)
        self.last_angle = Rotation2d()
        self.deadband = 0.05
        self.max_speed = MAX_SPEED_MPS
        self.kP_turn = getattr(constants, "STEER_kP", 0.8)  # tune carefully
        self.kP_drive = getattr(constants, "DRIVE_kP", 0.1)  # optional open-loop fallback

    # ------------------------
    # Hardware init
    # ------------------------
    def init_hardware(self):
        """Create motor and encoder objects. Call during robotInit."""
        # Drive motor
        self.drive_motor = SparkMax(self.drive_id, MotorType.kBrushless)
        self.drive_motor.restoreFactoryDefaults()
        self.drive_motor.setIdleMode(SparkMax.IdleMode.kBrake)
        self.drive_motor.setInverted(False)  # adjust if needed

        # drive encoder (relative encoder on SparkMax)
        self.drive_encoder = self.drive_motor.getEncoder()
        self.drive_pid = self.drive_motor.getPIDController()

        # Steer motor (also using a SparkMax + encoder for closed-loop position)
        self.steer_motor = SparkMax(self.steer_id, MotorType.kBrushless)
        self.steer_motor.restoreFactoryDefaults()
        self.steer_motor.setIdleMode(SparkMax.IdleMode.kBrake)
        self.steer_motor.setInverted(False)  # adjust if needed
        self.steer_encoder = self.steer_motor.getEncoder()
        self.steer_pid = self.steer_motor.getPIDController()

        # Phoenix6 CANcoder (absolute steering encoder)
        if ph is None:
            raise RuntimeError("phoenix6 library not found; install phoenix6 to use CANcoder/Pigeon2")
        # TODO: adapt constructor to your phoenix6 usage (client creation, node creation, etc.)
        # Example using a simple device wrapper (pseudocode):
        try:
            # Phoenix 6 usually requires a 'Client' and a 'Device' object; here we show a lightweight init.
            self.cancoder = ph.CANcoder(self.cancoder_id)  # may need to create a client first in phoenix6
        except Exception:
            # If your phoenix6 usage is different, replace above with your phoenix6 CANcoder creation code
            raise

        # Optional: tune PID controllers on SparkMax for steering position control
        # The SparkMax PID controller accepts gains: kP, kI, kD, kFF, IZone, output range.
        # These are examples — tune on bench.
        try:
            # Steering PID tuning (example values)
            self.steer_pid.setP(self.kP_turn)
            self.steer_pid.setI(0.0)
            self.steer_pid.setD(0.0)
            self.steer_pid.setFF(0.0)
        except Exception:
            # If SparkMax Python API on your env differs, adapt names accordingly
            wpilib.reportWarning(f"{self.name}: could not set SparkMax PID values - adapt in init_hardware()", stackTrace=False)

    # ------------------------
    # Read absolute (CANcoder) steering angle (in rotations 0..1)
    # ------------------------
    def _read_cancoder_rotations(self) -> float:
        # TODO: adapt to your phoenix6 CANcoder API method name (e.g., getPosition(), getAbsolutePosition())
        # We'll try to return rotations (rev fraction). If CANcoder returns degrees, divide by 360.
        val = None
        try:
            # Many wrappers return degrees; check docs and adapt
            raw = self.cancoder.getPosition()  # likely in degrees or rotations depending on API
            # If raw seems in degrees, convert: raw_deg / 360.0
            # We'll heuristically handle both: if > 2*pi -> treat as degrees
            if abs(raw) > 10:
                val = (raw % 360.0) / 360.0
            else:
                # assume rotations (0..1)
                val = raw % 1.0
        except Exception:
            raise RuntimeError(f"{self.name}: can't read CANcoder - adapt _read_cancoder_rotations() to your ph API")
        return val

    # ------------------------
    # High-level accessors
    # ------------------------
    def get_turn_angle(self) -> Rotation2d:
        """Return module steer angle as Rotation2d (radians)."""
        rotations = self._read_cancoder_rotations() - self.encoder_offset_rotations
        degrees = rotations * 360.0
        return Rotation2d.fromDegrees(degrees)

    def get_drive_velocity(self) -> float:
        """Return drive wheel speed in meters/second using SparkMax relative encoder RPM."""
        try:
            rpm = self.drive_encoder.getVelocity()  # SparkMax getVelocity returns RPM
        except Exception:
            # adapt if your API returns different name
            rpm = self.drive_motor.getEncoder().getVelocity()
        return rpm_to_mps(rpm, self.drive_gear_ratio)

    def get_position(self) -> SwerveModulePosition:
        """Return linear distance travelled and angle (for odometry)."""
        try:
            rotations = self.drive_encoder.getPosition()  # motor rotations
        except Exception:
            rotations = 0.0
        meters = rotations_to_meters(rotations, self.drive_gear_ratio)
        return SwerveModulePosition(meters, self.get_turn_angle())

    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(self.get_drive_velocity(), self.get_turn_angle())

    # ------------------------
    # Reset / zeroing
    # ------------------------
    def reset_to_absolute(self):
        """Set steer encoder position to match CANcoder absolute angle."""
        # Read absolute rotations and write that into the steer motor's encoder reference
        abs_rot = self._read_cancoder_rotations()  # fractional revs
        # Convert to Spark relative encoder rotations required by your steer motor control approach.
        # One simple approach: set the internal encoder position variable (if supported):
        try:
            # Example: if steer_encoder supports setPosition(rotations)
            self.steer_encoder.setPosition(abs_rot * self.steer_gear_ratio)
        except Exception:
            # Some teams implement an offset only and use CANcoder for control; record offset
            self.encoder_offset_rotations = abs_rot
            wpilib.reportWarning(f"{self.name}: couldn't set steer encoder position; stored offset instead", stackTrace=False)

        wpilib.log.info(f"[{self.name}] zeroed steer to {math.degrees(Rotation2d.fromRotations(abs_rot).radians):.2f}°")

    # ------------------------
    # Command the module to a desired state
    # ------------------------
    def set_state(self, desired_state: SwerveModuleState):
        """
        desired_state.speed: m/s
        desired_state.angle: Rotation2d
        """
        # Optimize to avoid large rotation moves
        current_angle = self.get_turn_angle()
        optimized = SwerveModuleState.optimize(desired_state, current_angle)

        # Drive: convert m/s -> motor RPM
        wheel_rps = optimized.speed / WHEEL_CIRCUMFERENCE_M
        motor_rps = wheel_rps * self.drive_gear_ratio
        motor_rpm = motor_rps * 60.0

        # Steer: target rotations (fraction of wheel revolution)
        target_rotations = optimized.angle.getDegrees() / 360.0

        # Drive: set velocity on SparkMax (RPM target). SparkMax Python PID controller accepts setReference with control type.
        try:
            # Many SparkMax wrappers expose setReference with ControlType.kVelocity and value in RPM
            self.drive_pid.setReference(motor_rpm, SparkMax.ControlType.kVelocity)
        except Exception:
            # Fallback: set open-loop percentage using kP_drive
            pct = (motor_rpm / (MAX_SPEED_MPS * 60.0)) if MAX_SPEED_MPS > 0 else 0
            self.drive_motor.set(pct)

        # Steer: set position – convert target_rotations into motor rotations (if steering motor uses a geared ratio)
        # If steer motor will be commanded in "rotations" units: set position = target_rotations * steer_gear_ratio
        try:
            target_motor_rot = target_rotations * self.steer_gear_ratio
            # Use SparkMax position control
            self.steer_pid.setReference(target_motor_rot, SparkMax.ControlType.kPosition)
        except Exception:
            # Fallback: simple P-control using relative encoder
            current = self.steer_encoder.getPosition()
            error = (target_rotations * self.steer_gear_ratio) - current
            self.steer_motor.set(self.kP_turn * error)

        self.last_angle = optimized.angle

    def stop(self):
        """Stop motors safely."""
        try:
            self.drive_motor.set(0)
            self.steer_motor.set(0)
        except Exception:
            wpilib.reportWarning(f"{self.name}: stop() couldn't set motors - check hardware API", stackTrace=False)


# ------------------------
# SwerveSubsystem (4 modules)
# ------------------------
class SwerveSubsystem(SubsystemBase):
    def __init__(self):
        super().__init__()

        # module order: FL, FR, BL, BR
        half_x = getattr(constants, "ROBOT_HALF_LENGTH", 0.28)
        half_y = getattr(constants, "ROBOT_HALF_WIDTH", 0.28)
        fl_pos = (half_x, half_y)
        fr_pos = (half_x, -half_y)
        bl_pos = (-half_x, half_y)
        br_pos = (-half_x, -half_y)

        drive_ids = getattr(constants, "DRIVE_MOTOR_PORTS", [1, 3, 5, 8])
        steer_ids = getattr(constants, "STEER_MOTOR_PORTS", [2, 4, 6, 9])
        cancoder_ids = getattr(constants, "MODULE_PORTS", [16, 17, 18, 19])

        self.front_left = SwerveModule("FL", drive_ids[0], steer_ids[0], cancoder_ids[0], fl_pos)
        self.front_right = SwerveModule("FR", drive_ids[1], steer_ids[1], cancoder_ids[1], fr_pos)
        self.back_left = SwerveModule("BL", drive_ids[2], steer_ids[2], cancoder_ids[2], bl_pos)
        self.back_right = SwerveModule("BR", drive_ids[3], steer_ids[3], cancoder_ids[3], br_pos)

        self.modules: List[SwerveModule] = [
            self.front_left,
            self.front_right,
            self.back_left,
            self.back_right,
        ]

        # kinematics & odometry
        self.kinematics = SwerveDrive4Kinematics(
            self.front_left.location,
            self.front_right.location,
            self.back_left.location,
            self.back_right.location,
        )

        # gyro / pigeon
        if ph is None:
            self.gyro = None
        else:
            # TODO: construct phoenix6 client and Pigeon2 device properly for your setup
            # Example pseudocode:
            # self.gyro = ph.Pigeon2(20)
            self.gyro = None

        self.odometry = SwerveDrive4Odometry(self.kinematics, Rotation2d(), self.get_module_positions(), Pose2d())

    # ------------------------
    # Hardware init
    # ------------------------
    def init_hardware(self):
        # init modules
        for m in self.modules:
            m.init_hardware()

        # init gyro/pigeon
        if ph is None:
            wpilib.reportWarning("phoenix6 not installed: Pigeon2/CANcoder functionality disabled", stackTrace=False)
        else:
            # TODO: instantiate phoenix6 Pigeon2 and zero yaw
            try:
                self.gyro = ph.Pigeon2(getattr(constants, "PIGEON_ID", 20))
                # zero yaw (adapt to API)
                self.gyro.setYaw(0)
            except Exception:
                wpilib.reportWarning("Couldn't instantiate Pigeon2 - adapt init_hardware()", stackTrace=False)

    # ------------------------
    # Lifecycle
    # ------------------------
    def robotInit(self):
        self.init_hardware()

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self, x: float, y: float, rotation: float):
        self.drive(x, y, rotation, field_relative=True)

    # ------------------------
    # Utilities
    # ------------------------
    def zero_modules_to_absolute(self):
        for m in self.modules:
            try:
                m.reset_to_absolute()
            except Exception as e:
                wpilib.reportWarning(f"{m.name}.reset_to_absolute() failed: {e}", stackTrace=False)

    def get_heading(self) -> Rotation2d:
        if self.gyro is None:
            return Rotation2d()
        try:
            # adapt method name to phoenix6 API: could be getYaw() / getRotation2d() / getAngle()
            yaw_deg = self.gyro.getYaw()
            return Rotation2d.fromDegrees(yaw_deg)
        except Exception:
            wpilib.reportWarning("get_heading(): adapt to your Pigeon2 API", stackTrace=False)
            return Rotation2d()

    def zero_heading(self):
        if self.gyro is None:
            return
        try:
            self.gyro.setYaw(0)
        except Exception:
            wpilib.reportWarning("zero_heading(): adapt to your Pigeon2 API", stackTrace=False)

    def get_module_positions(self) -> List[SwerveModulePosition]:
        return [m.get_position() for m in self.modules]

    def update_odometry(self):
        self.odometry.update(self.get_heading(), self.get_module_positions())

    def get_pose(self) -> Pose2d:
        return self.odometry.getPose()

    def reset_odometry(self, pose: Pose2d):
        self.odometry.resetPosition(self.get_heading(), self.get_module_positions(), pose)

    # ------------------------
    # Driving API
    # ------------------------
    def stop(self):
        for m in self.modules:
            m.stop()

    def drive(self, x_speed: float, y_speed: float, rot: float, field_relative: bool = True):
        """
        x_speed, y_speed in meters/sec, rot in rad/sec.
        """
        if field_relative:
            chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x_speed, y_speed, rot, self.get_heading())
        else:
            chassis_speeds = ChassisSpeeds(x_speed, y_speed, rot)

        states = self.kinematics.toSwerveModuleStates(chassis_speeds)
        SwerveDrive4Kinematics.desaturateWheelSpeeds(states, MAX_SPEED_MPS)

        for module, state in zip(self.modules, states):
            module.set_state(state)
