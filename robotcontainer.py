import wpilib
import commands2
from wpilib import SmartDashboard
from Swerve import Swerve
from Subsystems import Limelight, AutoAlign, DriveToTarget
from Commands import Autonomous


class RobotContainer:
    def __init__(self):

        # ----------------------------
        # Subsystems
        # ----------------------------
        self.driver_controller = wpilib.XboxController(0)
        self.swerve = Swerve()

        # Limelight configuration
        self.limelight = Limelight(
            mount_angle_deg=0,
            lens_height=0.5,
            target_height=0.5
        )
        self.limelight.set_led_mode(3)
        self.limelight.set_cam_mode(0)

        # ----------------------------
        # Auto Commands
        # ----------------------------
        self.autoChooser = wpilib.SendableChooser()
        self.autoChooser.setDefaultOption(
            "Drive Forward",
            Autonomous.AutoForward(self.swerve)
        )
        self.autoChooser.addOption(
            "Spin and Drive",
            Autonomous.AutoTurnAndDrive(self.swerve)
        )
        self.autoChooser.addOption(
            "Square Drive",
            Autonomous.AutoSquare(self.swerve)
        )
        SmartDashboard.putData("AutoSelector", self.autoChooser)

        # ----------------------------
        # Teleop Commands
        # ----------------------------
        self.AutoAlign_command = AutoAlign(
            self.swerve, self.limelight, desired_distance=2.5
        )
        self.DriveToTarget = DriveToTarget(
            self.swerve, self.limelight, desired_distance=1
        )

        # Configure controller buttons
        self.configureButtonBindings()

    # -----------------------------------------------------------
    # Button Bindings
    # -----------------------------------------------------------
    def configureButtonBindings(self):
        wpilib.JoystickButton(
            self.driver_controller,
            wpilib.XboxController.Button.k6
        ).whileTrue(self.AutoAlign_command)

        wpilib.JoystickButton(
            self.driver_controller,
            wpilib.XboxController.Button.k8
        ).whileTrue(self.DriveToTarget)

    # -----------------------------------------------------------
    # Autonomous selection
    # -----------------------------------------------------------
    def get_autonomous_command(self):
        return self.autoChooser.getSelected()

    # -----------------------------------------------------------
    # Mode Lifecycle Hooks
    # -----------------------------------------------------------
    def robotInit(self):
        self.swerve.robotInit()

    def autonomousInit(self):
        self.swerve.autonomousInit()

    def autonomousPeriodic(self):
        self.swerve.autonomousPeriodic()

    def teleopInit(self):
        self.swerve.teleopInit()

    def teleopPeriodic(self, x, y, rotation):
        self.swerve.teleopPeriodic(x, y, rotation)

    def testInit(self):
        self.swerve.testInit()

    def testPeriodic(self):
        self.swerve.testPeriodic()