import wpilib
import commands2
from robotcontainer import RobotContainer


class Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()
        self.container = RobotContainer()
        self.autonomousCommand = None

    def robotInit(self):
        # Only call this if RobotContainer actually defines robotInit()
        if hasattr(self.container, "robotInit"):
            self.container.robotInit()

    def autonomousInit(self):
        self.container.autonomousInit()

        # Zero the modules before automode
        self.container.swerve.zero_modules_to_absolute()

        # Get the autonomous command
        self.autonomousCommand = self.container.get_autonomous_command()

        # Schedule it
        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self):
        self.container.autonomousPeriodic()

    def teleopInit(self):
        # Cancel autonomous if it is running
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

        self.container.teleopInit()

        # Zero the modules again for safety
        self.container.swerve.zero_modules_to_absolute()

    def teleopPeriodic(self):
        # Use the Xbox controller from RobotContainer
        controller = self.container.controller  

        x = controller.getLeftX()
        y = controller.getLeftY()
        rotation = controller.getRightX()

        self.container.teleopPeriodic(x, y, rotation)

    def testInit(self):
        self.container.testInit()

    def testPeriodic(self):
        self.container.testPeriodic()