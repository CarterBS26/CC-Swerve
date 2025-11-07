import wpilib
import commands2
from robotcontainer import RobotContainer


class Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()
        self.container = RobotContainer()


    def robotInit(self):
        self.container.robotInit()
        self.autonomousCommand = None

    def autonomousInit(self):
        self.container.autonomousInit()
        self.container.swerve.zero_modules_to_absolute()
        self.autonomousCommand = self.container.get_autonomous_command()
        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self):
        self.container.autonomousPeriodic()

    def teleopInit(self):
        if self.autonomousCommand:
            self.autonomousInit.cancel()
        self.container.teleopInit()
        self.container.swerve.zero_modules_to_absolute()


    def teleopPeriodic(self):
        x = self.getJoystickAxis(wpilib.XboxController.Axis.kLeftX)
        y = self.getJoystickAxis(wpilib.XboxController.Axis.kLeftY)
        rotation = self.getJoystickAxis(wpilib.XboxController.Axis.kRightX)
        self.container.teleopPeriodic(x, y, rotation)

    def testInit(self):
        self.container.testInit()

    def testPeriodic(self):
        self.container.testPeriodic()