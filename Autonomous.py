import commands2
import wpilib
from pathplannerlib.auto import PathPlannerAuto

class AutoForward(commands2.CommandBase):
    def __init__(self, swerve, speed=0.5, time=3):
        super().__init__()
        self.swerve = swerve
        self.speed = speed
        self.time = time
        self.timer = wpilib.timer()
        self.addRequirements(swerve)

    def initialize(self):
        self.timer.reset()
        self.timer.start()
        self.swerve.drive(self.speed, 0, 0)

    def execute(self):
        self.swerve.drive(self.speed, 0, 0)

    def isFinished(self):
        return self.timer.hasElapsed(self.time)
    
    def end(self, interrupted):
        self.swerve.stop()

class AutoTurnAndDrive(commands2.CommandBase):
    def __init__(self, swerve, turn_speed=0.25, drive_speed=0.25):
        super().__init__()
        self.swerve = swerve
        self.turn_speed = turn_speed
        self.drive_speed = drive_speed
        self.timer = wpilib.timer()
        self.addRequirements(swerve)

    def initialize(self):
        self.timer.reset()
        self.timer.start()

    def execute(self):
        if self.timer.get() < 1:
            self.swerve.drive(0, 0, self.turn_speed)
        else: 
            self.swerve.drive(self.drive_speed, 0, 0)

    def isFinished(self):
        return self.timer.hasElapsed(1.5)
    
    def end(self, interrupted):
        self.swerve.stop()

class AutoSquare(commands2.CommandBase):
    def __init__(self, swerve, speed=0.25, segment_time=1):
        super().__init__()
        self.swerve = swerve
        self.speed = speed
        self.segment_time = segment_time
        self.timer = wpilib.timer()
        self.addRequirements(swerve)

    def initialize(self):
        self.timer.reset()
        self.timer.start()

    def execute(self):
        t = self.timer.get()
        if t < self.segment_time:
            self.swerve.drive(self.speed, 0, 0)
        elif t < 2 * self.segment_time:
            self.swerve.drive(0, self.speed, 0)
        elif t < 3 * self.segment_time:
            self.swerve.drive(-self.speed, 0, 0)
        elif t < 4 * self.segment_time:
            self.swerve.drive(0, -self.speed, 0)
        else: 
            self.swerve.stop

    def isFinished(self):
        return self.timer.hasElapsed(4 * self.segment_time)
    
    def end(self, interrupted):
        self.swerve.stop()

#class AutoPathFollow(commands2.SequentialCommandGroup):
#    def __inti__(self, path_name: str):
#        """run a full auto pathdesigned in path planner"""
#        super().__init__()
#        self.addCommands(PathPlannerAuto(path_name))
