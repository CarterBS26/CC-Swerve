import wpilib
from wpilib import SmartDashboard, Timer, NetworkTableInstance
from commands2 import Subsystem, CommandBase
import math

class Limelight(Subsystem):
    def __init__(self, table_name="limelight", mount_angle_deg=0, lens_height=0.5, target_height=0.5):
        super().__init__()
        self.table = NetworkTableInstance.getDefault().getTable(table_name)
        self.mount_angle_deg = mount_angle_deg
        self.lens_height = lens_height
        self.target_height = target_height
        self.last_valid_time = 0

    def get_tx(self):
        """horizontal offset from crosshair to target(degrees)"""
        return self.table.getNumber("tx", 0.0)
    
    def get_ty(self):
        """vertical offset from crosshair to target(degrees)"""
        return self.table.getNumber("ty", 0.0)
    
    def get_ta(self):
        """target area (% of image)"""
        return self.table.getNumber("ta", 0.0)
    
    def has_target(self):
        """Returns true if a target is detected"""
        return self.table.getNumber("tv", 0.0) == 1.0
    
    # LED / Pipeline Control
    def set_led_mode(self, mode):
        """
        LED modes:
        0 - pipeline default
        1 - Force Off
        2 - Force Blink
        3 - Force On
        """
        self.table.putNumber("ledMode", mode)

    def set_cam_mode(self, mode):
        """
        0 - Vision Processor
        1 - Dricing Camera (Processing off)
        """
        self.table.putNumber("camMode", mode)

    def set_pipeline(self, pipeline):
        """sets which pipeline is active (0-9)"""
        self.table.putNumber("pipeline", pipeline)

    #chat's example of SmartDashboard logging
    def periodic(self):
        SmartDashboard.putBoolean("limelight Has Target", self.has_target())
        SmartDashboard.putNumber("limelight tx", self.get_tx())
        SmartDashboard.putBoolean("limelight ty", self.get_ty())
        SmartDashboard.putBoolean("limelight ta", self.get_ta())

#distance calculations
    def get_distance(self):
        """estimate distance from limelight to target in meters"""
        if not self.has_target():
            return None
        ty = self.get_ty()
        total_angle_deg = self.mount_angle_deg + ty
        total_angle_rad = math.radians(total_angle_deg)
        distance = (self.target_height - self.lens_height) / math.tan(total_angle_rad)
        return distance

class DriveToTarget(CommandBase):
    def __init__(self, swerve, limelight, desired_distance=1):
        super().__init__()
        self.swerve = swerve
        self.limelight = limelight
        self.desired_distance = desired_distance
        self.addRequirements(swerve)

    def execute(self):
        if self.limelight.has_target():
            distance = self.limelight.get_distance()
            if distance is None:
                self.swerve.stop()
                return
            kP_drive = 0.3
            error = distance - self.desired_distance
            forward = kP_drive * error
            forward = max(min(forward, 0.4), -0.4) #output speed
            kP_turn  = 0.03
            turn = kP_turn * self.limelight.get_tx()
            turn = max(min(turn, 0.3), -0.3)
            self.swerve.drive(forward, 0, turn)
        else:
            self.swerve.stop()
    
    def end(self, interrupted):
        self.swerve.stop()

    def isFinished(self):
        if not self.limelight.has_target():
            return False
        distance = self.limelight.get_dstance()
        if distance is None:
            return False
        return abs(distance - self.desired_distance) < 0.1 #stop within 0.1 meter of target

class AutoAlign(CommandBase):
    def __init__(self, drivetrain, limelight):
        super().__init__()
        self.drivetrain = drivetrain
        self.limelight = limelight
        self.addRequirements(drivetrain)

    def execute(self):
        if self.limelight.has_target():
            error = self.limelight.get_tx()
            kP = 0.02
            turn_speed = kP * error #multiplying by tx turns the robot toward the target
            self.drivetrain.arcadeDrive(0, turn_speed)
        else:
            self.drivetrain.arcadeDrive(0,0)
