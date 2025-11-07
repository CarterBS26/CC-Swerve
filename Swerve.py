# TODO: Calibrate the gyro
# TODO: Initialize the encoder
# TODO: Invert motor directions (if necessary)
# TODO: Adjust deadbanding and velocity limiting constants

#This is a template from chiefdelfi/github - should be a good starting point, easy enough to learn and understand
from rev import SparkMax, SparkLowLevel
from pheonix6.hardware import Pigeon2, CANcoder
from wpimath.geometry import rotation2d, pose2d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModulePosition, SwerveModuleState, SwerveDrive4Odometry
from commands2 import subsystembase
import wpilib
import math
import constants 

class Swerve:
    def __init__(self):
        self.modules = [SwerveModule(i) for i in range(4)]

    def robotInit(self):
        for module in self.modules:
            module.robotInit()

    def autonomousInit(self):
        for module in self.modules:
            module.autonomousInit()

    def autonomousPeriodic(self):
        for module in self.modules:
            module.autonomousPeriodic()

    def teleopInit(self):
        for module in self.modules:
            module.teleopInit()

    def teleopPeriodic(self, x, y, rotation):
        for module in self.modules:
            module.teleopPeriodic(x, y, rotation)

    def testInit(self):
        for module in self.modules:
            module.testInit()

    def testPeriodic(self):
        for module in self.modules:
            module.testPeriodic()

class SwerveModule:
    def __init__(self, module_number):
        self.module_number = module_number
        self.drive_motor = wpilib.SparkMax(1, 3, 5, 8[module_number], wpilib.MotorType.kBrushless)
        self.steer_motor = wpilib.SparkMax(2, 4, 6, 9[module_number], wpilib.MotorType.kBrushless)
        self.encoder = wpilib.Encoder(16, 17, 18, 19)
        self.encoder_offset = abs.encoder_offset
        self.location = (0.28, 0.28) #meters from center of robot x,y
        self.last_angle = rotation2d()
        self.gyro = phoenix6.Pigeon2
        self.current_angle = 0.0
        self.name = __name__

    def get_absolute_pos(self):
        position = self.encoder.getAbsolutePosition()
        angle = (position - self.encoder_offset) * 2 * math.pi
        return angle
    def reset_to_absolute(self):
        """set the turning motor's encoder to match the absolute encoder"""
        absolute_angle = self.get_absolute_pos()
        self.turning_motor.setSelectedSensorPosition(absolute_angle)
        print(f"[{self.name}] zeroed to {math.degrees(absolute_angle):.2f}°")

    def get_drive_velocity(self):
        rotations_per_sec = self.drive_motor.get_velocity().value
        wheel_circumference = .1016 * math.pi
        gear_ratio = 5.78
        return (rotations_per_sec / gear_ratio) * wheel_circumference
    def get_turn_angle(self) -> rotation2d:
        degrees = self.encoder.get_absolute_position().value * 360.0
        return rotation2d.fromDegrees(degrees)
    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(
            self.get_drive_velocity(),
            self.get_turn_angle()
        )
    def get_position(self) -> SwerveModulePosition:
        distance = self.drive_motor.get_position().value
        wheel_circumference = 0.1016 * math.pi
        gear_ratio = 5.78
        meters = (distance / gear_ratio) * wheel_circumference
        return SwerveModulePosition(meters, self.get_turn_angle())
    def set_state(self, desired_state:SwerveModuleState):
        #optimize wheels spinny
        current_angle = self.get_turn_angle()
        optimized_state = SwerveModuleState.optimize(desired_state, current_angle)
        #drive speed command
        drive_rps = optimized_state.speed / (0.1016 * math.pi) * 5.78
        self.drive_motor.set_control("velocity", drive_rps)
        #turn to wanted angle
        target_degrees = optimized_state.angle.degrees()
        self.steer_motor.set_control("position", target_degrees / 360.0)
        self.last_angle = optimized_state.angle

       # Deadbanding constants
        self.deadband_x = 0.1
        self.deadband_y = 0.1
        self.deadband_rotation = 0.1

        # Velocity limiting constants
        self.max_velocity = 1.0

        # Angle limiting constants
        self.max_angle = math.pi / 2

        # Motor acceleration limiting constants
        self.max_acceleration = 0.5

    def robotInit(self):
        self.encoder.reset()
        self.gyro.reset()

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self, x, y, rotation):
        # Apply deadband
        if abs(x) < self.deadband_x:
            x = 0
        if abs(y) < self.deadband_y:
            y = 0
        if abs(rotation) < self.deadband_rotation:
            rotation = 0

        # Calculate velocities and angles
        velocity = math.hypot(x, y)
        angle = math.atan2(y, x) if x != 0 else 0

        # Limit velocity
        velocity = min(velocity, self.max_velocity)

        # Calculate motor speeds
        drive_speed, steer_speed = self.calculate_motor_speeds(velocity, angle)

        # Set motor speeds
        self.drive_motor.set(drive_speed)
        self.steer_motor.set(steer_speed)

    def testInit(self):
        pass

    def testPeriodic(self):
        pass

    def calculate_motor_speeds(self, velocity, angle):
        # Calculate drive and steer motor speeds using the velocity and angle
        drive_speed = velocity * math.cos(angle)
        steer_speed = velocity * math.sin(angle)

        # Limit motor acceleration
        drive_speed = max(-self.max_acceleration, min(self.max_acceleration, drive_speed))
        steer_speed = max(-self.max_acceleration, min(self.max_acceleration, steer_speed))

        return drive_speed, steer_speed
    
class SwerveSubsystem(subsystembase):
    def __init__(self):
        super().__init__()

        #start of gyro stuff
        self.gyro = Pigeon2(20, "rio")
        self.gyro.set_yaw(0)

        #sets the swerve base
        self.front_left = SwerveModule("FL", drive_motor=1, steer_motor=2, encoder=16)
        self.front_right = SwerveModule("FR", drive_motor=3, steer_motor=4, encoder=17)
        self.back_left = SwerveModule("BL", drive_motor=5, steer_motor=6, encoder=18)
        self.back_right = SwerveModule("BR", drive_motor=8, steer_motor=9, encoder=19)

        #weird kinematic thingy it told me to do
        self.kinematics = SwerveDrive4Kinematics(
            self.front_left.location,
            self.front_right.location,
            self.back_left.location,
            self.back_right.location,
        )

        #odometry - should make swerve swerve better
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.get_heading(),
            self.get_module_positions(),
            pose2d()
        )
    def zero_modules_to_absolute(self):
        self.front_left.reset_to_absolute()
        self.front_right.reset_to_absolute()
        self.back_left.reset_to_absolute()
        self.back_right.reset_to_absolute()

        #pigeon and odometer together = bueno
    def get_heading(self) -> rotation2d:
            """Return current robot heading as a rotation2d."""
            return rotation2d.fromdegrees(self.gryo.get_yaw().value)
    
    def zero_heading(self):
            """zero the gyro yaw to set the heading as 0"""
            self.gryo.set_yaw(0)

    def get_module_positions(self):
            """return list of swerve position objects for the odometer"""
            return [
                self.front_left.get_position(),
                self.front_right.get_position(),
                self.back_left.get_position(),
                self.back_right.get_position(),
            ]
    def update_odometry(self):
            """update robot pose estimate"""
            self.odometry.update(self.get_heading(), self.get_module_positions())

    def get_pose(self):
        """get current robot position estimate"""
        return self.odometry.getpose()
    
    def reset_odometry(self, pose: pose2d):
        """reset odometry to actual field pos"""
        self.odometry.resetPosition(self.get_heading(), self.get_module_positions, pose)

# actual driving stuff
    def drive(self, x_speed, y_speed, rot, field_relative=True):
        """x/y is m/s and rot is rad/s"""
        from wpimath.kinematics import ChassisSpeeds

        if field_relative:
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speed, y_speed, rot, self.get_heading()
            )
        else:
            speeds = ChassisSpeeds(x_speed, y_speed, rot)

        states = self.kinematics.toSwerveModuleStates(speeds)
        SwerveDrive4Kinematics.desaturateWheelSpeeds(states, 5.0) # n=5 is max speed m/s

        self.front_left.set_state(states[0])
        self.front_right.set_state(states[1])
        self.back_left.set_state(states[2])
        self.back_right.set_state(states[3])






