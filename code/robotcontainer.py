#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
from commands2 import cmd
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry

from phoenix6 import swerve
from wpilib import DriverStation
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
from wpilib import DriverStation


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self._max_speed = (
            1.0 * TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            0.75
        )  # 3/4 of a rotation per second max angular velocity

        # Create several template requests for various swerve operations.
        # CTR's swerve commands are expensive to create but cheaper to modify,
        # so we create template requests here in the constructor.  Then
        # whenever we want to actually have the swerve subsystem do anything,
        # we take the appropriate template request and modify it, and send the
        # result to the set_control() method to actually run it.
        #
        # We'll create the following template requests:
        #  * self._drive (FieldCentric drive request)
        #  * self._brake (SwerveDriveBrake request)
        #  * self._point (PointWheelsAt request)
        #
        self._drive = (
            # Our drive request template will be a default FieldCentric drive
            # request with the following modifications:
            #  * 5% deadband for both translation and rotation
            #  * use open-loop voltage control for drive motors
            #
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.05)
            .with_rotational_deadband(self._max_angular_rate * 0.05)
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._setspeeds = swerve.requests.ApplyChassisSpeeds()

        self._logger = Telemetry(self._max_speed)

        self._joystick = CommandXboxController(0)

        self.drivetrain = TunerConstants.create_drivetrain()

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure the PathPlanner AutoBuilder last
        self.configure_pp_auto_builder()


    def configure_pp_auto_builder(self):
        # Load the RobotConfig from the GUI settings. You should probably
        # store this in your Constants file.
        config = RobotConfig.fromGUISettings()

        AutoBuilder.configure(
            self.getPose, # Robot pose supplier

            # Method to reset odometry
            # (will be called if your auto has a starting pose)
            self.resetPose,

            # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.getRobotRelativeSpeeds,

            # Method that will drive the robot given ROBOT RELATIVE
            # ChassisSpeeds. Also outputs individual module feedforwards
            lambda speeds, feedforwards: self.driveRobotRelative(speeds, feedforwards),

            # PPHolonomicController is the built-in path following controller
            # for holonomic drive trains (including swerve)
            PPHolonomicDriveController(
                PIDConstants(5.0, 0.0, 0.0), # Translation PID constants
                PIDConstants(5.0, 0.0, 0.0) # Rotation PID constants
            ),

            config, # The robot configuration

            # Supplier to control path flipping based on alliance color
            self.shouldFlipPath,

            # Reference to drivetrain subsystem to set requirements
            self.drivetrain
        )


    def shouldFlipPath():
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed


    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        -self._joystick.getLeftY() * self._max_speed
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        -self._joystick.getLeftX() * self._max_speed
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        -self._joystick.getRightX() * self._max_angular_rate
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )

        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )

        self._joystick.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._joystick.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX())
                )
            )
        )

        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        (self._joystick.back() & self._joystick.y()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.back() & self._joystick.x()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self._joystick.start() & self._joystick.y()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.start() & self._joystick.x()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )

        # reset the field-centric heading on left bumper press
        self._joystick.leftBumper().onTrue(
            self.drivetrain.runOnce(self.drivetrain.seed_field_centric)
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        # Simple drive forward auton
        idle = swerve.requests.Idle()
        return cmd.sequence(
            # Reset our field centric heading to match the robot
            # facing away from our alliance station wall (0 deg).
            self.drivetrain.runOnce(
                lambda: self.drivetrain.seed_field_centric(Rotation2d.fromDegrees(0))
            ),
            # Then slowly drive forward (away from us) for 5 seconds.
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(0.5)
                    .with_velocity_y(0)
                    .with_rotational_rate(0)
                )
            )
            .withTimeout(5.0),
            # Finally idle for the rest of auton
            self.drivetrain.apply_request(lambda: idle)
        )


    def getPose(self):
        return self.drivetrain.get_state().pose


    def resetPose(self, pose):
        return self.drivetrain.reset_pose(pose)


    def getRobotRelativeSpeeds(self):
        return self.drivetrain.get_state().speeds


    def driveRobotRelative(self, speeds, feedforwards=None):
        request = self._setspeeds.with_speeds(speeds)
        self.drivetrain.set_control(request)
