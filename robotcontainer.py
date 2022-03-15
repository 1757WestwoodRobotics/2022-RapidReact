import wpilib

import commands2
import commands2.button

import constants

from commands.complexauto import ComplexAuto
from commands.drivedistance import DriveDistance
from commands.drivetotarget import DriveToTarget
from commands.defaultdrive import DefaultDrive
from commands.fieldrelativedrive import FieldRelativeDrive
from commands.targetrelativedrive import TargetRelativeDrive
from commands.resetdrive import ResetDrive
from commands.trajectoryauto import TrajectoryAuto

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.visionsubsystem import VisionSubsystem

from operatorinterface import OperatorInterface


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

        # The operator interface (driver controls)
        self.operatorInterface = OperatorInterface()

        # The robot's subsystems
        self.drive = DriveSubsystem()
        self.vision = VisionSubsystem()

        # Autonomous routines

        # A simple auto routine that drives forward a specified distance, and then stops.
        self.simpleAuto = DriveDistance(
            constants.kAutoDriveDistance,
            constants.kAutoDriveSpeedFactor,
            DriveDistance.Axis.X,
            self.drive,
        )

        # A complex auto routine that drives to the target, drives forward, waits, drives back
        self.complexAuto = ComplexAuto(self.drive)

        # A routine that drives to the target with a given offset
        self.driveToTarget = DriveToTarget(self.drive, constants.kAutoTargetOffset)

        # A routine that follows a set trajectory
        self.fiveBallAuto = TrajectoryAuto(self.drive, constants.kAuto5BallFilename)

        # Chooser
        self.chooser = wpilib.SendableChooser()

        # Add commands to the autonomous command chooser
        self.chooser.setDefaultOption("Complex Auto", self.complexAuto)
        self.chooser.addOption("Simple Auto", self.simpleAuto)
        self.chooser.addOption("Target Auto", self.driveToTarget)
        self.chooser.addOption("5 Ball Auto", self.fiveBallAuto)

        # Put the chooser on the dashboard
        wpilib.SmartDashboard.putData("Autonomous", self.chooser)

        self.configureButtonBindings()

        self.drive.setDefaultCommand(
            DefaultDrive(
                self.drive,
                self.operatorInterface.chassisControls.forwardsBackwards,
                self.operatorInterface.chassisControls.sideToSide,
                self.operatorInterface.chassisControls.rotation,
            )
        )

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        commands2.button.JoystickButton(
            *self.operatorInterface.fieldRelativeCoordinateModeControl
        ).whileHeld(
            FieldRelativeDrive(
                self.drive,
                self.operatorInterface.chassisControls.forwardsBackwards,
                self.operatorInterface.chassisControls.sideToSide,
                self.operatorInterface.chassisControls.rotation,
            )
        )

        commands2.button.JoystickButton(
            *self.operatorInterface.targetRelativeCoordinateModeControl
        ).whileHeld(
            TargetRelativeDrive(
                self.drive,
                self.operatorInterface.chassisControls.forwardsBackwards,
                self.operatorInterface.chassisControls.sideToSide,
                self.operatorInterface.chassisControls.rotation,
            )
        )

        commands2.button.JoystickButton(
            *self.operatorInterface.resetSwerveControl
        ).whenPressed(ResetDrive(self.drive))

        commands2.button.JoystickButton(
            *self.operatorInterface.driveToTargetControl
        ).whenHeld(DriveToTarget(self.drive, constants.kAutoTargetOffset))

    def getAutonomousCommand(self) -> commands2.Command:
        return self.chooser.getSelected()
