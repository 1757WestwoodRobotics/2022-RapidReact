import wpilib

import commands2
import commands2.button

import constants

from commands.complexauto import ComplexAuto
from commands.drivedistance import DriveDistance
from commands.drivetotarget import DriveToTarget
from commands.targetrelativedrive import TargetRelativeDrive
from commands.robotrelativedrive import RobotRelativeDrive
from commands.absoluterelativedrive import AbsoluteRelativeDrive
from commands.resetdrive import ResetDrive
from commands.climber.fullextend import FullLeftClimber, FullRightClimber
from commands.climber.moveclimberstomiddlerunghangposition import (
    MoveLeftClimberToMiddleRungHangPosition,
    MoveRightClimberToMiddleRungHangPosition,
)
from commands.climber.holdleftclimberextension import HoldLeftClimberExtension
from commands.climber.holdrightclimberextension import HoldRightClimberExtension
from commands.climber.toggleleftpiston import ToggleLeftPiston
from commands.climber.togglerightpiston import ToggleRightPiston
from commands.reverseballpath import ReverseBallPath
from commands.normalballpath import NormalBallPath
from commands.shootball import ShootBall

from commands.indexer.defaultindexer import DefaultIndexer
from commands.indexer.holdball import HoldBall
from commands.intake.defaultintake import DefaultIntake
from commands.intake.autoballintake import AutoBallIntake
from commands.intake.deployintake import DeployIntake
from commands.intake.retractintake import RetractIntake


from subsystems.drivesubsystem import DriveSubsystem
from subsystems.visionsubsystem import VisionSubsystem
from subsystems.climbers.leftclimbersubsystem import LeftClimber
from subsystems.climbers.rightclimbersubsystem import RightClimber
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.indexersubsystem import IndexerSubsystem

from operatorinterface import OperatorInterface
from util.helpfultriggerwrappers import AxisButton


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
        self.leftClimber = LeftClimber()
        self.rightClimber = RightClimber()
        self.intake = IntakeSubsystem()
        self.indexer = IndexerSubsystem()

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

        # Chooser
        self.chooser = wpilib.SendableChooser()

        # Add commands to the autonomous command chooser
        self.chooser.setDefaultOption("Complex Auto", self.complexAuto)
        self.chooser.addOption("Simple Auto", self.simpleAuto)
        self.chooser.addOption("Target Auto", self.driveToTarget)

        # Put the chooser on the dashboard
        wpilib.SmartDashboard.putData("Autonomous", self.chooser)

        self.configureButtonBindings()

        self.drive.setDefaultCommand(
            AbsoluteRelativeDrive(
                self.drive,
                self.operatorInterface.chassisControls.forwardsBackwards,
                self.operatorInterface.chassisControls.sideToSide,
                self.operatorInterface.chassisControls.rotationX,
                self.operatorInterface.chassisControls.rotationY,
            )
        )

        self.rightClimber.setDefaultCommand(
            HoldRightClimberExtension(self.rightClimber)
        )
        self.leftClimber.setDefaultCommand(HoldLeftClimberExtension(self.leftClimber))
        self.intake.setDefaultCommand(DefaultIntake(self.intake))
        self.indexer.setDefaultCommand(DefaultIndexer(self.indexer))

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        AxisButton(
            self.operatorInterface.deployIntakeControl,
            constants.kXboxTriggerActivationThreshold,
        ).whenHeld(DeployIntake(self.intake)).whenReleased(RetractIntake(self.intake))

        (
            AxisButton(
                self.operatorInterface.deployIntakeControl,
                constants.kXboxTriggerActivationThreshold,
            ).and_(
                AxisButton(
                    self.operatorInterface.reverseBallPath,
                    constants.kXboxTriggerActivationThreshold,
                )
            )
        ).whenActive(ReverseBallPath(self.intake, self.indexer))

        (
            AxisButton(
                self.operatorInterface.deployIntakeControl,
                constants.kXboxTriggerActivationThreshold,
            ).and_(
                AxisButton(
                    self.operatorInterface.reverseBallPath,
                    constants.kXboxTriggerActivationThreshold,
                ).not_()
            )
        ).whenActive(
            NormalBallPath(self.intake, self.indexer)
        )  # when let go of just the reverse button, go back to normal ball path

        commands2.button.JoystickButton(
            *self.operatorInterface.fieldRelativeCoordinateModeControl
        ).whileHeld(
            RobotRelativeDrive(
                self.drive,
                self.operatorInterface.chassisControls.forwardsBackwards,
                self.operatorInterface.chassisControls.sideToSide,
                self.operatorInterface.chassisControls.rotationX,
            )
        )

        commands2.button.JoystickButton(
            *self.operatorInterface.targetRelativeCoordinateModeControl
        ).whileHeld(
            TargetRelativeDrive(
                self.drive,
                self.operatorInterface.chassisControls.forwardsBackwards,
                self.operatorInterface.chassisControls.sideToSide,
                self.operatorInterface.chassisControls.rotationX,
            )
        )

        commands2.button.JoystickButton(
            *self.operatorInterface.resetSwerveControl
        ).whenPressed(ResetDrive(self.drive))

        commands2.button.JoystickButton(
            *self.operatorInterface.driveToTargetControl
        ).whenHeld(DriveToTarget(self.drive, constants.kAutoTargetOffset))

        commands2.button.JoystickButton(
            *self.operatorInterface.fullExtendLeftClimber
        ).whenPressed(FullLeftClimber(self.leftClimber)).whenReleased(
            MoveLeftClimberToMiddleRungHangPosition(
                self.leftClimber
            )  # This can swap for Hanging if we get working pistons
        )

        commands2.button.JoystickButton(
            *self.operatorInterface.fullExtendRightClimber
        ).whenPressed(FullRightClimber(self.rightClimber)).whenReleased(
            MoveRightClimberToMiddleRungHangPosition(self.rightClimber)
        )

        commands2.button.JoystickButton(
            *self.operatorInterface.toggleLeftClimbPiston
        ).whenPressed(ToggleLeftPiston(self.leftClimber))

        commands2.button.JoystickButton(
            *self.operatorInterface.toggleRightClimbPiston
        ).whenPressed(ToggleRightPiston(self.rightClimber))

        commands2.button.JoystickButton(
            *self.operatorInterface.autoBallIntakeControl
        ).whenHeld(AutoBallIntake(self.drive, self.intake))

        commands2.button.JoystickButton(*self.operatorInterface.shootBall).whenHeld(
            ShootBall(self.indexer)
        ).whenReleased(HoldBall(self.indexer))

    def getAutonomousCommand(self) -> commands2.Command:
        return self.chooser.getSelected()
