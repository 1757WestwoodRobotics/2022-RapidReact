import wpilib
from wpimath.geometry import Pose2d
import commands2
import commands2.button


import constants

from commands.resetdrive import ResetDrive
from commands.complexauto import ComplexAuto
from commands.indexer.feedforward import FeedForward
from commands.drivedistance import DriveDistance
from commands.drivetotarget import DriveToTarget
from commands.targetrelativedrive import TargetRelativeDrive
from commands.robotrelativedrive import RobotRelativeDrive
from commands.absoluterelativedrive import AbsoluteRelativeDrive
from commands.climber.pivotclimbersformid import PivotBothClimbersToVertical
from commands.climber.moveclimberstomiddlerungcaptureposition import (
    MoveBothClimbersToMiddleRungCapturePositionMovements,
)
from commands.climber.moveclimberstomiddlerunghangposition import (
    MoveBothClimbersToMiddleRungHangPosition,
)
from commands.climber.holdcimbersposition import (
    HoldBothClimbersPosition,
    HoldLeftClimberPosition,
)
from commands.climber.moveclimberstofullhangingposition import (
    MoveLeftClimberToFullHangingPosition,
    MoveRightClimberToFullHangingPosition,
)
from commands.climber.moveclimberstonextrungcaptureposition import (
    MoveLeftClimberToNextRungCapturePosition,
    MoveRightClimberToNextRungCapturePosition,
)
from commands.climber.holdcimbersposition import HoldRightClimberPosition

from commands.reverseballpath import ReverseBallPath
from commands.normalballpath import NormalBallPath
from commands.shootball import ShootBall
from commands.defensestate import DefenseState

from commands.indexer.holdball import HoldBall
from commands.intake.autoballintake import AutoBallIntake
from commands.intake.deployintake import DeployIntake
from commands.intake.retractintake import RetractIntake
from commands.shooter.aimshootertotarget import AimShooterToTarget
from commands.shooter.aimshootermanual import AimShooterManually
from commands.shooter.tarmacshot import TarmacShot
from commands.shooter.stopaimsystem import StopMovingParts

from commands.auto.fivebrstandard import FiveBRStandard
from commands.auto.fourblnoninvasive import FourBLNoninvasive
from commands.auto.twoblhangerouttake import TwoBLHangerOuttake
from commands.auto.threebrstandard import ThreeBRStandard

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.visionsubsystem import VisionSubsystem
from subsystems.climbers.leftclimbersubsystem import LeftClimber
from subsystems.climbers.rightclimbersubsystem import RightClimber
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.indexersubsystem import IndexerSubsystem
from subsystems.shootersubsystem import ShooterSubsystem

from operatorinterface import OperatorInterface
from util.helpfultriggerwrappers import AxisButton, SmartDashboardButton


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
        self.shooter = ShooterSubsystem()
        self.intake = IntakeSubsystem()
        self.indexer = IndexerSubsystem()

        # Autonomous routines

        # A simple auto routine that drives forward a specified distance, and then stops.
        self.simpleAuto = commands2.ParallelCommandGroup(
            commands2.SequentialCommandGroup(
                ResetDrive(self.drive),
                HoldBall(self.indexer),
                DeployIntake(self.intake),
                DriveDistance(
                    4 * constants.kWheelCircumference,
                    constants.kAutoDriveSpeedFactor,
                    DriveDistance.Axis.X,
                    self.drive,
                ),
                RetractIntake(self.intake),
                commands2.WaitCommand(2),
                FeedForward(self.indexer),
                commands2.WaitCommand(2),
                HoldBall(self.indexer),
            ),
            AimShooterToTarget(self.shooter),
        )

        # A complex auto routine that drives to the target, drives forward, waits, drives back
        self.complexAuto = ComplexAuto(self.drive)

        # A routine that drives to the target with a given offset
        self.driveToTarget = DriveToTarget(self.drive, constants.kAutoTargetOffset)

        # A routine that follows a set trajectory
        self.fiveBRStandard = FiveBRStandard(
            self.shooter, self.drive, self.intake, self.indexer
        )
        self.twoBLHangerOuttake = TwoBLHangerOuttake(
            self.shooter, self.drive, self.intake, self.indexer
        )
        self.fourBLNoninvasive = FourBLNoninvasive(
            self.shooter, self.drive, self.intake, self.indexer
        )
        self.threeBRStandard = ThreeBRStandard(
            self.shooter, self.drive, self.intake, self.indexer
        )

        # Chooser
        self.chooser = wpilib.SendableChooser()

        # Add commands to the autonomous command chooser
        self.chooser.addOption("Complex Auto", self.complexAuto)
        self.chooser.addOption("Target Auto", self.driveToTarget)
        self.chooser.addOption(
            "2 Ball Left Hanger Outtake Auto", self.twoBLHangerOuttake
        )
        self.chooser.addOption("4 Ball Left Noninvasive Auto", self.fourBLNoninvasive)
        self.chooser.addOption("5 Ball Right Standard Auto", self.fiveBRStandard)
        self.chooser.addOption("3 Ball Right Standard Auto", self.threeBRStandard)
        self.chooser.setDefaultOption("Simple Auto", self.simpleAuto)

        # Put the chooser on the dashboard
        wpilib.SmartDashboard.putData("Autonomous", self.chooser)

        self.configureButtonBindings()

        self.drive.setDefaultCommand(
            AbsoluteRelativeDrive(
                self.drive,
                lambda: self.operatorInterface.chassisControls.forwardsBackwards()
                * constants.kNormalSpeedMultiplier,
                lambda: self.operatorInterface.chassisControls.sideToSide()
                * constants.kNormalSpeedMultiplier,
                self.operatorInterface.chassisControls.rotationX,
                self.operatorInterface.chassisControls.rotationY,
            )
        )

        self.rightClimber.setDefaultCommand(HoldRightClimberPosition(self.rightClimber))
        self.leftClimber.setDefaultCommand(HoldLeftClimberPosition(self.leftClimber))
        self.shooter.setDefaultCommand(AimShooterToTarget(self.shooter))

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

        commands2.button.JoystickButton(*self.operatorInterface.turboSpeed).whenHeld(
            AbsoluteRelativeDrive(
                self.drive,
                lambda: self.operatorInterface.chassisControls.forwardsBackwards()
                * constants.kTurboSpeedMultiplier,
                lambda: self.operatorInterface.chassisControls.sideToSide()
                * constants.kTurboSpeedMultiplier,
                self.operatorInterface.chassisControls.rotationX,
                self.operatorInterface.chassisControls.rotationY,
            )
        )

        commands2.button.JoystickButton(
            *self.operatorInterface.fieldRelativeCoordinateModeControl
        ).toggleWhenPressed(
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

        commands2.button.JoystickButton(*self.operatorInterface.resetGyro).whenPressed(
            ResetDrive(self.drive, Pose2d(0, 0, 0))
        )

        commands2.button.JoystickButton(
            *self.operatorInterface.defenseStateControl
        ).whileHeld(DefenseState(self.drive))

        commands2.button.JoystickButton(
            *self.operatorInterface.driveToTargetControl
        ).whenHeld(DriveToTarget(self.drive, constants.kAutoTargetOffset))

        commands2.button.JoystickButton(
            *self.operatorInterface.pivotBothClimbers
        ).whenPressed(
            PivotBothClimbersToVertical(self.leftClimber, self.rightClimber),
        ).whenPressed(
            StopMovingParts(self.indexer, self.shooter)
        )

        commands2.button.JoystickButton(
            *self.operatorInterface.moveBothClimbersToMiddleRungCapturePosition
        ).whenPressed(
            MoveBothClimbersToMiddleRungCapturePositionMovements(
                self.leftClimber, self.rightClimber
            )
        )
        commands2.button.JoystickButton(
            *self.operatorInterface.moveBothClimbersToMiddleRungHangPosition
        ).whenPressed(
            MoveBothClimbersToMiddleRungHangPosition(
                self.leftClimber, self.rightClimber
            )
        )
        commands2.button.JoystickButton(
            *self.operatorInterface.holdBothClimbersPosition
        ).whenPressed(HoldBothClimbersPosition(self.leftClimber, self.rightClimber))

        commands2.button.JoystickButton(
            *self.operatorInterface.leftClimberToNextRungCapturePosition
        ).whenPressed(MoveLeftClimberToNextRungCapturePosition(self.leftClimber))

        commands2.button.JoystickButton(
            *self.operatorInterface.rightClimberToNextRungCapturePosition
        ).whenPressed(MoveRightClimberToNextRungCapturePosition(self.rightClimber))

        commands2.button.JoystickButton(
            *self.operatorInterface.leftClimberToHangingPosition
        ).whenPressed(MoveLeftClimberToFullHangingPosition(self.leftClimber))

        commands2.button.JoystickButton(
            *self.operatorInterface.rightClimberToHangingPosition
        ).whenPressed(MoveRightClimberToFullHangingPosition(self.rightClimber))

        commands2.button.JoystickButton(
            *self.operatorInterface.autoBallIntakeControl
        ).whenHeld(AutoBallIntake(self.drive, self.intake))

        commands2.button.JoystickButton(*self.operatorInterface.shootBall).whenHeld(
            ShootBall(self.indexer)
        ).whenReleased(HoldBall(self.indexer))

        SmartDashboardButton(constants.kShootingManualModeKey).whileHeld(
            AimShooterManually(self.shooter)
        )

        commands2.button.JoystickButton(*self.operatorInterface.tarmacShot).whileHeld(
            TarmacShot(self.shooter)
        )

    def getAutonomousCommand(self) -> commands2.Command:
        return self.chooser.getSelected()
