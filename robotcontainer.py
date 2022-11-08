import wpilib
from wpimath.geometry import Pose2d
import commands2
import commands2.button
from commands.drive.arcardedrive import ArcadeDrive
from commands.drive.curvaturedrive import CurvatureDrive
from commands.drive.pivotdrive import PivotDrive


import constants

from commands.resetdrive import ResetDrive
from commands.complexauto import ComplexAuto
from commands.indexer.feedforward import FeedForward
from commands.indexer.holdball import HoldBall
from commands.drivedistance import DriveDistance
from commands.drivetotarget import DriveToTarget
from commands.drive.absoluterelativedrive import AbsoluteRelativeDrive
from commands.drive.robotrelativedrive import RobotRelativeDrive
from commands.drive.fieldrelativedrive import FieldRelativeDrive
from commands.drive.tankdrive import TankDrive

from commands.reverseballpath import ReverseBallPath
from commands.normalballpath import NormalBallPath
from commands.shootball import ShootBall
from commands.defensestate import DefenseState

from commands.intake.autoballintake import AutoBallIntake
from commands.intake.deployintake import DeployIntake
from commands.intake.retractintake import RetractIntake
from commands.shooter.aimshootertotarget import AimShooterToTarget
from commands.shooter.aimshootermanual import AimShooterManually

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
from subsystems.lightsubsystem import LightSubsystem

from operatorinterface import OperatorInterface
from util.helpfultriggerwrappers import ModifiableJoystickButton


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
        self.lights = LightSubsystem()

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

        self.driveChooser = wpilib.SendableChooser()

        self.driveChooser.setDefaultOption(
            "Absolute Drive",
            AbsoluteRelativeDrive(
                self.drive,
                self.operatorInterface.chassisControls.forwardsBackwards,
                self.operatorInterface.chassisControls.sideToSide,
                self.operatorInterface.chassisControls.rotationX,
                self.operatorInterface.chassisControls.rotationY,
            ),
        )
        self.driveChooser.addOption(
            "Tank Drive",
            TankDrive(
                self.drive,
                self.operatorInterface.chassisControls.forwardsBackwards,
                self.operatorInterface.chassisControls.rotationY,
            ),
        )
        self.driveChooser.addOption(
            "Robot Relative Drive",
            RobotRelativeDrive(
                self.drive,
                self.operatorInterface.chassisControls.forwardsBackwards,
                self.operatorInterface.chassisControls.sideToSide,
                self.operatorInterface.chassisControls.rotationX,
            ),
        )
        self.driveChooser.addOption(
            "Field Relative Drive",
            FieldRelativeDrive(
                self.drive,
                self.operatorInterface.chassisControls.forwardsBackwards,
                self.operatorInterface.chassisControls.sideToSide,
                self.operatorInterface.chassisControls.rotationX,
            ),
        )
        self.driveChooser.addOption(
            "Arcade Drive",
            ArcadeDrive(
                self.drive,
                self.operatorInterface.chassisControls.forwardsBackwards,
                self.operatorInterface.chassisControls.sideToSide,
            ),
        )
        self.driveChooser.addOption(
            "Curvature Drive",
            CurvatureDrive(
                self.drive,
                self.operatorInterface.chassisControls.forwardsBackwards,
                self.operatorInterface.chassisControls.sideToSide,
            ),
        )
        self.driveChooser.addOption(
            "Pivot Drive",
            PivotDrive(
                self.drive,
                self.operatorInterface.chassisControls.forwardsBackwards,
                self.operatorInterface.chassisControls.sideToSide,
            ),
        )

        wpilib.SmartDashboard.putData("Drivetrain", self.driveChooser)

        self.shooter.setDefaultCommand(AimShooterManually(self.shooter))

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        ModifiableJoystickButton(self.operatorInterface.deployIntakeControl,).whenHeld(
            DeployIntake(self.intake)
        ).whenReleased(RetractIntake(self.intake))

        (
            ModifiableJoystickButton(self.operatorInterface.deployIntakeControl,).and_(
                ModifiableJoystickButton(
                    self.operatorInterface.reverseBallPath,
                )
            )
        ).whenActive(ReverseBallPath(self.intake, self.indexer))

        (
            ModifiableJoystickButton(self.operatorInterface.deployIntakeControl,).and_(
                ModifiableJoystickButton(
                    self.operatorInterface.reverseBallPath,
                ).not_()
            )
        ).whenActive(
            NormalBallPath(self.intake, self.indexer)
        )  # when let go of just the reverse button, go back to normal ball path

        ModifiableJoystickButton(self.operatorInterface.resetGyro).whenPressed(
            ResetDrive(self.drive, Pose2d(0, 0, 0))
        )

        ModifiableJoystickButton(
            self.operatorInterface.defenseStateControl
        ).whenPressed(DefenseState(self.drive))

        ModifiableJoystickButton(self.operatorInterface.driveToTargetControl).whenHeld(
            DriveToTarget(self.drive, constants.kAutoTargetOffset)
        )

        ModifiableJoystickButton(self.operatorInterface.autoBallIntakeControl).whenHeld(
            AutoBallIntake(self.drive, self.intake)
        )

        ModifiableJoystickButton(self.operatorInterface.shootBall).whenHeld(
            ShootBall(self.indexer)
        ).whenReleased(HoldBall(self.indexer))

    def getAutonomousCommand(self) -> commands2.Command:
        return self.chooser.getSelected()

    def getDriveCommand(self) -> commands2.Command:
        return self.driveChooser.getSelected()
