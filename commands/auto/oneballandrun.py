from commands2 import ParallelCommandGroup, SequentialCommandGroup, WaitCommand
from wpimath.trajectory import TrajectoryConfig

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.indexersubsystem import IndexerSubsystem
from subsystems.shootersubsystem import ShooterSubsystem

from commands.auto.autohelper import trajectoryFromFile
from commands.resetdrive import ResetDrive
from commands.intake.retractintake import RetractIntake
from commands.intake.deployintake import DeployIntake
from commands.indexer.feedforward import FeedForward
from commands.indexer.holdball import HoldBall
from commands.followtrajectory import FollowTrajectory
from commands.shooter.aimshootertotarget import AimShooterToTarget


import constants


class OneBARunMovements(SequentialCommandGroup):
    def __init__(
        self,
        drive: DriveSubsystem,
        intake: IntakeSubsystem,
        indexer: IndexerSubsystem,
    ):

        trajectoryConfig = TrajectoryConfig(
            constants.kMaxForwardLinearVelocity,
            constants.kMaxForwardLinearAcceleration,
        )
        trajectoryConfig.setKinematics(drive.kinematics)

        path = trajectoryFromFile("One Ball Run")

        super().__init__(
            ResetDrive(drive, path.getInitialState().pose),
            HoldBall(indexer),
            DeployIntake(intake),  # pickup ball 2
            WaitCommand(constants.kAutoTimeFromStopToShoot),
            FeedForward(indexer),  # shoot balls 1 and 2
            WaitCommand(constants.kAutoTimeFromShootToMove),
            FollowTrajectory(drive, path),  # get away!
            WaitCommand(constants.kAutoTimeFromStopToShoot),
            HoldBall(indexer),
            RetractIntake(intake),
        )


class OneBARun(ParallelCommandGroup):
    def __init__(
        self,
        shooter: ShooterSubsystem,
        drive: DriveSubsystem,
        intake: IntakeSubsystem,
        indexer: IndexerSubsystem,
    ):
        super().__init__(
            AimShooterToTarget(shooter),
            OneBARunMovements(drive, intake, indexer),
        )
