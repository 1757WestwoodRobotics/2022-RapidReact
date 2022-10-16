from commands2 import ParallelCommandGroup, SequentialCommandGroup, WaitCommand
from wpimath.trajectory import TrajectoryConfig

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.indexersubsystem import IndexerSubsystem
from subsystems.shootersubsystem import ShooterSubsystem

from commands.auto.autohelper import trajectoryFromFile
from commands.resetdrive import ResetDrive
from commands.intake.retractintake import RetractIntake
from commands.indexer.feedforward import FeedForward
from commands.indexer.holdball import HoldBall
from commands.followtrajectory import FollowTrajectory
from commands.shooter.aimshootertotarget import AimShooterToTarget
from commands.intake.deployintake import DeployIntake
from commands.normalballpath import NormalBallPath
from commands.reverseballpath import ReverseBallPath


import constants


class TwoBLHubspitMovements(SequentialCommandGroup):
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

        pathA = trajectoryFromFile("2bL-hubspit-a")
        pathB = trajectoryFromFile("2bL-hubspit-b")

        super().__init__(
            ResetDrive(drive, pathA.getInitialState().pose),
            HoldBall(indexer),
            DeployIntake(intake),  # pickup second preload
            WaitCommand(constants.kAutoTimeFromStopToShoot),
            FeedForward(indexer),  # shoot balls 1 and 2
            WaitCommand(constants.kAutoTimeFromShootToMove),
            HoldBall(indexer),
            FollowTrajectory(drive, pathA),  # pickup ball 3
            WaitCommand(constants.kAutoTimeFromStopToShoot),
            FeedForward(indexer),  # shoot ball 3
            WaitCommand(constants.kAutoTimeFromShootToMove),
            HoldBall(indexer),
            FollowTrajectory(drive, pathB),  # pickup balls 4 and 5
            WaitCommand(constants.kAutoTimeFromStopToShoot),
            ReverseBallPath(intake, indexer),
            WaitCommand(constants.kAutoTimeFromShootToMove),
            NormalBallPath(intake, indexer),  # spit out balls into hub
            RetractIntake(intake),
            HoldBall(indexer),
        )


class TwoBLHubspit(ParallelCommandGroup):
    def __init__(
        self,
        shooter: ShooterSubsystem,
        drive: DriveSubsystem,
        intake: IntakeSubsystem,
        indexer: IndexerSubsystem,
    ):
        self.setName(__class__.__name__)
        super().__init__(
            AimShooterToTarget(shooter),
            TwoBLHubspitMovements(drive, intake, indexer),
        )
