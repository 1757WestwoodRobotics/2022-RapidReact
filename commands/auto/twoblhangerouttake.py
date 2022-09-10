
from commands2 import ParallelCommandGroup, SequentialCommandGroup, WaitCommand
from wpimath.trajectory import TrajectoryConfig

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.indexersubsystem import IndexerSubsystem
from subsystems.shootersubsystem import ShooterSubsystem

from commands.auto.autohelper import trajectoryFromFile
from commands.resetdrive import ResetDrive
from commands.intake.deployintake import DeployIntake
from commands.intake.retractintake import RetractIntake
from commands.indexer.feedforward import FeedForward
from commands.indexer.holdball import HoldBall
from commands.followtrajectory import FollowTrajectory
from commands.shooter.aimshootertotarget import AimShooterToTarget
from commands.reverseballpath import ReverseBallPath
from commands.normalballpath import NormalBallPath


import constants


class TwoBLHangerOuttakeMovements(SequentialCommandGroup):
    def __init__(
        self, drive: DriveSubsystem, intake: IntakeSubsystem, indexer: IndexerSubsystem
    ):

        trajectoryConfig = TrajectoryConfig(
            constants.kMaxForwardLinearVelocity, constants.kMaxForwardLinearAcceleration
        )
        trajectoryConfig.setKinematics(drive.kinematics)

        pathA = trajectoryFromFile("2bL-hangerouttake-a")
        pathB = trajectoryFromFile("2bL-hangerouttake-b")
        pathC = trajectoryFromFile("2bL-hangerouttake-c")

        super().__init__(
            ResetDrive(drive, pathA.initialPose()),
            DeployIntake(intake),
            FollowTrajectory(drive, pathA),  # pickup ball 2
            WaitCommand(constants.kAutoTimeFromStopToShoot),
            FeedForward(indexer),
            WaitCommand(constants.kAutoTimeFromShootToMove),
            HoldBall(indexer),
            DeployIntake(intake),
            FollowTrajectory(drive, pathB),  # grab the red ball
            FollowTrajectory(drive, pathC),
            ReverseBallPath(intake, indexer),
            WaitCommand(
                constants.kAutoTimeFromShootToMove
            ),  # Make sure it gets ejected
            NormalBallPath(intake, indexer),
            RetractIntake(intake),
        )


class TwoBLHangerOuttake(ParallelCommandGroup):
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
            TwoBLHangerOuttakeMovements(drive, intake, indexer),
        )
