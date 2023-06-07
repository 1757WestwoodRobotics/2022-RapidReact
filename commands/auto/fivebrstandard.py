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


import constants


class FiveBRMovements(SequentialCommandGroup):
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

        pathA = trajectoryFromFile("5bR-standard-a")
        pathB = trajectoryFromFile("5bR-standard-b")
        pathC = trajectoryFromFile("5bR-standard-c")
        pathD = trajectoryFromFile("5bR-standard-d")

        super().__init__(
            ResetDrive(drive, pathA.getInitialState().pose),
            HoldBall(indexer),
            DeployIntake(intake),  # pickup second preload
            WaitCommand(constants.kAutoTimeFromStopToShoot * 2),
            FeedForward(indexer),  # shoot balls 1 and 2
            WaitCommand(constants.kAutoTimeFromShootToMove),
            HoldBall(indexer),
            FollowTrajectory(drive, pathA),  # pickup ball 3
            FollowTrajectory(drive, pathB),  # pickup ball 4
            WaitCommand(constants.kAutoTimeFromStopToShoot),
            RetractIntake(intake),
            FeedForward(indexer),  # shoot ball 3/4
            WaitCommand(constants.kAutoTimeFromShootToMove),
            HoldBall(indexer),
            DeployIntake(intake),
            FollowTrajectory(drive, pathC),  # go to terminal, pickup terminal ball
            WaitCommand(
                constants.kAutoTerminalWaitTime
            ),  # wait for user ball in terminal
            RetractIntake(intake),
            FollowTrajectory(drive, pathD),  # go back to shooting range
            WaitCommand(constants.kAutoTimeFromStopToShoot),
            FeedForward(indexer),  # shoot balls 5/6
            WaitCommand(constants.kAutoTimeFromShootToMove),
            HoldBall(indexer),
        )


class FiveBRStandard(ParallelCommandGroup):
    def __init__(
        self,
        shooter: ShooterSubsystem,
        drive: DriveSubsystem,
        intake: IntakeSubsystem,
        indexer: IndexerSubsystem,
    ):
        super().__init__(
            AimShooterToTarget(shooter), FiveBRMovements(drive, intake, indexer)
        )
