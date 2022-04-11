from os import path

from commands2 import ParallelCommandGroup, SequentialCommandGroup, WaitCommand
from wpimath.trajectory import TrajectoryConfig, TrajectoryUtil

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.indexersubsystem import IndexerSubsystem
from subsystems.shootersubsystem import ShooterSubsystem

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

        pathA = TrajectoryUtil.fromPathweaverJson(
            path.join(
                path.dirname(path.realpath(__file__)),
                "..",
                "..",
                "deploy",
                "pathplanner",
                "generatedJSON",
                "5bR-standard-a.wpilib.json",
            )
        )
        pathB = TrajectoryUtil.fromPathweaverJson(
            path.join(
                path.dirname(path.realpath(__file__)),
                "..",
                "..",
                "deploy",
                "pathplanner",
                "generatedJSON",
                "5bR-standard-b.wpilib.json",
            )
        )
        pathC = TrajectoryUtil.fromPathweaverJson(
            path.join(
                path.dirname(path.realpath(__file__)),
                "..",
                "..",
                "deploy",
                "pathplanner",
                "generatedJSON",
                "5bR-standard-c.wpilib.json",
            )
        )
        pathD = TrajectoryUtil.fromPathweaverJson(
            path.join(
                path.dirname(path.realpath(__file__)),
                "..",
                "..",
                "deploy",
                "pathplanner",
                "generatedJSON",
                "5bR-standard-d.wpilib.json",
            )
        )

        super().__init__(
            ResetDrive(drive, pathA.initialPose()),
            HoldBall(indexer),
            DeployIntake(intake),
            FollowTrajectory(drive, pathA),  # pickup ball 2
            WaitCommand(constants.kAutoTimeFromStopToShoot),
            RetractIntake(intake),
            FeedForward(indexer),  # shoot balls 1 and 2
            WaitCommand(constants.kAutoTimeFromShootToMove),
            DeployIntake(intake),
            HoldBall(indexer),
            FollowTrajectory(drive, pathB),  # pickup ball 3
            WaitCommand(constants.kAutoTimeFromStopToShoot),
            RetractIntake(intake),
            FeedForward(indexer),  # shoot ball 3
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
            FeedForward(indexer),  # shoot balls 4 and 5
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
        self.setName(__class__.__name__)
        super().__init__(
            AimShooterToTarget(shooter), FiveBRMovements(drive, intake, indexer)
        )
