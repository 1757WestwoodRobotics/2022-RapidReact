from os import path

from commands2 import SequentialCommandGroup, WaitCommand, ParallelCommandGroup
from wpimath.trajectory import TrajectoryConfig, TrajectoryUtil
from commands.shooter.aimshootertotarget import AimShooterToTarget


from subsystems.drivesubsystem import DriveSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.indexersubsystem import IndexerSubsystem

from commands.intake.deployintake import DeployIntake
from commands.intake.retractintake import RetractIntake
from commands.followtrajectory import FollowTrajectory
from commands.indexer.feedforward import FeedForward
from commands.indexer.holdball import HoldBall
from commands.normalballpath import NormalBallPath
from commands.reverseballpath import ReverseBallPath
from commands.resetdrive import ResetDrive

import constants
from subsystems.shootersubsystem import ShooterSubsystem


class TwoBLHangerOuttakeMovements(SequentialCommandGroup):
    def __init__(
        self, drive: DriveSubsystem, intake: IntakeSubsystem, indexer: IndexerSubsystem
    ):

        trajectoryConfig = TrajectoryConfig(
            constants.kMaxForwardLinearVelocity, constants.kMaxForwardLinearAcceleration
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
                "2bL-hangerouttake-a.wpilib.json",
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
                "2bL-hangerouttake-b.wpilib.json",
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
                "2bL-hangerouttake-c.wpilib.json",
            )
        )

        super().__init__(
            ResetDrive(drive, pathA.sample(0).pose),
            DeployIntake(intake),
            FollowTrajectory(drive, pathA),  # pickup ball 2
            WaitCommand(constants.kAutoTimeFromStopToShoot),
            RetractIntake(intake),
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
