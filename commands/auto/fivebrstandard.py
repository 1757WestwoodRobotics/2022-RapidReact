from commands2 import SequentialCommandGroup, WaitCommand
from commands.followtrajectory import FollowTrajectory
from os import path
from wpimath.trajectory import TrajectoryConfig, TrajectoryUtil
from commands.indexer.feedforward import FeedForward
from commands.indexer.holdball import HoldBall
from commands.resetdrive import ResetDrive

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.indexersubsystem import IndexerSubsystem

from commands.intake.deployintake import DeployIntake
from commands.intake.retractintake import RetractIntake
import constants


class FiveBRStandard(SequentialCommandGroup):
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
            ResetDrive(drive, pathA.sample(0).pose),
            DeployIntake(intake),
            FollowTrajectory(drive, pathA),
            RetractIntake(intake),
            FeedForward(indexer),
            WaitCommand(2),
            DeployIntake(intake),
            HoldBall(indexer),
            FollowTrajectory(drive, pathB),
            RetractIntake(intake),
            FeedForward(indexer),
            WaitCommand(2),
            DeployIntake(intake),
            HoldBall(indexer),
            FollowTrajectory(drive, pathC),
            WaitCommand(2),
            FollowTrajectory(drive, pathD),
            RetractIntake(intake),
            FeedForward(indexer),
        )
