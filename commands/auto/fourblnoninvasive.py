from os import path

from commands2 import SequentialCommandGroup, WaitCommand
from wpimath.trajectory import TrajectoryConfig, TrajectoryUtil

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.indexersubsystem import IndexerSubsystem

from commands.intake.deployintake import DeployIntake
from commands.intake.retractintake import RetractIntake
from commands.followtrajectory import FollowTrajectory
from commands.indexer.feedforward import FeedForward
from commands.indexer.holdball import HoldBall
from commands.resetgyro import ResetGyro

import constants


class FourBLNoninvasive(SequentialCommandGroup):
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
                "4bL-noninvasive-a.wpilib.json",
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
                "4bL-noninvasive-b.wpilib.json",
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
                "4bL-noninvasive-c.wpilib.json",
            )
        )

        super().__init__(
            ResetGyro(drive, pathA.sample(0).pose),
            DeployIntake(intake),
            FollowTrajectory(drive, pathA),  # pickup ball 2
            RetractIntake(intake),
            WaitCommand(constants.kAutoTimeFromStopToShoot),
            FeedForward(indexer),  # shoot balls 1 and 2
            WaitCommand(constants.kAutoTimeFromShootToMove),
            DeployIntake(intake),
            HoldBall(indexer),
            FollowTrajectory(drive, pathB),  # move through hangar to terminal
            RetractIntake(intake),
            FollowTrajectory(drive, pathC),  # move into shooting range
            WaitCommand(constants.kAutoTimeFromStopToShoot),
            FeedForward(indexer),
            WaitCommand(constants.kAutoTimeFromShootToMove),
            HoldBall(indexer),
        )
