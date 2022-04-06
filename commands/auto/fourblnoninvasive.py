from os import path

from commands2 import ParallelCommandGroup, SequentialCommandGroup, WaitCommand
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
from commands.resetdrive import ResetDrive

import constants
from subsystems.shootersubsystem import ShooterSubsystem


class FourBLNoninvasiveMovements(SequentialCommandGroup):
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
            ResetDrive(drive, pathA.sample(0).pose),
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


class FourBLNoninvasive(ParallelCommandGroup):
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
            FourBLNoninvasiveMovements(drive, intake, indexer),
        )
