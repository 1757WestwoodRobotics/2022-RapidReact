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


class FourBLNoninvasiveMovements(SequentialCommandGroup):
    def __init__(
        self, drive: DriveSubsystem, intake: IntakeSubsystem, indexer: IndexerSubsystem
    ):

        trajectoryConfig = TrajectoryConfig(
            constants.kMaxForwardLinearVelocity, constants.kMaxForwardLinearAcceleration
        )
        trajectoryConfig.setKinematics(drive.kinematics)

        pathA = trajectoryFromFile("4bL-noninvasive-a")
        pathB = trajectoryFromFile("4bL-noninvasive-b")
        pathC = trajectoryFromFile("4bL-noninvasive-c")

        super().__init__(
            ResetDrive(drive, pathA.getInitialState().pose),
            DeployIntake(intake),
            FollowTrajectory(drive, pathA),  # pickup ball 2
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
        super().__init__(
            AimShooterToTarget(shooter),
            FourBLNoninvasiveMovements(drive, intake, indexer),
        )
