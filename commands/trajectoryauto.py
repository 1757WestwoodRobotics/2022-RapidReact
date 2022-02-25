from commands2 import SequentialCommandGroup
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig
from commands.followtrajectory import FollowTrajectory
from commands.resetdrive import ResetDrive

from subsystems.drivesubsystem import DriveSubsystem
import constants


class TrajectoryAuto(SequentialCommandGroup):
    def __init__(self, drive: DriveSubsystem) -> None:
        self.drive = drive
        trajectoryConfig = TrajectoryConfig(
            constants.kMaxForwardLinearVelocity, constants.kMaxForwardLinearAcceleration
        )
        trajectoryConfig.setKinematics(self.drive.kinematics)

        trajectory = TrajectoryGenerator.generateTrajectory(
            Pose2d(0, 0, Rotation2d(0)),
            [Translation2d(1, 0), Translation2d(1, -1)],
            Pose2d(2, -1, Rotation2d.fromDegrees(180)),
            trajectoryConfig,
        )

        super().__init__(
            ResetDrive(self.drive), FollowTrajectory(self.drive, trajectory)
        )
