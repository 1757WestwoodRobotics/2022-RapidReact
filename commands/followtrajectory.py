from math import pi
from commands2 import CommandBase
from pathplannerlib import PathPlannerTrajectory
from wpilib import Timer
from wpimath.controller import (
    PIDController,
    ProfiledPIDControllerRadians,
    HolonomicDriveController,
)
from wpimath.trajectory import TrapezoidProfileRadians

from subsystems.drivesubsystem import DriveSubsystem
import constants


class FollowTrajectory(CommandBase):
    def __init__(
        self, drive: DriveSubsystem, trajectory: PathPlannerTrajectory
    ) -> None:
        CommandBase.__init__(self)

        self.drive = drive

        self.xController = PIDController(
            constants.kTrajectoryPositionPGain,
            constants.kTrajectoryPositionIGain,
            constants.kTrajectoryPositionDGain,
        )
        self.yController = PIDController(
            constants.kTrajectoryPositionPGain,
            constants.kTrajectoryPositionIGain,
            constants.kTrajectoryPositionDGain,
        )
        self.thetaController = ProfiledPIDControllerRadians(
            constants.kTrajectoryAnglePGain,
            constants.kTrajectoryAngleIGain,
            constants.kTrajectoryAngleDGain,
            TrapezoidProfileRadians.Constraints(
                constants.kMaxRotationAngularVelocity,
                constants.kMaxRotationAngularAcceleration,
            ),
        )
        self.thetaController.enableContinuousInput(-pi, pi)

        self.controller = HolonomicDriveController(
            self.xController, self.yController, self.thetaController
        )

        self.trajectory = trajectory

        self.timer = Timer()

        self.addRequirements([self.drive])
        self.setName(__class__.__name__)

    def initialize(self):
        self.timer.reset()
        self.timer.start()

    def execute(self) -> None:
        curTime = self.timer.get()
        desiredState = self.trajectory.sample(curTime)
        targetChassisSpeeds = self.controller.calculate(
            self.drive.getPose(),
            desiredState.pose,
            desiredState.velocity,
            desiredState.pose.rotation(),
        )

        self.drive.arcadeDriveWithSpeeds(
            targetChassisSpeeds, DriveSubsystem.CoordinateMode.RobotRelative
        )

    def isFinished(self) -> bool:
        return self.timer.hasElapsed(self.trajectory.getTotalTime())

    def end(self, _interrupted: bool) -> None:
        self.drive.arcadeDriveWithFactors(
            0, 0, 0, DriveSubsystem.CoordinateMode.RobotRelative
        )
