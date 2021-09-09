import commands2
import wpimath.geometry

import math
from enum import Enum, auto

import constants
from subsystems.drivesubsystem import DriveSubsystem

from units import units


class DriveDistance(commands2.CommandBase):
    class Axis(Enum):
        X = auto()
        Y = auto()

    def __init__(
        self, distance, speedFactor, axis: Axis, drive: DriveSubsystem
    ) -> None:
        super().__init__()
        distanceMag = distance.magnitude
        self.distance = (
            math.copysign(distanceMag, distanceMag * speedFactor) * distance.units
        )
        self.speedFactor = math.copysign(speedFactor, distanceMag * speedFactor)
        self.axis = axis
        self.drive = drive
        self.addRequirements(drive)

    def initialize(self) -> None:
        currentPose = self.drive.odometry.getPose()
        if self.axis is DriveDistance.Axis.X:
            self.targetPose = currentPose + wpimath.geometry.Transform2d(
                self.distance.to(units.meters).magnitude, 0, 0
            )
        elif self.axis is DriveDistance.Axis.Y:
            self.targetPose = currentPose + wpimath.geometry.Transform2d(
                0, self.distance.to(units.meters).magnitude, 0
            )
        self.updateDistanceToTarget()

    def execute(self) -> None:
        self.updateDistanceToTarget()
        if self.axis is DriveDistance.Axis.X:
            self.drive.arcadeDriveWithFactors(self.speedFactor, 0, 0)
        elif self.axis is DriveDistance.Axis.Y:
            self.drive.arcadeDriveWithFactors(0, self.speedFactor, 0)

    def end(self, interrupted: bool) -> None:
        self.drive.arcadeDriveWithFactors(0, 0, 0)

    def isFinished(self) -> bool:
        return self.distanceToTarget < constants.kAutoDistanceThreshold

    def updateDistanceToTarget(self) -> None:
        currentPose = self.drive.odometry.getPose()
        self.distanceToTarget = (
            currentPose.translation().distance(self.targetPose.translation())
        ) * units.meters
