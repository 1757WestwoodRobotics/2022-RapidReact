import commands2
import wpilib
import wpimath.geometry

import math

import constants
from pyfrc.physics.units import units
from subsystems.drivesubsystem import DriveSubsystem


class DriveDistance(commands2.CommandBase):
    @units.wraps(None, (None, units.meters, None, None))
    def __init__(self, distance, speedFactor, drive: DriveSubsystem) -> None:
        super().__init__()
        self.distance = math.copysign(distance, distance * speedFactor)
        self.speedFactor = math.copysign(speedFactor, distance * speedFactor)
        self.drive = drive
        self.addRequirements(drive)

    def initialize(self) -> None:
        currentPose = self.drive.odometry.getPose()
        self.targetPose = currentPose + \
            wpimath.geometry.Transform2d(self.distance, 0, 0)
        self.updateDistanceToTarget()

    def execute(self) -> None:
        self.updateDistanceToTarget()
        self.drive.arcadeDriveWithFactors(self.speedFactor, 0, 0)

    def end(self, interrupted: bool) -> None:
        self.drive.arcadeDriveWithFactors(0, 0, 0)

    def isFinished(self) -> bool:
        return self.distanceToTarget < constants.kAutoDistanceThreshold

    def updateDistanceToTarget(self) -> None:
        currentPose = self.drive.odometry.getPose()
        self.distanceToTarget = (currentPose.translation().distance(
            self.targetPose.translation())) * units.meters
