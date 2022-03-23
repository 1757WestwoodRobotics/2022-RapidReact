from enum import Enum, auto
from commands2 import CommandBase
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d, Transform2d, Rotation2d
from operatorinterface import Control2D

from subsystems.shootingsubsystem import ShootingSubsystem
import constants
from util.angleoptimize import optimizeAngle
from util.convenientmath import rotationFromTranslation


class AimShooterToTarget(CommandBase):
    def __init__(self, shooter: ShootingSubsystem, shooterOffset: Control2D) -> None:
        SmartDashboard.putBoolean(constants.kShootingManualModeKey, False)
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.shooter = shooter
        self.distanceAndRotationOffset = shooterOffset
        self.addRequirements([self.shooter])

    def execute(self) -> None:
        if SmartDashboard.getBoolean(constants.kShootingManualModeKey, False):
            self.manualMode()
        else:
            self.normalMode()

    def manualMode(self) -> None:
        wheelSpeed = SmartDashboard.getNumber(constants.kShootingWheelSpeedKey, 0)
        hoodAngle = SmartDashboard.getNumber(constants.kShootingHoodAngleKey, 0)
        turretPosition = SmartDashboard.getNumber(constants.kShootingTurretAngleKey, 0)
        self.shooter.setWheelSpeed(wheelSpeed)
        self.shooter.setHoodAngle(Rotation2d.fromDegrees(hoodAngle))
        self.shooter.rotateTurret(
            Rotation2d.fromDegrees(turretPosition)
            + constants.kOffsetAngleRange * self.distanceAndRotationOffset.sideToSide()
        )

    def normalMode(self) -> None:
        deltaPos = Pose2d(
            *[
                constants.kRobotUpdateRate * element * constants.kPredictiveAimGain
                for element in SmartDashboard.getNumberArray(
                    constants.kDriveVelocityKeys, [0, 0, 0]
                )
            ]
        )
        currentPose = Pose2d(
            *SmartDashboard.getNumberArray(
                constants.kRobotPoseArrayKeys.valueKey, [0, 0, 0]
            )
        )

        staticDifference = Transform2d(currentPose, constants.kSimDefaultTargetLocation)
        staticRotation = rotationFromTranslation(staticDifference.translation())

        ## NOTE: untested moving code, currently not in use
        newPosition = Pose2d(
            deltaPos.X() + currentPose.X(),
            deltaPos.Y() + currentPose.Y(),
            deltaPos.rotation() + currentPose.rotation(),
        )

        movingDifference = Transform2d(newPosition, constants.kSimDefaultTargetLocation)
        movingRotation = rotationFromTranslation(movingDifference.translation())

        deltaRotation = movingRotation - staticRotation

        if SmartDashboard.getBoolean(
            constants.kTargetDistanceRelativeToRobotKeys.validKey, False
        ):  # if we can obtain a distance, use that information
            distance = (
                SmartDashboard.getNumber(
                    constants.kTargetDistanceRelativeToRobotKeys.valueKey, 0.0
                )
                + constants.kOffsetDistanceRange
                * self.distanceAndRotationOffset.forwardsBackwards()
            )

            targetSpeed = constants.kShootingMappingFunction(distance)
            self.shooter.setWheelSpeed(targetSpeed)

            targetAngle = Rotation2d.fromDegrees(
                constants.kHoodMappingFunction(distance)
            )
            self.shooter.setHoodAngle(targetAngle)

        if SmartDashboard.getBoolean(
            constants.kTargetAngleRelativeToRobotKeys.validKey, False
        ):  # if we have an angle, use the relative angle to adjust the turret for precision
            angle = (
                SmartDashboard.getNumber(
                    constants.kTargetAngleRelativeToRobotKeys.valueKey, 0.0
                )
                + constants.kOffsetAngleRange.degrees()
                * self.distanceAndRotationOffset.sideToSide()
            )

            self.shooter.trackTurret(
                angle + deltaRotation.degrees()
            )  # always track the turret
        else:  # ...otherwise use odometry to estimate where the target SHOULD be
            self.shooter.rotateTurret(
                optimizeAngle(
                    constants.kTurretRelativeForwardAngle,
                    staticRotation
                    + constants.kOffsetAngleRange
                    * self.distanceAndRotationOffset.sideToSide(),
                )
                + constants.kTurretOffsetFromRobotAngle  # shooter 0 is robot 180
            )
