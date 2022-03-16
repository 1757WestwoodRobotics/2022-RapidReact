from commands2 import CommandBase
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d, Transform2d, Rotation2d
from operatorinterface import Control2D

from subsystems.shootingsubsystem import ShootingSubsystem
import constants
from util.angleoptimize import optimizeAngle
from util.convenientmath import rotationFromTranslation


class AimSystem(CommandBase):
    def __init__(self, shoot: ShootingSubsystem, shooterOffset: Control2D) -> None:
        SmartDashboard.putBoolean(constants.kShootingManualModeKey, False)
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.shoot = shoot
        self.offset = shooterOffset
        self.addRequirements([self.shoot])

    def execute(self) -> None:
        if SmartDashboard.getBoolean(constants.kShootingManualModeKey, False):
            self.manualMode()
        else:
            self.normalMode()

    def manualMode(self) -> None:
        wheelSpeed = SmartDashboard.getNumber(constants.kShootingWheelSpeedKey, 0)
        hoodAngle = SmartDashboard.getNumber(constants.kShootingHoodAngleKey, 0)
        turretPosition = SmartDashboard.getNumber(constants.kShootingTurretAngleKey, 0)
        self.shoot.setWheelSpeed(wheelSpeed)
        self.shoot.setHoodAngle(Rotation2d.fromDegrees(hoodAngle))
        self.shoot.rotateTurret(
            Rotation2d.fromDegrees(turretPosition)
            + constants.kOffsetAngleRange * self.offset.sideToSide()
        )

    def normalMode(self) -> None:
        deltaPos = Pose2d(
            *[
                constants.kRobotUpdateRate * i * constants.kPredictiveAimGain
                for i in SmartDashboard.getNumberArray(
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
        ):
            distance = (
                SmartDashboard.getNumber(
                    constants.kTargetDistanceRelativeToRobotKeys.valueKey, 0.0
                )
                + constants.kOffsetDistanceRange * self.offset.forwardsBackwards()
            )

            targetSpeed = constants.kShootingMappingFunction(distance)
            self.shoot.setWheelSpeed(targetSpeed)

            targetAngle = Rotation2d.fromDegrees(
                constants.kHoodMappingFunction(distance)
            )
            self.shoot.setHoodAngle(targetAngle)

        if SmartDashboard.getBoolean(
            constants.kTargetAngleRelativeToRobotKeys.validKey, False
        ):
            angle = (
                SmartDashboard.getNumber(
                    constants.kTargetAngleRelativeToRobotKeys.valueKey, 0.0
                )
                + constants.kOffsetAngleRange.degrees() * self.offset.sideToSide()
            )

            self.shoot.trackTurret(
                angle + deltaRotation.degrees()
            )  # always track the turret
        else:
            self.shoot.rotateTurret(
                optimizeAngle(
                    constants.kTurretForwardAngle,
                    movingRotation
                    + constants.kOffsetAngleRange * self.offset.sideToSide(),
                )
                + Rotation2d.fromDegrees(180)  # shooter 0 is robot 180
            )
