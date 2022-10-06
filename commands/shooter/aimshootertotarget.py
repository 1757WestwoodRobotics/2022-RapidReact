from typing import Tuple
from commands2 import CommandBase
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d, Pose2d, Transform2d, Translation2d

from subsystems.shootersubsystem import ShooterSubsystem
import constants
from util.angleoptimize import optimizeAngle
from util.convenientmath import Interpolator, rotationFromTranslation, number


class AimShooterToTarget(CommandBase):
    def __init__(self, shooter: ShooterSubsystem) -> None:
        SmartDashboard.putBoolean(constants.kShootingManualModeKey, False)
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.shooter = shooter
        self.addRequirements([self.shooter])

        self.timeInterp = Interpolator(constants.kTimeOfFlightList)

    def calculateDistanceAndAngleOffsets(self) -> Tuple[number, Rotation2d]:
        distance = SmartDashboard.getNumber(
            constants.kTargetDistanceRelativeToRobotKeys.valueKey, 0
        )
        robotVel = Transform2d(
            *SmartDashboard.getNumberArray(constants.kDriveVelocityKeys, [0, 0, 0])
        )

        timeToHitTarget = self.timeInterp.interpolate(distance)
        deltaTranslation = Translation2d(
            robotVel.X() * -timeToHitTarget, robotVel.Y() * -timeToHitTarget
        )
        deltaDistance = deltaTranslation.distance(Translation2d(0, 0))

        currentPose = Pose2d(
            *SmartDashboard.getNumberArray(
                constants.kRobotPoseArrayKeys.valueKey, [0, 0, 0]
            )
        )

        staticDifference = Transform2d(currentPose, constants.kSimDefaultTargetLocation)
        oldAngle = rotationFromTranslation(staticDifference.translation())
        newAngle = rotationFromTranslation(
            staticDifference.translation() + deltaTranslation
        )
        deltaAngle = newAngle - oldAngle

        return (deltaDistance, deltaAngle)

    def execute(self) -> None:
        distanceChange, angleChange = self.calculateDistanceAndAngleOffsets()
        if SmartDashboard.getBoolean(
            constants.kReadyToFireKey, False
        ):  # only start tracking when ready to fire
            distance = SmartDashboard.getNumber(
                constants.kTargetDistanceRelativeToRobotKeys.valueKey, 0
            )
            hoodAngle = constants.kHoodMappingFunction(distance + distanceChange)
            wheelSpeed = constants.kShootingMappingFunction(
                distance + distanceChange
            ) + SmartDashboard.getNumber(constants.kWheelSpeedTweakKey, 0)
            self.shooter.setHoodAngle(Rotation2d.fromDegrees(hoodAngle))
            self.shooter.setWheelSpeed(wheelSpeed)

            if SmartDashboard.getBoolean(
                constants.kTargetAngleRelativeToRobotKeys.validKey, False
            ):  # if we have an angle, use the relative angle to adjust the turret for precision
                angle = SmartDashboard.getNumber(
                    constants.kTargetAngleRelativeToRobotKeys.valueKey, 0.0
                )

                self.shooter.trackTurret(
                    angle + angleChange.radians()
                )  # always track the turret
            else:  # ...otherwise use odometry to estimate where the target SHOULD be
                currentPose = Pose2d(
                    *SmartDashboard.getNumberArray(
                        constants.kRobotPoseArrayKeys.valueKey, [0, 0, 0]
                    )
                )

                staticDifference = Transform2d(
                    currentPose, constants.kSimDefaultTargetLocation
                )
                staticRotation = rotationFromTranslation(staticDifference.translation())

                self.shooter.rotateTurret(
                    optimizeAngle(
                        constants.kTurretRelativeForwardAngle,
                        staticRotation + angleChange,
                    )
                    + constants.kTurretOffsetFromRobotAngle
                )
