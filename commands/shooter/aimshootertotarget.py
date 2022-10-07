from commands2 import CommandBase
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d, Pose2d, Transform2d

from subsystems.shootersubsystem import ShooterSubsystem
import constants
from util.angleoptimize import optimizeAngle
from util.convenientmath import rotationFromTranslation


class AimShooterToTarget(CommandBase):
    def __init__(self, shooter: ShooterSubsystem) -> None:
        SmartDashboard.putBoolean(constants.kShootingManualModeKey, False)
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.shooter = shooter
        self.addRequirements([self.shooter])

    def execute(self) -> None:
        if SmartDashboard.getBoolean(
            constants.kReadyToFireKey, False
        ):  # only start tracking when ready to fire
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
                optimizeAngle(constants.kTurretRelativeForwardAngle, staticRotation)
                + constants.kTurretOffsetFromRobotAngle
            )

            distance = staticDifference.translation().norm()
            hoodAngle = constants.kHoodMappingFunction(distance)
            wheelSpeed = constants.kShootingMappingFunction(
                distance
            ) + SmartDashboard.getNumber(constants.kWheelSpeedTweakKey, 0)
            self.shooter.setHoodAngle(Rotation2d.fromDegrees(hoodAngle))
            self.shooter.setWheelSpeed(wheelSpeed)
