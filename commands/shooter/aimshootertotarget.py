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
        distance = SmartDashboard.getNumber(
            constants.kTargetDistanceRelativeToRobotKeys.valueKey, 0
        )
        hoodAngle = constants.kHoodMappingFunction(distance)
        wheelSpeed = constants.kShootingMappingFunction(distance)
        self.shooter.setHoodAngle(Rotation2d.fromDegrees(hoodAngle))
        self.shooter.setWheelSpeed(wheelSpeed)

        if SmartDashboard.getBoolean(
            constants.kTargetAngleRelativeToRobotKeys.validKey, False
        ):  # if we have an angle, use the relative angle to adjust the turret for precision
            angle = SmartDashboard.getNumber(
                constants.kTargetAngleRelativeToRobotKeys.valueKey, 0.0
            )

            self.shooter.trackTurret(angle)  # always track the turret
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
                optimizeAngle(constants.kTurretRelativeForwardAngle, staticRotation)
                + constants.kTurretOffsetFromRobotAngle
            )
