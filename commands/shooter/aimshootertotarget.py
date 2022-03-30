from commands2 import CommandBase
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d
from operatorinterface import Control2D

from subsystems.shootersubsystem import ShooterSubsystem
import constants


class AimShooterToTarget(CommandBase):
    def __init__(self, shooter: ShooterSubsystem, shooterOffset: Control2D) -> None:
        SmartDashboard.putBoolean(constants.kShootingManualModeKey, False)
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.shooter = shooter
        self.distanceAndRotationOffset = shooterOffset
        self.addRequirements([self.shooter])

    def execute(self) -> None:
        self.shooter.setHoodAngle(Rotation2d.fromDegrees(8))
        self.shooter.setWheelSpeed(550)

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

            self.shooter.trackTurret(angle)  # always track the turret
        else:  # ...otherwise use odometry to estimate where the target SHOULD be
            self.shooter.rotateTurret(Rotation2d(0))
