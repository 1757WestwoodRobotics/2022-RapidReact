from commands2 import CommandBase
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d
from operatorinterface import Control2D

from subsystems.shootersubsystem import ShooterSubsystem

import constants


class AimShooterManually(CommandBase):
    def __init__(self, shooter: ShooterSubsystem, shooterOffset: Control2D) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.shooter = shooter
        self.distanceAndRotationOffset = shooterOffset
        self.addRequirements([self.shooter])
        SmartDashboard.putNumber(constants.kShootingHoodAngleKey, 8)

    def execute(self) -> None:
        # wheelSpeed = SmartDashboard.getNumber(constants.kShootingWheelSpeedKey, 550)
        # hoodAngle = SmartDashboard.getNumber(constants.kShootingHoodAngleKey, 8)
        wheelSpeed = 550
        hoodAngle = 8
        turretPosition = (
            SmartDashboard.getNumber(constants.kShootingTurretAngleKey, 0)
            + 90 * self.distanceAndRotationOffset.sideToSide()
        )
        self.shooter.setWheelSpeed(wheelSpeed)
        self.shooter.setHoodAngle(Rotation2d.fromDegrees(hoodAngle))
        self.shooter.rotateTurret(Rotation2d.fromDegrees(turretPosition))
