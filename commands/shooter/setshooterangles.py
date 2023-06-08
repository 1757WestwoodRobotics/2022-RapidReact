from commands2 import CommandBase
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d

from subsystems.shootersubsystem import ShooterSubsystem

import constants


class SetShooterAngles(CommandBase):
    def __init__(self, shooter: ShooterSubsystem, hoodAngle, turretAngle):
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.shooter = shooter
        self.hoodAngle = hoodAngle
        self.turretAngle = turretAngle

        self.addRequirements([self.shooter])

    def execute(self) -> None:
        SmartDashboard.putNumber(constants.kShootingHoodAngleKey, self.hoodAngle)
        SmartDashboard.putNumber(constants.kShootingTurretAngleKey, self.turretAngle)
        self.shooter.setHoodAngle(Rotation2d.fromDegrees(self.hoodAngle))
        self.shooter.rotateTurret(Rotation2d.fromDegrees(self.turretAngle))
