from commands2 import CommandBase
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d
from operatorinterface import AnalogInput

from subsystems.shootersubsystem import ShooterSubsystem

import constants
from util.convenientmath import map_range


class SetShooterAngles(CommandBase):
    def __init__(
        self,
        shooter: ShooterSubsystem,
        hoodControl: AnalogInput,
        turretControl: AnalogInput,
    ):
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.shooter = shooter

        self.hoodControl = hoodControl
        self.turretControl = turretControl

        self.addRequirements([self.shooter])

    def execute(self) -> None:
        hoodDemand = self.hoodControl()
        turretDemand = self.turretControl()

        hoodAngle = map_range(
            hoodDemand,
            constants.kAxisMin,
            constants.kAxisMax,
            constants.kHoodMinAngle,
            constants.kHoodMaxAngle,
        )
        turretAngle = (
            map_range(
                turretDemand,
                constants.kAxisMin,
                constants.kAxisMax,
                constants.kShooterRotationSpeed * -1,
                constants.kShooterRotationSpeed,
            )
            + self.shooter.getTurretRotation().degrees()
        )
        wheelSpeed = SmartDashboard.getNumber(constants.kShootingWheelSpeedKey, 500)

        self.shooter.setHoodAngle(Rotation2d.fromDegrees(hoodAngle))
        self.shooter.rotateTurret(Rotation2d.fromDegrees(turretAngle))
        self.shooter.setWheelSpeed(wheelSpeed)
