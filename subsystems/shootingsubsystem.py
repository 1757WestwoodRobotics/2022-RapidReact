from commands2 import SubsystemBase
from wpilib import SmartDashboard

from wpimath.geometry import Rotation2d
from util.helpfulIO import Falcon, limitSwitch

import constants


class ShootingSubsystem(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        # actuators
        self.turretMotor = Falcon(
            constants.kTurretMotorName,
            constants.kTurretMotorId,
            constants.kSimTurretMotorPort,
            constants.kTurretPGain,
            constants.kTurretIGain,
            constants.kTurretDGain,
            constants.kTurretPIDSlot,
            constants.kTurretMotorInverted,
        )
        self.shootingMotor = Falcon(
            constants.kShootingMotorName,
            constants.kShootingMotorId,
            constants.kSimShootingMotorPort,
            constants.kShootingPGain,
            constants.kShootingIGain,
            constants.kShootingDGain,
            constants.kShootingPIDSlot,
            constants.kShootingMotorInverted,
        )
        self.hoodMotor = Falcon(
            constants.kHoodMotorName,
            constants.kHoodMotorId,
            constants.kSimHoodMotorPort,
            constants.kHoodPGain,
            constants.kHoodIGain,
            constants.kHoodDGain,
            constants.kHoodPIDSlot,
        )
        self.turretMaximumSwitch = limitSwitch(
            self.turretMotor, False, constants.kSimTurretMaximumLimitSwitchPort
        )
        self.turretMinimumSwitch = limitSwitch(
            self.turretMotor, True, constants.kSimTurretMinimumLimitSwitchPort
        )

        self.hoodMinimumSwitch = limitSwitch(
            self.hoodMotor, False, constants.kSimHoodMinimumSwitchPort
        )
        self.hoodMaximumSwitch = limitSwitch(
            self.hoodMotor, True, constants.kSimHoodMaximumSwitchPort
        )

    def periodic(self) -> None:
        if not SmartDashboard.getBoolean(constants.kShootingManualModeKey, False):
            SmartDashboard.putNumber(
                constants.kShootingWheelSpeedKey, self.getWheelSpeed()
            )
            SmartDashboard.putNumber(
                constants.kShootingHoodAngleKey, self.getHoodAngle().degrees()
            )
            SmartDashboard.putNumber(
                constants.kShootingTurretAngleKey, self.getTurretRotation().degrees()
            )

    def getWheelSpeed(self) -> int:
        """returns wheel speed in RPM"""
        return self.shootingMotor.getSpeed()

    def setWheelSpeed(self, speed: int) -> None:
        self.shootingMotor.setSpeed(speed)

    def setHoodAngle(self, angle: Rotation2d) -> None:
        """angle to fire the ball with
        absolute with 0 being straight and 90 degrees being direct to the sky"""
        encoderPulses = (
            angle.radians()
            * constants.kTalonEncoderPulsesPerRadian
            / constants.kHoodGearRatio
        )
        self.hoodMotor.setPosition(encoderPulses)

    def getHoodAngle(self) -> Rotation2d:
        return Rotation2d(
            self.hoodMotor.getPosition()
            / constants.kTalonEncoderPulsesPerRadian
            * constants.kHoodGearRatio
        )

    def rotateTurret(self, angle: Rotation2d):
        encoderPulses = (
            angle.radians()
            * constants.kTalonEncoderPulsesPerRadian
            / constants.kTurretGearRatio
        )
        self.turretMotor.setPosition(encoderPulses)

    def getTurretRotation(self) -> Rotation2d:
        angle = Rotation2d(
            (self.turretMotor.getPosition() / constants.kTalonEncoderPulsesPerRadian)
            * constants.kTurretGearRatio
        )
        return angle

    def trackTurret(self, relativeAngle: float):
        """relativeAngle: radians"""
        rotation = self.getTurretRotation() + Rotation2d(relativeAngle)
        self.rotateTurret(rotation)
