from commands2 import SubsystemBase

from wpimath.geometry import Rotation2d
from util.helpfulIO import Falcon, limitSwitch

import constants


class ShootingSubsystem(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        # actuators
        self.stagingMotor = Falcon(
            constants.kStagingMotorName,
            constants.kStagingMotorId,
            constants.kSimStagingMotorPort,
        )

        self.turretMotor = Falcon(
            constants.kTurretMotorName,
            constants.kTurretMotorId,
            constants.kSimTurretMotorPort,
            constants.kTurretPGain,
            constants.kTurretIGain,
            constants.kTurretDGain,
            constants.kTurretPIDSlot,
        )
        self.shootingMotor = Falcon(
            constants.kShootingMotorName,
            constants.kShootingMotorId,
            constants.kSimShootingMotorPort,
            constants.kShootingPGain,
            constants.kShootingIGain,
            constants.kShootingDGain,
            constants.kShootingPIDSlot,
        )
        self.hoodMotor = Falcon(
            constants.kHoodMotorName,
            constants.kHoodMotorId,
            constants.kSimHoodMotorPort,
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

    def setWheelSpeed(self, speed: int) -> None:
        print(f"Speed set to {speed}")
        self.shootingMotor.setSpeed(speed)

    def launchCargo(self) -> None:
        print("launching mechanism activated!")
        self.stagingMotor.setSpeed(1000)
        # something something balls fire!

    def stopLaunchingCargo(self) -> None:
        self.stagingMotor.setSpeed(0)

    def setHoodAngle(self, angle: Rotation2d) -> None:
        """angle to fire the ball with
        absolute with 0 being straight and 90 degrees being direct to the sky"""
        print(f"hood angle set to {angle}")
        encoderPulses = angle.radians() * constants.kTalonEncoderPulsesPerRadian
        self.hoodMotor.setPosition(encoderPulses)

    def rotateTurret(self, angle: Rotation2d):
        print(f"Turret rotated to {angle.degrees()}")
        encoderPulses = (
            angle.radians() * constants.kTalonEncoderPulsesPerRadian
        ) / constants.kTurretGearRatio
        self.turretMotor.setPosition(encoderPulses)

    def getTurretRotation(self) -> Rotation2d:
        angle = Rotation2d(
            (self.turretMotor.getPosition() / constants.kTalonEncoderPulsesPerRadian)
            * constants.kTurretGearRatio
        )
        print(f"Current Turret Angle: {angle.degrees()}")
        return angle

    def trackTurret(self, relativeAngle: float):
        """relativeAngle: radians"""
        print(f"CURRENT TIME: {Timer.getMatchTime()}")
        rotation = self.getTurretRotation() + Rotation2d(relativeAngle)
        self.rotateTurret(rotation)
