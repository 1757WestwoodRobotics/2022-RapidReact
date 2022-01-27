from commands2 import SubsystemBase

from wpimath.geometry import Rotation2d
from util.helpfulIO import Falcon, limitSwitch

import constants


class ShootingSubsystem(SubsystemBase):
    def __init__(self, name: str) -> None:
        SubsystemBase.__init__(self)
        self.name = name

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
        )
        self.shootingMotor = Falcon(
            constants.kShootingMotorName,
            constants.kShootingMotorId,
            constants.kSimShootingMotorPort,
            constants.kShootingPIDSlot,
            constants.kShootingPGain,
            constants.kShootingIGain,
            constants.kShootingDGain,
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

    def setWheelSpeed(self, speed: float) -> None:
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
        encoderPulses = angle.radians() * constants.kSwerveEncoderPulsesPerRadian
        self.hoodMotor.setPosition(encoderPulses)

    def rotateTurret(self, angle: Rotation2d):
        print(f"Turret rotated to {angle}")
        encoderPulses = angle.radians() * constants.kSwerveEncoderPulsesPerRadian
        self.turretMotor.setPosition(encoderPulses)

    def getTurretRotation(self) -> Rotation2d:
        return Rotation2d(
            self.turretMotor.getPosition() / constants.kSwerveEncoderPulsesPerRadian
        )

    def update(self, distance: float, relativeAngle: float):
        """distance: meters
        relativeAngle: degrees"""
        rotation = self.getTurretRotation() + Rotation2d.fromDegrees(relativeAngle)
        self.rotateTurret(rotation)
        speed = distance * 0.2  # TODO: determine this function from testing
        self.setWheelSpeed(speed)
