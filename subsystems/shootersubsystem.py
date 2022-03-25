from commands2 import SubsystemBase
from wpilib import SmartDashboard, RobotBase

from wpimath.geometry import Rotation2d
from util.angleoptimize import optimizeAngle
from util.convenientmath import map_range
from util.helpfulIO import Falcon

import constants


class ShooterSubsystem(SubsystemBase):
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
            constants.kHoodMotorInverted,
        )

        self.initializationMinimum = 0
        self.initializationMaximum = 0

        self.targetTurretAngle = Rotation2d()
        self.targetHoodAngle = Rotation2d()
        self.targetWheelSpeed = 0

    def setAsStartingPosition(self) -> None:
        self.hoodMotor.setCurrentEncoderPulseCount(constants.kHoodStartingAngle)
        if (
            RobotBase.isReal()
        ):  # only possible to calibrate the real robot, sim is perfect by default
            turretRealPosition = map_range(
                self.turretMotor.getPosition(),
                self.initializationMinimum,
                self.initializationMaximum,
                constants.kTurretMinimumAngle.radians()
                * constants.kTalonEncoderPulsesPerRadian,
                constants.kTurretMaximumAngle.radians()
                * constants.kTalonEncoderPulsesPerRadian,
            )
            self.turretMotor.setCurrentEncoderPulseCount(turretRealPosition)
            self.turretMotor.setBrakeMode()

    def periodic(self) -> None:
        self.initializationMinimum = min(
            self.initializationMinimum, self.turretMotor.getPosition()
        )
        self.initializationMaximum = max(
            self.initializationMaximum, self.turretMotor.getPosition()
        )
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
        SmartDashboard.putBoolean(
            constants.kShootingFlywheelOnTargetKey, self.wheelOnTarget()
        )
        SmartDashboard.putBoolean(
            constants.kShootingHoodOnTargetKey, self.hoodOnTarget()
        )
        SmartDashboard.putBoolean(
            constants.kShootingTurretOnTargetKey, self.turretOnTarget()
        )

    def wheelOnTarget(self) -> bool:
        return (
            abs(self.getWheelSpeed() - self.targetWheelSpeed)
            <= constants.kWheelSpeedTolerence
        )

    def hoodOnTarget(self) -> bool:
        return (
            abs((self.getHoodAngle() - self.targetHoodAngle).radians())
            <= constants.kHoodAngleTolerence.radians()
        )

    def turretOnTarget(self) -> bool:
        return (
            abs((self.getTurretRotation() - self.targetTurretAngle).radians())
            <= constants.kTurretAngleTolerence.radians()
        )

    def getWheelSpeed(self) -> int:
        """returns wheel speed in RPM"""
        return self.shootingMotor.getSpeed()

    def setWheelSpeed(self, speed: int) -> None:
        self.targetWheelSpeed = speed
        self.shootingMotor.setSpeed(speed)

    def setHoodAngle(self, angle: Rotation2d) -> None:
        """angle to fire the ball with
        absolute with 0 being straight and 90 degrees being direct to the sky"""
        self.targetHoodAngle = angle
        clampedAngle = min(
            max(
                angle.radians(),
                (constants.kHoodMinimum + constants.kHoodSoftLimitBuffer).radians(),
            ),
            (constants.kHoodMaximum - constants.kHoodSoftLimitBuffer).radians(),
        )
        encoderPulses = (
            clampedAngle
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

    def rotateTurret(self, angle: Rotation2d) -> None:
        if (
            angle.radians() > constants.kTurretMaximumAngle.radians()
            or angle.radians() < constants.kTurretMinimumAngle.radians()
        ):
            return

        self.targetTurretAngle = angle
        encoderPulses = (
            max(
                (
                    constants.kTurretMinimumAngle + constants.kTurretSoftLimitBuffer
                ).radians(),
                min(
                    angle.radians(),
                    constants.kTurretMaximumAngle.radians()
                    - constants.kTurretSoftLimitBuffer.radians(),
                ),
            )
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
        self.rotateTurret(
            optimizeAngle(constants.kTurretRelativeForwardAngle, rotation)
        )
