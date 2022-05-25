from commands2 import SubsystemBase
from wpilib import SmartDashboard, RobotBase
from wpimath.geometry import Rotation2d
from ctre import ControlMode, NeutralMode
from util.angleoptimize import optimizeAngle
from util.convenientmath import map_range
from util.ctrecheck import ctreCheckError
from util.simfalcon import createMotor
import constants


class ShooterSubsystem(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)
        # actuators
        # TURRET
        self.turretMotor = createMotor(constants.kTurretMotorId)
        print(f"Initializing Falcon: {constants.kTurretMotorName}")
        if not ctreCheckError(
            "configFactoryDefault",
            self.turretMotor.configFactoryDefault(constants.kConfigurationTimeoutLimit),
        ):
            return
        if not ctreCheckError(
            "config_kP",
            self.turretMotor.config_kP(
                constants.kTurretMotorPIDSlot,
                constants.kTurretMotorPGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_kI",
            self.turretMotor.config_kI(
                constants.kTurretMotorPIDSlot,
                constants.kTurretMotorIGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_kD",
            self.turretMotor.config_kD(
                constants.kTurretMotorPIDSlot,
                constants.kTurretMotorDGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_Invert",
            self.turretMotor.setInverted(constants.kTurretMotorInverted),
        ):
            return
        self.turretMotor.setNeutralMode(NeutralMode.Brake)
        # SHOOTER
        self.shootingMotor = createMotor(constants.kShootingMotorId)
        print(f"Initializing Falcon: {constants.kShootingMotorName}")
        if not ctreCheckError(
            "configFactoryDefault",
            self.shootingMotor.configFactoryDefault(
                constants.kConfigurationTimeoutLimit
            ),
        ):
            return
        if not ctreCheckError(
            "config_kP",
            self.shootingMotor.config_kP(
                constants.kShootingMotorPIDSlot,
                constants.kShootingMotorPGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_kI",
            self.shootingMotor.config_kI(
                constants.kShootingMotorPIDSlot,
                constants.kShootingMotorIGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_kD",
            self.shootingMotor.config_kD(
                constants.kShootingMotorPIDSlot,
                constants.kShootingMotorDGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_Invert",
            self.shootingMotor.setInverted(constants.kShootingMotorInverted),
        ):
            return
        # HOOD
        self.hoodMotor = createMotor(constants.kHoodMotorId)
        print(f"Initializing Falcon: {constants.kHoodMotorName}")
        if not ctreCheckError(
            "configFactoryDefault",
            self.hoodMotor.configFactoryDefault(constants.kConfigurationTimeoutLimit),
        ):
            return
        if not ctreCheckError(
            "config_kP",
            self.hoodMotor.config_kP(
                constants.kHoodMotorPIDSlot,
                constants.kHoodMotorPGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_kI",
            self.hoodMotor.config_kI(
                constants.kHoodMotorPIDSlot,
                constants.kHoodMotorIGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_kD",
            self.hoodMotor.config_kD(
                constants.kHoodMotorPIDSlot,
                constants.kHoodMotorDGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_Invert",
            self.hoodMotor.setInverted(constants.kHoodMotorInverted),
        ):
            return

        self.initializationMinimum = -1
        self.initializationMaximum = 1

        self.targetTurretAngle = Rotation2d()
        self.targetHoodAngle = Rotation2d()
        self.targetWheelSpeed = 0

        SmartDashboard.putNumber(constants.kWheelSpeedTweakKey, 0)

    def setAsStartingPosition(self) -> None:
        self.hoodMotor.setSelectedSensorPosition(constants.kHoodStartingAngle)
        if (
            RobotBase.isReal()
        ):  # only possible to calibrate the real robot, sim is perfect by default
            turretRealPosition = map_range(
                self.turretMotor.getSelectedSensorPosition(),
                self.initializationMinimum,
                self.initializationMaximum,
                constants.kTurretMinimumAngle.radians()
                * constants.kTalonEncoderPulsesPerRadian,
                constants.kTurretMaximumAngle.radians()
                * constants.kTalonEncoderPulsesPerRadian,
            )
            self.turretMotor.setSelectedSensorPosition(turretRealPosition)
            if not ctreCheckError(
                "brake_set", self.turretMotor.setNeutralMode(NeutralMode.Brake)
            ):
                return

    def periodic(self) -> None:
        self.initializationMinimum = min(
            self.initializationMinimum, self.turretMotor.getSelectedSensorPosition()
        )
        self.initializationMaximum = max(
            self.initializationMaximum, self.turretMotor.getSelectedSensorPosition()
        )
        SmartDashboard.putNumber(
            "mappedVal",
            map_range(
                self.turretMotor.getSelectedSensorPosition(),
                self.initializationMinimum,
                self.initializationMaximum,
                constants.kTurretMinimumAngle.radians()
                * constants.kTalonEncoderPulsesPerRadian,
                constants.kTurretMaximumAngle.radians()
                * constants.kTalonEncoderPulsesPerRadian,
            ),
        )
        if not SmartDashboard.getBoolean(constants.kShootingManualModeKey, True):
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
        return (
            self.shootingMotor.getSelectedSensorVelocity()
            / constants.kTalonVelocityPerRPM
        )

    def setWheelSpeed(self, speed: int) -> None:
        self.targetWheelSpeed = speed
        self.shootingMotor.set(
            ControlMode.Velocity, speed * constants.kTalonVelocityPerRPM
        )

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
        self.hoodMotor.set(ControlMode.Position, encoderPulses)

    def getHoodAngle(self) -> Rotation2d:
        return Rotation2d(
            self.hoodMotor.getSelectedSensorPosition()
            / constants.kTalonEncoderPulsesPerRadian
            * constants.kHoodGearRatio
        )

    def rotateTurret(self, angle: Rotation2d) -> None:
        if (
            angle.radians()
            > constants.kTurretMaximumAngle.radians()
            - constants.kTurretSoftLimitBuffer.radians()
            or angle.radians()
            < constants.kTurretMinimumAngle.radians()
            + constants.kTurretSoftLimitBuffer.radians()
        ):
            return

        self.targetTurretAngle = angle
        encoderPulses = (
            angle.radians()
            * constants.kTalonEncoderPulsesPerRadian
            / constants.kTurretGearRatio
        )
        self.turretMotor.set(ControlMode.Position, encoderPulses)

    def getTurretRotation(self) -> Rotation2d:
        angle = Rotation2d(
            (
                self.turretMotor.getSelectedSensorPosition()
                / constants.kTalonEncoderPulsesPerRadian
            )
            * constants.kTurretGearRatio
        )
        return angle

    def trackTurret(self, relativeAngle: float):
        """relativeAngle: radians"""
        rotation = self.getTurretRotation() + Rotation2d(relativeAngle)
        self.rotateTurret(
            optimizeAngle(constants.kTurretRelativeForwardAngle, rotation)
        )
