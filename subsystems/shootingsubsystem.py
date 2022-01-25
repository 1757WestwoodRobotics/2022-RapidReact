from typing import Callable
from commands2 import SubsystemBase

from wpimath.geometry import Rotation2d
from wpimath.system.plant import DCMotor
from ctre import WPI_TalonFX, ControlMode
from wpilib import PWMVictorSPX, RobotBase, DigitalInput
from subsystems.drivesubsystem import ctreCheckError

import constants

LimitSwitch = Callable[[], bool]


class Falcon:  # represents either a simulated motor or a real Falcon 500
    def __init__(
        self, name: str, realId: int, simId: int
    ) -> None:  # depending on if its in simulation or the real motor, different motors will be used
        self.name = name
        self.motor = WPI_TalonFX(realId) if RobotBase.isReal() else PWMVictorSPX(simId)

        if RobotBase.isReal():
            print(f"Initializing Falcon: {self.name}")
            if not ctreCheckError(
                "configFactoryDefault",
                self.motor.configFactoryDefault(constants.kConfigurationTimeoutLimit),
            ):
                return
            if not ctreCheckError(
                "config_kP",
                self.motor.config_kP(
                    constants.kShootingPIDSlot,
                    constants.kShootingPGain,
                    constants.kConfigurationTimeoutLimit,
                ),
            ):
                return
            if not ctreCheckError(
                "config_kI",
                self.motor.config_kP(
                    constants.kShootingPIDSlot,
                    constants.kShootingIGain,
                    constants.kConfigurationTimeoutLimit,
                ),
            ):
                return
            if not ctreCheckError(
                "config.kD",
                self.motor.config_kD(
                    constants.kShootingPIDSlot,
                    constants.kShootingDGain,
                    constants.kConfigurationTimeoutLimit,
                ),
            ):
                return
            print("Falcon Initialization Complete")

    def setSpeed(self, rpm: int):
        if RobotBase.isReal():
            self.motor.set(
                rpm / DCMotor.falcon500().freeSpeed
            )  # use percent based on "ideal" motor
        else:
            driveEncoderPulsesPerSecond = (
                rpm * constants.kWheelEncoderPulsesPerRevolution
            )
            self.motor.set(
                ControlMode.Velocity,
                driveEncoderPulsesPerSecond / constants.k100MillisecondsPerSecond,
            )


def limitSwitch(falcon: Falcon, isRealForwardSwitch: bool, simId: int) -> LimitSwitch:
    def value() -> bool:
        if RobotBase.isReal():
            if isRealForwardSwitch:
                return falcon.isFwdLimitSwitchClosed()
            else:
                return falcon.isRevLimitSwitchClosed()
        else:
            return DigitalInput(simId).get()

    return value


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
        self.setWheelSpeed(5000)  # TODO: better mapping
        # something something balls fire!

    def setHoodAngle(self, angle: Rotation2d) -> None:
        """angle to fire the ball with
        absolute with 0 being straight and 90 degrees being direct to the sky"""
        print(f"hood angle set to {angle}")
        self.hoodMotor.motor.set(0)  # TODO: actually move the hood to an angle

    def rotateTurret(self, angle: Rotation2d) -> None:
        print(f"Turret rotated to {angle}")
        self.hoodMotor.motor.set(0)  # TODO: actually rotate the turret
