from typing import Callable
from ctre import WPI_TalonFX, ControlMode
from wpilib import RobotBase, PWMVictorSPX, DigitalInput
from wpimath.controller import PIDController
from wpimath.system.plant import DCMotor
from util.ctrecheck import ctreCheckError
from util.convenientmath import clamp

import constants

LimitSwitch = Callable[[], bool]


class Falcon:  # represents either a simulated motor or a real Falcon 500
    def __init__(
        self,
        name: str,
        realId: int,
        simId: int,
        PGain: int = 1,
        IGain: int = 0,
        DGain: int = 0,
        PIDSlot: int = 0,
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
                    PIDSlot,
                    PGain,
                    constants.kConfigurationTimeoutLimit,
                ),
            ):
                return
            if not ctreCheckError(
                "config_kI",
                self.motor.config_kP(
                    PIDSlot,
                    IGain,
                    constants.kConfigurationTimeoutLimit,
                ),
            ):
                return
            if not ctreCheckError(
                "config.kD",
                self.motor.config_kD(
                    PIDSlot,
                    DGain,
                    constants.kConfigurationTimeoutLimit,
                ),
            ):
                return
            print("Falcon Initialization Complete")
        else:
            self.pidController = PIDController(PGain, IGain, DGain)
            self.simEncoder = 0

    def _setSimMotor(self, amount: float) -> None:
        """moves just the simulated motor [-1,1]"""
        if not RobotBase.isReal():
            clampedAmount = clamp(amount, -1, -1)
            self.motor.set(clampedAmount)
            self.simEncoder += (
                clampedAmount
                * DCMotor.falcon500().freeSpeed
                * constants.kTalonEncoderPulsesPerRevolution
            )  # amount of free speed, free speed is in RPM, convert from revolutions to encodes ticks expected
        else:
            raise IndexError("Cannot Move A Simulated Motor On A Real Robot")

    def setPosition(self, pos: int) -> None:
        """set the position of the motor in encoder ticks"""
        if RobotBase.isReal():
            self.motor.set(ControlMode.Position, pos)
        else:
            positionChange = pos - self.simEncoder
            change = self.pidController.calculate(positionChange)
            self._setSimMotor(change)

    def getPosition(self) -> int:
        """returns the position in encoder ticks"""
        if RobotBase.isReal():
            return int(self.motor.getSelectedSensorPosition())
        else:
            return int(self.simEncoder)

    def setSpeed(self, rpm: int):
        """set the speed of the motor in Revolutions Per Minute"""
        if not RobotBase.isReal():
            amount = self.pidController.calculate(rpm)
            self._setSimMotor(
                amount / DCMotor.falcon500().freeSpeed
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
                return falcon.motor.isFwdLimitSwitchClosed()
            else:
                return falcon.motor.isRevLimitSwitchClosed()
        else:
            return DigitalInput(simId).get()

    return value
