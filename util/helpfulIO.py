from typing import Callable
from ctre import WPI_TalonFX, ControlMode
from wpilib import RobotBase, PWMVictorSPX, DigitalInput, SmartDashboard
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
        PGain: float = 1,
        IGain: float = 0,
        DGain: float = 0,
        PIDSlot: int = 0,
        inverted: bool = False,
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
                self.motor.config_kI(
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
            if not ctreCheckError("invert", self.motor.setInverted(inverted)):
                return
            print("Falcon Initialization Complete")
        else:
            self.pidController = PIDController(PGain, IGain, DGain)
            self.simEncoder = 0

            SmartDashboard.putNumber(
                f"{constants.kMotorBaseKey}/{self.name}/encoder/value", 0
            )
            SmartDashboard.putNumber(
                f"{constants.kMotorBaseKey}/{self.name}/output/value", 0
            )

            SmartDashboard.putBoolean(
                f"{constants.kMotorBaseKey}/{self.name}/encoder/overwritten", False
            )
            SmartDashboard.putBoolean(
                f"{constants.kMotorBaseKey}/{self.name}/output/overwritten", False
            )

    def _setSimMotor(self, amount: float) -> None:
        """moves just the simulated motor [-1,1]"""
        if not RobotBase.isReal():

            if SmartDashboard.getBoolean(
                f"{constants.kMotorBaseKey}/{self.name}/output/overwritten", False
            ):
                clampedAmount = clamp(
                    SmartDashboard.getNumber(
                        f"{constants.kMotorBaseKey}/{self.name}/output/value", 0
                    ),
                    -1,
                    1,
                )
            clampedAmount = clamp(amount, -1, 1)
            self.motor.set(clampedAmount)

            if SmartDashboard.getBoolean(
                f"{constants.kMotorBaseKey}/{self.name}/encoder/overwritten", False
            ):
                self.simEncoder = SmartDashboard.getNumber(
                    f"{constants.kMotorBaseKey}/{self.name}/encoder/value",
                    0,
                )
            else:
                self.simEncoder += (
                    clampedAmount
                    # * 2 * tau # radians per second
                    * DCMotor.falcon500().freeSpeed  # radians per second
                    * constants.kTalonEncoderPulsesPerRadian  # encoder ticks per radian
                    * 1
                    / 50  # 50 updates per second
                )  # amount of free speed, free speed is in RPS, convert from revolutions to encodes ticks expected

            SmartDashboard.putNumber(
                f"{constants.kMotorBaseKey}/{self.name}/encoder/value", self.simEncoder
            )
            SmartDashboard.putNumber(
                f"{constants.kMotorBaseKey}/{self.name}/output/value", self.motor.get()
            )
        else:
            raise IndexError("Cannot Move A Simulated Motor On A Real Robot")

    def setCurrentEncoderPulseCount(self, count: int) -> None:
        if RobotBase.isReal():
            self.motor.setSelectedSensorPosition(count)
        else:
            self.simEncoder = count

    def setPosition(self, pos: int) -> None:
        """set the position of the motor in encoder ticks"""
        if RobotBase.isReal():
            self.motor.set(ControlMode.Position, pos)
        else:
            change = self.pidController.calculate(self.simEncoder, pos)
            self._setSimMotor(
                change / constants.kTalonEncoderPulsesPerRevolution
            )  # convert the change in encoder ticks into change into motor %

    def getPosition(self) -> int:
        """returns the position in encoder ticks"""
        if RobotBase.isReal():
            return int(self.motor.getSelectedSensorPosition())
        else:
            return int(self.simEncoder)

    def getSpeed(self) -> int:
        """returns rpm of the motor"""
        if RobotBase.isReal():
            return (
                self.motor.getSelectedSensorVelocity()
                / constants.k100MillisecondsPerSecond
                / constants.kTalonEncoderPulsesPerRevolution
            )
        return (
            self.motor.get()
            * DCMotor.falcon500().freeSpeed
            / constants.kRadiansPerRevolution
            * constants.kSecondsPerMinute
        )

    def setSpeed(self, rpm: int):
        """set the speed of the motor in Revolutions Per Minute"""
        if not RobotBase.isReal():
            amount = self.pidController.calculate(self.getSpeed(), rpm)
            self._setSimMotor(
                (amount + self.getSpeed())
                / (
                    DCMotor.falcon500().freeSpeed
                    / constants.kRadiansPerRevolution
                    * constants.kSecondsPerMinute
                )
            )  # use percent based on "ideal" motor
        else:
            driveEncoderPulsesPerSecond = (
                rpm
                * constants.kWheelEncoderPulsesPerRevolution
                / constants.kSecondsPerMinute
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
