from wpilib import (
    DoubleSolenoid,
    PneumaticsModuleType,
    Solenoid,
)
from ctre import ControlMode
from util.ctrecheck import ctreCheckError
from util.simfalcon import createMotor
import constants


class ClimberModule:
    def __init__(
        self,
        motorName,
        realMotorID,
        motorPGain,
        motorIGain,
        motorDGain,
        brakeID,
        actuatorID1,
        actuatorID2,
        inverted=False,
    ) -> None:
        self.brake = Solenoid(PneumaticsModuleType.REVPH, brakeID)
        self.piston = DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            actuatorID1,
            actuatorID2,
        )
        self.climberMotor = createMotor(realMotorID)
        print(f"Initializing Falcon: {motorName}")
        if not ctreCheckError(
            "configFactoryDefault",
            self.climberMotor.configFactoryDefault(
                constants.kConfigurationTimeoutLimit
            ),
        ):
            return
        if not ctreCheckError(
            "config_kP",
            self.climberMotor.config_kP(
                constants.kClimberMotorPIDSlot,
                motorPGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_kI",
            self.climberMotor.config_kI(
                constants.kClimberMotorPIDSlot,
                motorIGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_kD",
            self.climberMotor.config_kD(
                constants.kClimberMotorPIDSlot,
                motorDGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_Invert",
            self.climberMotor.setInverted(inverted),
        ):
            return
        self.climberMotor.configReverseSoftLimitThreshold(
            constants.kClimberMiddleRungHangPosition
        )
        self.climberMotor.configReverseSoftLimitEnable(True)
        self.extendPiston()

    def activateBrake(self) -> None:
        self.brake.set(False)

    def deactivateBrake(self) -> None:
        self.brake.set(True)

    def extendPiston(self) -> None:
        self.piston.set(DoubleSolenoid.Value.kForward)

    def retractPiston(self) -> None:
        self.piston.set(DoubleSolenoid.Value.kReverse)

    def setClimberFullExtension(self) -> None:
        self.climberMotor.set(
            ControlMode.Position, constants.kClimberMiddleRungCapturePosition
        )

    def setClimberMidExtension(self) -> None:
        self.climberMotor.set(
            ControlMode.Position, constants.kClimberMiddleRungHangPosition
        )
