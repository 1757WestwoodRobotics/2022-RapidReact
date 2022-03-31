from enum import Enum, auto
from commands2 import SubsystemBase
from wpilib import PneumaticsModuleType, Solenoid
from ctre import ControlMode
from util.ctrecheck import ctreCheckError
from util.simfalcon import createMotor
import constants


class IntakeSubsystem(SubsystemBase):
    class Mode(Enum):
        Deployed = auto()
        Retracted = auto()
        Reversed = auto()

    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)

        self.intakeSolenoid = Solenoid(
            PneumaticsModuleType.REVPH, constants.kIntakeSolenoidChannelId
        )

        self.intakeMotor = createMotor(
            constants.kIntakeMotorId,
            constants.kCANivoreName,
        )
        print(f"Initializing Falcon: {constants.kIntakeMotorName}")
        if not ctreCheckError(
            "configFactoryDefault",
            self.intakeMotor.configFactoryDefault(constants.kConfigurationTimeoutLimit),
        ):
            return
        if not ctreCheckError(
            "config_kP",
            self.intakeMotor.config_kP(
                constants.kIntakeMotorPIDSlot,
                constants.kIntakeMotorPGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_kI",
            self.intakeMotor.config_kI(
                constants.kIntakeMotorPIDSlot,
                constants.kIntakeMotorIGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_kD",
            self.intakeMotor.config_kD(
                constants.kIntakeMotorPIDSlot,
                constants.kIntakeMotorDGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_Invert",
            self.intakeMotor.setInverted(constants.kIntakeMotorInverted),
        ):
            return
        print(f"{constants.kIntakeMotorName} Falcon Initialization Complete")
        self.state = self.Mode.Retracted

    def periodic(self) -> None:
        if self.state == self.Mode.Deployed:
            self.intakeSolenoid.set(True)
            self.intakeMotor.set(
                ControlMode.Velocity,
                constants.kIntakeSpeed * constants.kTalonVelocityPerRPM,
            )
        elif self.state == self.Mode.Reversed:
            self.intakeSolenoid.set(True)
            self.intakeMotor.set(
                ControlMode.Velocity,
                -constants.kIntakeSpeed * constants.kTalonVelocityPerRPM,
            )
        elif self.state == self.Mode.Retracted:
            self.intakeSolenoid.set(False)
            self.intakeMotor.set(
                ControlMode.Velocity, 0
            )  # drive to 0 instead of neutral to help make sure the rollers have stopped by the time the intake is in the retracted position

    def reverseIntake(self) -> None:
        self.state = self.Mode.Reversed

    def deployIntake(self) -> None:
        self.state = self.Mode.Deployed

    def retractIntake(self) -> None:
        self.state = self.Mode.Retracted
