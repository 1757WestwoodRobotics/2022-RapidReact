from enum import Enum, auto
from commands2 import SubsystemBase
from wpilib import PneumaticsModuleType, Solenoid, SmartDashboard
import constants
from util.helpfulIO import Falcon


class IntakeSubsystem(SubsystemBase):
    class Mode(Enum):
        Deployed = auto()
        Retracted = auto()
        Reversed = auto()

        def asString(self) -> str:
            mapping = {
                self.Deployed: "Deployed",
                self.Retracted: "Retracted",
                self.Reversed: "Reversed",
            }
            return mapping[self]

    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)

        self.intakeSolenoid = Solenoid(
            PneumaticsModuleType.REVPH, constants.kIntakeSolenoidChannelId
        )

        self.intakeMotor = Falcon(
            constants.kIntakeMotorName,
            constants.kIntakeMotorId,
            constants.kSimIntakeMotorPort,
            inverted=constants.kIntakeMotorInverted,
        )
        self.state = self.Mode.Retracted

    def periodic(self) -> None:
        SmartDashboard.putString(constants.kIntakeSystemStateKey, self.state.asString())

    def reverseIntake(self) -> None:
        self.state = self.Mode.Reversed

    def deployIntake(self) -> None:
        self.state = self.Mode.Deployed

    def retractIntake(self) -> None:
        self.state = self.Mode.Retracted

    def defaultIntake(self) -> None:
        if self.state == self.Mode.Deployed:
            self.intakeSolenoid.set(True)
            self.intakeMotor.setSpeed(constants.kIntakeSpeed)
        elif self.state == self.Mode.Reversed:
            self.intakeSolenoid.set(True)
            self.intakeMotor.setSpeed(-constants.kIntakeSpeed)
        elif self.state == self.Mode.Retracted:
            self.intakeSolenoid.set(False)
            self.intakeMotor.setSpeed(0)
