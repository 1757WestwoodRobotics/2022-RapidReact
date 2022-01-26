from commands2 import SubsystemBase
from wpilib import PWMVictorSPX, RobotBase
import constants


class IntakeSubsystem(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)
        self.intakeActive = False  # default to intake retracted

        if RobotBase.isReal():
            pass
        else:
            self.intakeMotor = PWMVictorSPX(constants.kSimIntakeMotorPort)

    def toggleIntake(self) -> None:
        self.intakeActive = not self.intakeActive

    def isIntakeDeployed(self) -> bool:
        return self.intakeActive

    def deployIntake(self) -> None:
        self.intakeActive = True

    def retractIntake(self) -> None:
        self.intakeActive = False

    def runIntake(self) -> None:
        self.intakeMotor.set(1.0)

    def stopIntake(self) -> None:
        self.intakeMotor.set(0)
