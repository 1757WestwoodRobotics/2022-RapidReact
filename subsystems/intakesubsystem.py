from commands2 import SubsystemBase


class IntakeSubsystem(SubsystemBase):
    def __init__(self, name: str) -> None:
        self.name = name
        self.intakeActive = False  # default to intake retracted

    def toggleIntake(self) -> None:
        self.intakeActive = not self.intakeActive

    def isIntakeDeployed(self) -> bool:
        return self.intakeActive

    def deployIntake(self) -> None:
        self.intakeActive = True

    def retractIntake(self) -> None:
        self.intakeActive = False
