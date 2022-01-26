from commands2 import CommandBase

from subsystems.intakesubsystem import IntakeSubsystem


class ToggleIntakeMotor(CommandBase):
    def __init__(self, intake: IntakeSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.intake = intake
        self.addRequirements(intake)

    def initialize(self) -> None:
        print(f"Command: {self.getName()}")
        self.intake.intakeActive = not self.intake.intakeActive

    def execute(self) -> None:
        print("bruh")
        if self.intake.intakeActive:
            self.intake.runIntake()
        else:
            self.intake.stopIntake()
