from commands2 import CommandBase

from subsystems.intakesubsystem import IntakeSubsystem


class ToggleIntakeDeploy(CommandBase):
    def __init__(self, intake: IntakeSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.intake = intake
        self.addRequirements(intake)

    def initialize(self) -> None:
        print(f"Command: {self.getName()}")
        self.intake.toggleIntakeDeploy()

    def execute(self) -> None:
        if self.intake.isIntakeDeployed():
            self.intake.deployIntake()
        else:
            self.intake.retractIntake()
