from commands2 import CommandBase
from subsystems.intakesubsystem import IntakeSubsystem


class DefaultIntake(CommandBase):
    def __init__(self, intake: IntakeSubsystem):
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.intake = intake
        self.addRequirements([self.intake])

    def execute(self) -> None:
        self.intake.defaultIntake()
