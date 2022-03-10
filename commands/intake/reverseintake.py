from commands2 import CommandBase

from subsystems.intakesubsystem import IntakeSubsystem


class ReverseIntake(CommandBase):
    def __init__(self, intake: IntakeSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.intake = intake
        self.addRequirements([self.intake])

    def initialize(self) -> None:
        print(f"Command: {self.getName()}")
        self.intake.reverseIntake()

    # pylint: disable-next=no-self-use
    def end(self, _interrupted: bool) -> None:
        print("... DONE")

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return True
