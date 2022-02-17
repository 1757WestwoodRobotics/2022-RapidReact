from commands2 import CommandBase

from subsystems.intakesubsystem import IntakeSubsystem


class ReverseIntake(CommandBase):
    def __init__(self, intake: IntakeSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.intake = intake
        self.addRequirements(intake)

    def initialize(self) -> None:
        print(f"Command: {self.getName()}")
        self.intake.toggleReverseIntake()

        if self.intake.isIntakeDeployed():
            self.intake.runIntake(self.intake.intakeReversed)

    # pylint: disable=unused-argument,no-self-use
    def end(self, _interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return True
