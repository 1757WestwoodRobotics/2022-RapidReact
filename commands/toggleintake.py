from commands2 import CommandBase

from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.indexersubsystem import IndexerSubsystem


class ToggleIntake(CommandBase):
    def __init__(self, intake: IntakeSubsystem, indexer: IndexerSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.intake = intake
        self.indexer = indexer
        self.addRequirements(intake)

    def initialize(self) -> None:
        print(f"Command: {self.getName()}")
        self.intake.toggleIntake()
        self.indexer.toggleIndexerSystem()

    def end(self, _interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return True
