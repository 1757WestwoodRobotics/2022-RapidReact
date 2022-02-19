from commands2 import CommandBase
from subsystems.indexersubsystem import IndexerSubsystem

from subsystems.intakesubsystem import IntakeSubsystem


class ReverseBallPath(CommandBase):
    def __init__(self, intake: IntakeSubsystem, indexer: IndexerSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.intake = intake
        self.indexer = indexer
        self.addRequirements([self.intake, self.indexer])

    def initialize(self) -> None:
        print(f"Command: {self.getName()}")
        self.intake.toggleReverseIntake()
        self.indexer.toggleReverseBallPath()

        if self.intake.isIntakeDeployed():
            self.intake.runIntake()
            self.indexer.runIndexer()
            self.indexer.runStaging()

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return True
