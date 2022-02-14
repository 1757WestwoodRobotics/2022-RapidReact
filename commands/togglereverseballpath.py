from commands2 import CommandBase
from subsystems.indexersubsystem import IndexerSubsystem

from subsystems.intakesubsystem import IntakeSubsystem


class ReverseBallPath(CommandBase):
    def __init__(self, intake: IntakeSubsystem, indexer: IndexerSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.intake = intake
        self.indexer = indexer
        self.addRequirements(intake)
        self.addRequirements(indexer)

    def initialize(self) -> None:
        print(f"Command: {self.getName()}")
        self.intake.toggleReverseIntake()
        self.indexer.toggleReverseBallPath()

        if self.intake.isIntakeDeployed():
            self.intake.runIntake(self.intake.intakeReversed)
            self.indexer.runIndexer(self.indexer.indexerReversed)
            self.indexer.runStaging(self.indexer.stagingReversed)

    # pylint: disable=unused-argument,no-self-use
    def end(self, _interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return True
