from commands2 import CommandBase
from subsystems.indexersubsystem import IndexerSubsystem


class ReverseIndexer(CommandBase):
    def __init__(self, indexer: IndexerSubsystem):
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.indexer = indexer
        self.addRequirements([self.indexer])

    def initialize(self) -> None:
        print(f"Command: {self.getName()}")

    def execute(self) -> None:
        self.indexer.reversePath()

    def end(self, _interrupted: bool) -> None:
        print("... DONE")

    def isFinished(self) -> bool:
        return True
