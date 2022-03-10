from commands2 import CommandBase
from subsystems.indexersubsystem import IndexerSubsystem


class DefaultIndexer(CommandBase):
    def __init__(self, indexer: IndexerSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.indexer = indexer
        self.addRequirements([self.indexer])

    def execute(self) -> None:
        self.indexer.defaultIndexer()
