from commands2 import CommandBase

from subsystems.indexersubsystem import IndexerSubsystem


class runIndexer(CommandBase):
    def __init__(self, indexer: IndexerSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.indexer = indexer
        self.addRequirements(indexer)

    def execute(self) -> None:
        self.indexer.runIndexer()

    # pylint: disable=unused-argument,no-self-use
    def end(self, _interrupted: bool) -> None:
        self.indexer.stopIndexer()

    def isFinished(self) -> bool:
        return True
