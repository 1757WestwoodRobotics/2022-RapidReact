from commands2 import CommandBase
from setuptools import Command

from subsystems.indexersubsystem import IndexerSubsystem


class RunIndexer(CommandBase):
    def __init__(self, indexer: IndexerSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.indexer = indexer
        self.addRequirements(indexer)

    def initialize(self) -> None:
        print("running indexer")
        self.indexer.runIndexer()
