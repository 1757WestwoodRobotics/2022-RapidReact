from commands2 import ParallelCommandGroup

from commands.indexer.feedforward import FeedForward

from subsystems.indexersubsystem import IndexerSubsystem


class ShootBall(ParallelCommandGroup):
    def __init__(self, indexer: IndexerSubsystem):
        super().__init__(FeedForward(indexer))
        self.setName(__class__.__name__)
