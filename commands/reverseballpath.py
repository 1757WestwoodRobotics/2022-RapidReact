from commands2 import ParallelCommandGroup

from commands.indexer.reverseindexer import ReverseIndexer
from commands.intake.reverseintake import ReverseIntake
from subsystems.indexersubsystem import IndexerSubsystem

from subsystems.intakesubsystem import IntakeSubsystem


class ReverseBallPath(ParallelCommandGroup):
    def __init__(self, intake: IntakeSubsystem, indexer: IndexerSubsystem) -> None:
        super().__init__(ReverseIndexer(indexer), ReverseIntake(intake))
